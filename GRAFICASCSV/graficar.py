#!/usr/bin/env python3
"""
graficar.py - Convierte los logs CSV del Kestrel (ESP32-CAM + MPU + DHT) en graficas y un resumen.

USO (desde la carpeta con los CSV, o indicando carpetas/archivos):
    python graficar.py                      # todos los CSV de la carpeta actual
    python graficar.py D:\\logs              # todos los CSV de otra carpeta
    python graficar.py log_009.csv          # un archivo concreto
    python graficar.py --forzar             # regenera aunque el CSV no haya cambiado
    python graficar.py --sin-detalle        # sin la figura de zoom del evento principal

SALIDA (carpeta "graficas", se puede cambiar con --salida):
    <log>_resumen.png   figura completa: aceleracion, giroscopio, orientacion, humedad/temperatura, calidad
    <log>_detalle.png   zoom automatico al evento mas intenso (si lo hay)
    resumen_logs.csv    una fila por log con las metricas principales
    .cache_graficas.json  recuerda que logs ya se graficaron: no se rehacen si el CSV no cambio

QUE SE GRAFICA Y POR QUE (no hay panel de velocidad):
    Integrar el acelerometro para sacar velocidad no funciona sin GPS: la gravedad (1 g constante) y el
    sesgo del sensor hacen que el error crezca sin limite (1 g mal restado = 9.8 m/s de error cada segundo).
    En su lugar se muestran magnitudes que SI se pueden medir: aceleracion, velocidad angular y una
    orientacion estimada (roll/pitch). Cuando el GPS este integrado, se anade un panel de velocidad real.

Requiere: pandas, numpy, matplotlib.
"""

import argparse
import json
import sys
import textwrap
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")  # sin ventanas: solo guarda archivos
import matplotlib.pyplot as plt

SCRIPT_VERSION = "2"  # cambia este numero si modificas el script y quieres que se regeneren todas las graficas

# ----------------------------------------------------------------------------------------------
# CONFIGURACION
# ----------------------------------------------------------------------------------------------
# Como esta montado el modulo MPU en el avion: a que eje del sensor corresponde cada eje del avion.
# (adelante, derecha, abajo). Valores: "+x", "-x", "+y", "-y", "+z", "-z".
# Por defecto: modulo plano con el eje Z hacia ARRIBA y el eje X hacia ADELANTE -> ("+x", "-y", "-z").
# Cambialo cuando montes el modulo; el roll/pitch estimado depende de esto.
MONTAJE = ("+x", "-y", "-z")

RANGO_ACEL_G = 16.0       # rango del acelerometro configurado en el logger (+-16 g)
RANGO_GIRO_DPS = 1000.0   # rango del giroscopio configurado en el logger (+-1000 dps)

# Si el MPU pierde la configuracion y vuelve a +-2 g / +-250 dps (paso en log_009), los valores salen
# 8 veces (acel) y 4 veces (giro) mas grandes. Se detecta y se corrige automaticamente.
CORREGIR_ESCALA = True
FACTOR_ACEL_DEFECTO = RANGO_ACEL_G / 2.0     # 8
FACTOR_GIRO_DPS_DEFECTO = RANGO_GIRO_DPS / 250.0  # 4

ALFA_FILTRO = 0.98            # filtro complementario: peso del giroscopio
UMBRAL_ACTIVIDAD_G = 0.25     # |a| - 1 g mayor que esto = hay movimiento
UMBRAL_ACTIVIDAD_DPS = 40.0   # velocidad angular mayor que esto = hay movimiento
MARGEN_EVENTO_S = 3.0         # segundos antes/despues del evento en la figura de detalle
HUECO_MALO_MS = 50.0          # un intervalo entre muestras mayor que esto cuenta como hueco

COLS_MPU = ["ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]
COLS_DHT = ["hum_pct", "temp_c"]


# ----------------------------------------------------------------------------------------------
# UTILIDADES
# ----------------------------------------------------------------------------------------------
def tramos(mascara, t):
    """Devuelve lista de (t_inicio, t_fin) para cada racha contigua de True."""
    mascara = np.asarray(mascara, dtype=bool)
    if not mascara.any():
        return []
    cambios = np.diff(mascara.astype(int), prepend=0, append=0)
    inicios = np.where(cambios == 1)[0]
    fines = np.where(cambios == -1)[0] - 1
    return [(t[i], t[j]) for i, j in zip(inicios, fines)]


def _wrap(grados):
    return (grados + 180.0) % 360.0 - 180.0


def matriz_montaje(montaje):
    """Matriz que pasa un vector del sistema del sensor al del avion (adelante, derecha, abajo)."""
    idx = {"x": 0, "y": 1, "z": 2}
    m = np.zeros((3, 3))
    for fila, s in enumerate(montaje):
        signo = -1.0 if s.startswith("-") else 1.0
        m[fila, idx[s[-1].lower()]] = signo
    if np.linalg.det(m) < 0:
        print("  AVISO: MONTAJE no es una rotacion valida (adelante x derecha debe dar abajo). "
              "El roll/pitch saldra invertido.")
    return m


# ----------------------------------------------------------------------------------------------
# CARGA Y LIMPIEZA
# ----------------------------------------------------------------------------------------------
def cargar(ruta):
    """Lee un CSV y lo deja listo. Devuelve None si no es un log reconocible."""
    try:
        df = pd.read_csv(ruta)
    except Exception as e:  # archivo vacio, dañado...
        print(f"  Se omite ({e})")
        return None
    df.columns = [c.strip() for c in df.columns]
    if "t_ms" not in df.columns:
        print("  Se omite: no tiene columna t_ms")
        return None

    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=["t_ms"]).sort_values("t_ms").drop_duplicates("t_ms").reset_index(drop=True)
    if len(df) < 20:
        print("  Se omite: muy pocas filas")
        return None

    tiene_mpu = all(c in df.columns for c in COLS_MPU)
    tiene_dht = all(c in df.columns for c in COLS_DHT)
    if not (tiene_mpu or tiene_dht):
        print("  Se omite: no tiene columnas de MPU ni de DHT")
        return None

    df["t"] = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0

    # DHT: valores imposibles (p. ej. 947 % de un DHT mal configurado) se descartan
    n_dht_invalidos = 0
    if tiene_dht:
        mala_h = ~df["hum_pct"].between(0, 100) & df["hum_pct"].notna()
        mala_t = ~df["temp_c"].between(-20, 80) & df["temp_c"].notna()
        n_dht_invalidos = int((mala_h | mala_t).sum())
        df.loc[mala_h | mala_t, ["hum_pct", "temp_c"]] = np.nan

    return {"df": df, "mpu": tiene_mpu, "dht": tiene_dht, "dht_invalidos": n_dht_invalidos,
            "tiene_ev": "ev" in df.columns}


def detectar_escala_erronea(mag, t):
    """Marca tramos largos donde |a| en reposo ronda 8 g: el MPU volvio a su escala por defecto.
    Las rachas separadas menos de 10 s se unen (al agitar el modulo la mediana sale un instante de la banda)."""
    med = pd.Series(mag).rolling(101, center=True, min_periods=30).median().to_numpy()
    candidato = (med > 6.0) & (med < 10.0)
    rachas = [list(r) for r in tramos(candidato, t) if r[1] - r[0] >= 3.0]
    unidas = []
    for a, b in rachas:
        if unidas and a - unidas[-1][1] < 10.0:
            unidas[-1][1] = b
        else:
            unidas.append([a, b])
    mascara = np.zeros(len(t), dtype=bool)
    for a, b in unidas:
        mascara |= (t >= a) & (t <= b)
    return mascara


def preparar_mpu(info):
    """Calcula aceleracion/giro finales (con correccion de escala), magnitudes, saturacion y orientacion."""
    df = info["df"]
    t = df["t"].to_numpy()
    acc = df[COLS_MPU[:3]].to_numpy(dtype=float)
    gyr = df[COLS_MPU[3:]].to_numpy(dtype=float)

    # Lecturas "basura": el bus I2C devolvio todo 0xFF (-1 en cada eje) sin dar error. No son datos reales.
    basura = (~np.isnan(acc).any(axis=1) & ~np.isnan(gyr).any(axis=1)
              & (np.abs(acc) < 0.0015).all(axis=1) & (np.abs(gyr) < 0.04).all(axis=1))
    acc[basura] = np.nan
    gyr[basura] = np.nan

    # ax y az en cero exacto durante mucho tiempo = eje muerto, standby o contacto malo
    validas_crudas = ~np.isnan(acc).any(axis=1)
    cero_pct = 100.0 * float(((acc[:, 0] == 0) & (acc[:, 2] == 0) & validas_crudas).sum()) / max(int(validas_crudas.sum()), 1)

    # Saturacion medida sobre los datos CRUDOS (antes de corregir la escala)
    valida = ~(np.isnan(acc).any(axis=1) | np.isnan(gyr).any(axis=1))
    saturada = np.zeros(len(t), dtype=bool)
    saturada[valida] = ((np.abs(acc[valida]).max(axis=1) >= 0.995 * RANGO_ACEL_G) |
                        (np.abs(gyr[valida]).max(axis=1) >= 0.995 * RANGO_GIRO_DPS))

    mag_cruda = np.sqrt(np.nansum(acc ** 2, axis=1))
    mag_cruda[np.isnan(acc).any(axis=1)] = np.nan

    corregida = np.zeros(len(t), dtype=bool)
    if CORREGIR_ESCALA:
        corregida = detectar_escala_erronea(mag_cruda, t)
        if corregida.any():
            acc[corregida] /= FACTOR_ACEL_DEFECTO
            gyr[corregida] /= FACTOR_GIRO_DPS_DEFECTO

    mag = np.sqrt(np.sum(acc ** 2, axis=1))
    mag[np.isnan(acc).any(axis=1)] = np.nan
    gmag = np.sqrt(np.sum(gyr ** 2, axis=1))
    gmag[np.isnan(gyr).any(axis=1)] = np.nan
    # |a| "sostenido": el menor de cada par de muestras consecutivas. Un glitch del bus I2C dura UNA muestra;
    # un golpe o una sacudida real suele durar mas (y el pico de 1 sola muestra puede ser real o no).
    mag2 = np.full(len(mag), np.nan)
    mag2[:-1] = np.minimum(mag[:-1], mag[1:])

    # Sesgo del giroscopio: mediana de los primeros 3 s si el modulo estaba quieto
    sesgo = np.zeros(3)
    ini = (t < 3.0) & ~np.isnan(gyr).any(axis=1)
    if ini.sum() >= 100 and np.nanstd(mag[ini]) < 0.03 and np.nanmax(np.nanstd(gyr[ini], axis=0)) < 5.0:
        sesgo = np.nanmedian(gyr[ini], axis=0)

    # Orientacion estimada (filtro complementario) en ejes del avion
    m = matriz_montaje(MONTAJE)
    acc_b = acc @ m.T
    gyr_b = gyr @ m.T
    sesgo_b = m @ sesgo
    roll, pitch = estimar_orientacion(t, acc_b, gyr_b, sesgo_b)

    info.update(t=t, acc=acc, gyr=gyr, mag=mag, mag2=mag2, gmag=gmag, saturada=saturada, corregida=corregida,
                sesgo=sesgo, roll=roll, pitch=pitch, basura=int(basura.sum()), cero_pct=cero_pct)


def estimar_orientacion(t, acc_b, gyr_b, sesgo_b):
    """Roll (ala derecha abajo = +) y pitch (nariz arriba = +) con filtro complementario.
    El yaw no se puede estimar sin magnetometro. Es una ESTIMACION: en maniobras fuertes se apoya solo
    en el giroscopio y deriva."""
    n = len(t)
    roll = np.full(n, np.nan)
    pitch = np.full(n, np.nan)
    af, ar, ad = acc_b[:, 0], acc_b[:, 1], acc_b[:, 2]
    p = gyr_b[:, 0] - sesgo_b[0]   # velocidad de roll
    q = gyr_b[:, 1] - sesgo_b[1]   # velocidad de pitch
    mag = np.sqrt(af ** 2 + ar ** 2 + ad ** 2)
    roll_acc = np.degrees(np.arctan2(-ar, -ad))
    pitch_acc = np.degrees(np.arctan2(af, np.hypot(ar, ad)))

    r_est = p_est = None
    t_prev = None
    for i in range(n):
        if np.isnan(mag[i]) or np.isnan(p[i]) or np.isnan(q[i]):
            continue
        if r_est is None or (t[i] - t_prev) > 0.25:      # primer dato o hueco largo: reiniciar con el acelerometro
            r_est, p_est = roll_acc[i], pitch_acc[i]
        else:
            dt = t[i] - t_prev
            r_pred = r_est + p[i] * dt
            p_pred = p_est + q[i] * dt
            k = (1.0 - ALFA_FILTRO) if abs(mag[i] - 1.0) < 0.25 else 0.0   # con aceleracion fuerte, no fiarse del acelerometro
            r_est = r_pred + k * _wrap(roll_acc[i] - r_pred)
            p_est = p_pred + k * (pitch_acc[i] - p_pred)
        t_prev = t[i]
        roll[i], pitch[i] = _wrap(r_est), p_est
    return roll, pitch


def detectar_evento(info):
    """Busca el tramo de movimiento con el mayor pico de |a|. Devuelve (t0, t1) o None."""
    t, mag, gmag, mag2 = info["t"], info["mag"], info["gmag"], info["mag2"]
    activo = (np.abs(mag - 1.0) > UMBRAL_ACTIVIDAD_G) | (gmag > UMBRAL_ACTIVIDAD_DPS)
    activo = np.nan_to_num(activo.astype(float)) > 0
    activo = pd.Series(activo.astype(float)).rolling(51, center=True, min_periods=1).max().to_numpy() > 0
    rachas = tramos(activo, t)
    if not rachas:
        return None
    # unir rachas separadas menos de 2 s
    unidas = [list(rachas[0])]
    for a, b in rachas[1:]:
        if a - unidas[-1][1] < 2.0:
            unidas[-1][1] = b
        else:
            unidas.append([a, b])
    mejor, pico_mejor = None, -1.0
    for a, b in unidas:
        sel = (t >= a) & (t <= b)
        pico = np.nanmax(mag2[sel]) if np.isfinite(mag2[sel]).any() else -1.0
        if pico > pico_mejor:
            mejor, pico_mejor = (a, b), pico
    if mejor is None:
        return None
    return max(t[0], mejor[0] - MARGEN_EVENTO_S), min(t[-1], mejor[1] + MARGEN_EVENTO_S)


# ----------------------------------------------------------------------------------------------
# METRICAS
# ----------------------------------------------------------------------------------------------
def calcular_metricas(nombre, info):
    df = info["df"]
    t = df["t"].to_numpy()
    dur = float(t[-1])
    dt_ms = np.diff(df["t_ms"].to_numpy())
    fila = {
        "archivo": nombre,
        "formato": "completo" if (info["mpu"] and info["dht"]) else ("solo_mpu" if info["mpu"] else "solo_dht"),
        "filas": len(df),
        "duracion_s": round(dur, 1),
        "hz_medio": round(len(df) / dur, 1) if dur > 0 else np.nan,
        "hueco_max_ms": float(dt_ms.max()) if len(dt_ms) else np.nan,
        "huecos_mayores_50ms": int((dt_ms > HUECO_MALO_MS).sum()) if info["mpu"] else "",
    }
    if info["mpu"]:
        mag, gmag = info["mag"], info["gmag"]
        validas = ~np.isnan(mag)
        quieto = pd.Series(mag).rolling(101, min_periods=50).std().to_numpy() < 0.02
        reposo = np.nanmedian(mag[quieto & validas]) if (quieto & validas).sum() > 100 else np.nan
        i_pico = int(np.nanargmax(mag)) if validas.any() else 0
        fila.update({
            "lecturas_fallidas_pct": round(100.0 * (~validas).mean(), 2),
            "eventos_config": int(np.nanmax(df["ev"])) if info["tiene_ev"] and df["ev"].notna().any() else "",
            "pico_acel_g": round(float(mag[i_pico]), 2) if validas.any() else np.nan,
            "t_pico_s": round(float(t[i_pico]), 2) if validas.any() else np.nan,
            "pico_sostenido_g": round(float(np.nanmax(info["mag2"])), 2) if np.isfinite(info["mag2"]).any() else np.nan,
            "pico_giro_dps": round(float(np.nanmax(gmag)), 0) if validas.any() else np.nan,
            "acel_reposo_g": round(float(reposo), 3) if np.isfinite(reposo) else "",
            "lecturas_basura": info["basura"],
            "ax_az_cero_pct": round(info["cero_pct"], 1),
            "muestras_saturadas": int(info["saturada"].sum()),
            "tramo_escala_corregida_s": round(float(sum(b - a for a, b in tramos(info["corregida"], t))), 1),
        })
    if info["dht"]:
        fila.update({
            "hum_min": df["hum_pct"].min(), "hum_max": df["hum_pct"].max(),
            "temp_min": df["temp_c"].min(), "temp_max": df["temp_c"].max(),
        })
    ev = info.get("evento")
    fila["evento_t0_s"] = round(ev[0], 1) if ev else ""
    fila["evento_t1_s"] = round(ev[1], 1) if ev else ""
    return fila


# ----------------------------------------------------------------------------------------------
# GRAFICAS
# ----------------------------------------------------------------------------------------------
def _decorar(ax, titulo, ylabel):
    ax.set_title(titulo, loc="left", fontsize=10, fontweight="bold")
    ax.set_ylabel(ylabel, fontsize=9)
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.tick_params(labelsize=8)


def _marcar_tramos(ejes, info, t):
    """Sombrea tramos con escala corregida y dibuja lineas donde el MPU perdio su configuracion."""
    corr = tramos(info["corregida"], info["t"]) if info["mpu"] else []
    ev_t = []
    df = info["df"]
    if info["tiene_ev"]:
        ev = df["ev"].ffill().fillna(0).to_numpy()
        ev_t = df["t"].to_numpy()[np.where(np.diff(ev, prepend=ev[0]) > 0)[0]]
    for ax in ejes:
        for a, b in corr:
            ax.axvspan(a, b, color="orange", alpha=0.12, lw=0)
        for x in ev_t:
            ax.axvline(x, color="red", linestyle="--", linewidth=1.0, alpha=0.8)


def dibujar(info, nombre, ruta_png, ventana=None, dpi=150):
    """Dibuja la figura. Si 'ventana'=(t0,t1) se hace zoom a ese intervalo (y se reescalan los ejes)."""
    df = info["df"]
    t_all = df["t"].to_numpy()
    sel = np.ones(len(t_all), dtype=bool) if ventana is None else (t_all >= ventana[0]) & (t_all <= ventana[1])
    t = t_all[sel]

    paneles = []
    if info["mpu"]:
        paneles += ["acel", "giro", "orient"]
    if info["dht"] and ventana is None:
        paneles.append("dht")
    if ventana is None and info["mpu"]:
        paneles.append("calidad")

    n = len(paneles)
    fig, axes = plt.subplots(n, 1, figsize=(12, 2.7 * n + 0.9), sharex=True, constrained_layout=True)
    axes = np.atleast_1d(axes)

    for ax, nombre_panel in zip(axes, paneles):
        if nombre_panel == "acel":
            acc = info["acc"][sel]
            mag = info["mag"][sel]
            for k, (etq, col) in enumerate(zip(("ax", "ay", "az"), ("tab:blue", "tab:orange", "tab:green"))):
                ax.plot(t, acc[:, k], label=etq, color=col, linewidth=0.8, alpha=0.85)
            ax.plot(t, mag, label="|a| total", color="black", linewidth=1.3)
            ax.axhline(1.0, color="gray", linestyle=":", linewidth=1.0)
            if np.isfinite(mag).any():
                i = int(np.nanargmax(mag))
                sost = np.nanmax(info["mag2"][sel]) if np.isfinite(info["mag2"][sel]).any() else mag[i]
                aislado = mag[i] > 2.0 and mag[i] > 1.8 * sost
                texto = f"pico {mag[i]:.1f} g, t = {t[i]:.2f} s"
                if aislado:
                    texto += f"\n(1 sola muestra; sostenido {sost:.1f} g:\n puede ser real o un glitch del bus)"
                ax.plot(t[i], mag[i], "v", color="red", markersize=8)
                ax.annotate(texto, xy=(t[i], mag[i]), xytext=(8, -4), textcoords="offset points",
                            fontsize=8, color="red", va="top")
            ax.legend(loc="upper right", fontsize=8, ncol=4)
            _decorar(ax, "Aceleracion (g)   [en reposo |a| = 1 g]", "g")
        elif nombre_panel == "giro":
            gyr = info["gyr"][sel]
            for k, (etq, col) in enumerate(zip(("gx", "gy", "gz"), ("tab:blue", "tab:orange", "tab:green"))):
                ax.plot(t, gyr[:, k], label=etq, color=col, linewidth=0.8, alpha=0.9)
            ax.legend(loc="upper right", fontsize=8, ncol=3)
            _decorar(ax, "Velocidad angular (grados/s)", "grados/s")
        elif nombre_panel == "orient":
            ax.plot(t, info["roll"][sel], label="roll (+ = ala derecha abajo)", color="tab:purple", linewidth=1.0)
            ax.plot(t, info["pitch"][sel], label="pitch (+ = nariz arriba)", color="tab:red", linewidth=1.0)
            ax.axhline(0.0, color="gray", linestyle=":", linewidth=1.0)
            ax.legend(loc="upper right", fontsize=8, ncol=2)
            _decorar(ax, f"Orientacion ESTIMADA (filtro complementario; montaje adelante={MONTAJE[0]}, "
                         f"derecha={MONTAJE[1]}, abajo={MONTAJE[2]}; sin yaw)", "grados")
        elif nombre_panel == "dht":
            h = df["hum_pct"].to_numpy()[sel]
            c = df["temp_c"].to_numpy()[sel]
            ax.plot(t, h, color="tab:blue", linewidth=1.2, label="humedad (%)")
            ax.set_ylabel("humedad (%)", fontsize=9, color="tab:blue")
            ax2 = ax.twinx()
            ax2.plot(t, c, color="crimson", linewidth=1.2, label="temperatura (C)")
            ax2.set_ylabel("temperatura (C)", fontsize=9, color="crimson")
            ax2.tick_params(labelsize=8)
            ax.set_title("Humedad y temperatura (DHT)   [la temperatura incluye el calor de la placa]",
                         loc="left", fontsize=10, fontweight="bold")
            ax.grid(True, linestyle="--", alpha=0.5)
            ax.tick_params(labelsize=8)
        elif nombre_panel == "calidad":
            dt_ms = np.diff(df["t_ms"].to_numpy())
            ax.plot(t_all[1:], dt_ms, drawstyle="steps-post", color="tab:gray", linewidth=0.9)
            techo = float(np.clip(dt_ms.max() * 1.1, 30, 1100))
            ax.set_ylim(0, techo)
            if info["mpu"]:
                perdidas = t_all[np.isnan(info["acc"]).any(axis=1)]
                if len(perdidas):
                    ax.vlines(perdidas, 0, techo * 0.25, color="red", alpha=0.6, linewidth=0.8,
                              label=f"lectura fallida ({len(perdidas)})")
                    ax.legend(loc="upper right", fontsize=8)
            _decorar(ax, "Calidad del registro: intervalo entre muestras (ms)", "ms")

    axes[-1].set_xlabel("Tiempo desde el arranque (s)", fontsize=10)
    _marcar_tramos(axes, info, t)
    if ventana is not None:
        axes[0].set_xlim(ventana)
    else:
        axes[0].set_xlim(t_all[0], t_all[-1])

    dur = t_all[-1]
    hz = len(df) / dur
    sub = [f"{dur:.0f} s", (f"{hz:.0f} Hz" if hz >= 10 else f"{hz:.1f} Hz")]
    if info["mpu"]:
        if info["cero_pct"] > 20:
            sub.append(f"ATENCION: ax y az en cero exacto el {info['cero_pct']:.0f} % del tiempo (sensor mal configurado o contacto malo)")
        if info["basura"]:
            sub.append(f"{info['basura']} lecturas basura (todo 0xFF) descartadas")
        if info["saturada"].any():
            sub.append(f"{int(info['saturada'].sum())} muestras saturadas")
        if info["corregida"].any():
            tr = sum(b - a for a, b in tramos(info["corregida"], info["t"]))
            sub.append(f"zona naranja ({tr:.0f} s): el MPU volvio a su escala de fabrica, datos corregidos (acel/8, giro/4)")
        if info["tiene_ev"] and df["ev"].fillna(0).max() > 0:
            sub.append(f"lineas rojas: el MPU perdio la configuracion {int(df['ev'].max())} vez/veces")
    if info["dht_invalidos"]:
        sub.append(f"{info['dht_invalidos']} lecturas DHT imposibles descartadas")
    titulo = nombre if ventana is None else f"{nombre}  -  detalle del evento ({ventana[0]:.1f} a {ventana[1]:.1f} s)"
    subtitulo = textwrap.fill("  |  ".join(sub), width=105)
    fig.suptitle(titulo + "\n" + subtitulo, fontsize=11, fontweight="bold")

    fig.savefig(ruta_png, dpi=dpi)
    plt.close(fig)


# ----------------------------------------------------------------------------------------------
# PROGRAMA PRINCIPAL
# ----------------------------------------------------------------------------------------------
def recoger_csv(entradas, carpeta_salida):
    archivos = []
    for e in entradas:
        p = Path(e)
        if p.is_dir():
            archivos += sorted(p.glob("*.csv"))
        elif p.is_file():
            archivos.append(p)
        else:
            print(f"No existe: {p}")
    salida = carpeta_salida.resolve()
    resultado = []
    for a in archivos:
        if a.name == "resumen_logs.csv" or salida in a.resolve().parents:
            continue
        resultado.append(a)
    return resultado


def main():
    ap = argparse.ArgumentParser(description="Grafica los logs CSV del Kestrel.")
    ap.add_argument("entradas", nargs="*", default=["."], help="carpetas o archivos CSV (por defecto: la carpeta actual)")
    ap.add_argument("--salida", default="graficas", help="carpeta donde se guardan las imagenes (por defecto: graficas)")
    ap.add_argument("--forzar", action="store_true", help="regenerar aunque el CSV no haya cambiado")
    ap.add_argument("--sin-detalle", action="store_true", help="no generar la figura de detalle del evento")
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()

    carpeta_salida = Path(args.salida)
    carpeta_salida.mkdir(parents=True, exist_ok=True)
    archivos = recoger_csv(args.entradas, carpeta_salida)
    if not archivos:
        print("No se encontro ningun archivo .csv.")
        sys.exit(1)

    ruta_cache = carpeta_salida / ".cache_graficas.json"
    cache = {}
    if ruta_cache.exists():
        try:
            cache = json.loads(ruta_cache.read_text(encoding="utf-8"))
        except Exception:
            cache = {}

    print(f"Se encontraron {len(archivos)} archivos CSV. Salida: {carpeta_salida.resolve()}\n")
    filas = {}
    hechos = saltados = omitidos = 0

    for ruta in archivos:
        nombre = ruta.stem
        st = ruta.stat()
        firma = f"{st.st_size}_{int(st.st_mtime)}"
        png = carpeta_salida / f"{nombre}_resumen.png"

        if (not args.forzar and cache.get(ruta.name) == {"firma": firma, "version": SCRIPT_VERSION} and png.exists()):
            print(f"[=] {ruta.name}: sin cambios, se omite (usa --forzar para regenerar)")
            saltados += 1
            continue

        print(f"[>] {ruta.name}")
        info = cargar(ruta)
        if info is None:
            omitidos += 1
            continue
        if info["mpu"]:
            preparar_mpu(info)
            info["evento"] = detectar_evento(info)
        else:
            info["evento"] = None

        dibujar(info, nombre, png, dpi=args.dpi)
        print(f"    guardado: {png.name}")
        if info["evento"] is not None and not args.sin_detalle:
            png_det = carpeta_salida / f"{nombre}_detalle.png"
            dibujar(info, nombre, png_det, ventana=info["evento"], dpi=args.dpi)
            print(f"    guardado: {png_det.name}  (evento {info['evento'][0]:.1f}-{info['evento'][1]:.1f} s)")

        filas[ruta.name] = calcular_metricas(ruta.name, info)
        cache[ruta.name] = {"firma": firma, "version": SCRIPT_VERSION}
        hechos += 1

    # Resumen: se actualizan las filas procesadas y se conservan las anteriores
    ruta_res = carpeta_salida / "resumen_logs.csv"
    if filas:
        nuevo = pd.DataFrame(list(filas.values()))
        if ruta_res.exists():
            try:
                previo = pd.read_csv(ruta_res)
                previo = previo[~previo["archivo"].isin(nuevo["archivo"])]
                nuevo = pd.concat([previo, nuevo], ignore_index=True)
            except Exception:
                pass
        nuevo.sort_values("archivo").to_csv(ruta_res, index=False)
    ruta_cache.write_text(json.dumps(cache, indent=1), encoding="utf-8")

    print(f"\nListo: {hechos} graficados, {saltados} sin cambios, {omitidos} omitidos.")
    if ruta_res.exists():
        print(f"Resumen de todos los logs: {ruta_res}")


if __name__ == "__main__":
    main()
