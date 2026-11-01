Write-Host "Subiendo al Pico..." -ForegroundColor Cyan
py -m mpremote connect COM5 cp firmware\rp2040\main.py :main.py
Write-Host "Listo." -ForegroundColor Green
