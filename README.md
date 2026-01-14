# ✈️ SYNKRO PLANE – RC Aircraft with FPV

Experimental fixed-wing UAV for testing, learning, and development.
<div align="center">
  <img src="foto%20plane.jpeg" width="800"/>
</div>
NOTE: this black design IS NOT the final one, we´re working on ours, its a test developed by Choisaint to make sure the dimensions and calculations were right, this desing is NOT ours, it belongs to Joyplanes, you can search their channel on YT or their model aircraft courses in their web

---

## 📌 Overview

**SYNKRO-PLANE** is an experimental fixed-wing RC aircraft project developed for educational, testing, and prototyping purposes, integrating FPV (First Person View) and low-cost electronic components.

The main objective is learning by building: basic aerodynamics, flight control, electronics, digital manufacturing, and iterative testing.

This repository documents the entire process: design decisions, technical choices, failures, improvements, and results.

---

## Project Goals

- Design and build a functional RC aircraft from scratch/Easy to find materials  
- Integrate a stable FPV system  
- Perform real flight tests and document them  
- Apply basic aeronautical engineering principles  
- Develop a reproducible and scalable platform  

---

## Key Features

- Configuration: high-wing, tractor motor  
- Conventional RC control  
- FPV-ready architecture  
- Lightweight structure (foam, cardboard, or 3D-printed parts depending on version)  
- Modular design for easy repairs and upgrades  

---

## FPV System

- Analog FPV camera  
- Video transmitter (VTX)  
- Dedicated or shared power supply (depending on iteration)  
- Intended for test flights and first-person visualization  

> FPV is used strictly for experimental purposes and in compliance with local regulations (visual observer / line-of-sight when required).

---

## Electronics

Typical system components (may vary between iterations):

- Brushless motor 1400kv 2212
- ESC  30 A
- Propeller (e.g. 8040 during initial tests)  
- Servos for:
  - Elevator  
  - Rudder / ailerons (depending on configuration)  
- RC receiver (≤10 channels)  
- LiPo battery  
- Independent FPV system
- ESP 32 CAM
- MPU 6050

---

## Design Philosophy

SYnKRO is not a commercial model. It is a flying laboratory.

Every failure is data. Every break is part of the process.

Core principles:

- Rapid iteration  
- Low cost  
- Repairability  
- Honest documentation (including mistakes)  

---

## Project Status

**Actively under development**

- Initial flight tests completed  
- Structural repairs documented  
- 3D-printed propeller testing performed  
- Aerodynamic optimization in progress  
- Progressive FPV integration  

---

## Logbook

All progress, tests, failures, and technical decisions are recorded in the project logbook available in this repository.

Includes:

- Structural changes we´re using foamboard for the final model
- Test results  
- Identified issues  
- Applied solutions  

---
# ⚠️ Disclaimer

This project is experimental.
It is not intended for transport, commercial use, or autonomous operations outside controlled environments.

The author assumes no responsibility for misuse of the design or information provided.

---

# Authors
Santiago Gonzalez R (AKA Choisaint) Mechatronic Engineering student
Miguel Angel G (AKA Mag) Electronic Engineering student
Diego Moreno O (AKA Serper) Mechatronic Engineering Student
## Repository Structure

/docs        → Documentation and logbook
/models      → 3D models and printable parts
/electronics → Diagrams and configurations
/media       → Photos and test videos
