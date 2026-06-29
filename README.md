# SIVERT2 Unity Simulation Framework

This repository contains the Unity3D part of the SIVERT2 simulation framework, developed within the FFI VINNOVA SIVERT2 research project. It focuses on 3D scene management, agent dynamics, and channel modeling for vehicular system-level simulations (V2N/V2I).

The second part of the framework is an adapted NS-3 module enabling vehicular system-level simulations and network interaction. That module is hosted in a separate repository.

- Engine: Unity 2022.3.13f1 (LTS)
- License: MIT (see `LICENSE`)

## Getting started

- Clone the repo and open the project in Unity 2022.3 LTS or later.
- On first open, Unity will regenerate Library/ and project files locally.

## Repository layout

- `Assets/` Unity assets and C# scripts
- `Packages/` Unity package manifest/lock
- `ProjectSettings/` Unity project settings
- `NS3_Logs/`, `H_Output/` output folders (ignored by git)
- `EMI_Measured_Data/` measurement data (large raw data; CSV/PNG kept, other formats ignored)

For questions, please contact:
- Aleksei Fedorov — aleksei.fedorov@eit.lth.se
- Nikita Lyamin — nikita.lyamin@volvocars.com
