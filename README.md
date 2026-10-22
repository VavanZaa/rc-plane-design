# V-201 RC Plane Design Pipeline

I built a computational aircraft-design workflow connecting a full-aircraft aerodynamic baseline, constrained multipoint wing optimization, geometry export and Autodesk Fusion import tools. This source package focuses on the V-201 pipeline rather than historical V-101 models and generated reports.

Developed by [Vavan Farizzaman](https://github.com/VavanZaa).

## Algorithm

1. Load explicit geometry and operating-condition configuration, convert millimetres to SI units, and construct the baseline aircraft in AeroSandbox.
2. Trim the complete aircraft at cruise and evaluate declared static-stability derivatives.
3. Parameterize the outboard planform, twist, dihedral and a smooth Kulfan airfoil field while retaining fixed interface constraints.
4. Optimize multipoint objectives with NeuralFoil/AeroSandbox and the configured IPOPT backend.
5. Perform separate numerical checks for mesh convergence, section stall order, trim/stability and the assumed structural load requirement.
6. Export geometry/reference files and construct wing/fuselage/tail components through the Fusion add-ins; keep XFLR5 export as a separate comparison path.

## Workflow

```mermaid
flowchart LR
Config["Geometry and flight-condition JSON"] --> Baseline["Full-aircraft baseline and trim"] --> Optimize["Constrained Kulfan / NeuralFoil optimization"] --> Check["Convergence, stability and load checks"] --> Export["Geometry and XFLR5 exports"] --> CAD["Fusion import tools"]
```

## Run

Commands below assume a Windows terminal at the repository root unless stated otherwise.

```powershell
python -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python V-201/aerodynamic/v201_baseline.py
# Optimization has additional solver requirements; inspect its configuration first.
# Fusion add-ins run inside Autodesk Fusion, not the standalone Python interpreter.
```

## Code map

`V-201/aerodynamic/`: baseline, optimization configuration and exports; `V-201/fusion_addin/`: CAD construction/import scripts; `tools/`: format/geometry helpers.

## Scope and evidence

Aerodynamic and structural checks are conditional models, not flight-test validation. The optimization currently configures IPOPT/MA97 and needs a compatible separately licensed HSL installation; the source contains a machine-specific optional HSL search path. Proprietary solver binaries, CAD installers, historical large F3D files and generated Plotly/CFD output are excluded.

This is a focused source snapshot. Installed dependencies, local environments and bulk generated research assets are rebuilt or supplied separately. See [VALIDATION.md](VALIDATION.md) for the checks performed on this export and any remaining setup requirements.
