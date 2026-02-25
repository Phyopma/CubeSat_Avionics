# W26 Constants Traceability

This document tracks structural/orbit constants loaded by the HITL physics engine from:

- `/Users/phyopyae/eecs159/physics_engine/config/structural_constants_w26.json`

Source reference document path stored in config metadata:

- `/Users/phyopyae/eecs159/CubeSat Stats and Variables W26.pdf`

## Raw Source Values

- Semimajor axis: `6928.14 km`
- Inclination: `97.5977 deg`
- Period: `95.65 min`
- RAAN note: `41.2 deg`
- Center of mass (mm): `x=0.24, y=140.58, z=0.94`
- Principal axes:
  - `ix=(0.00, 1.00, -0.04)`
  - `iy=(-1.00, 0.00, -0.05)`
  - `iz=(-0.05, 0.04, 1.00)`
- Principal moments (g·mm²):
  - `px=11867257.57`
  - `py=12575303.76`
  - `pz=12866690.27`

## Unit Conversions

- `1 g·mm² = 1e-9 kg·m²`

Converted principal moments:

- `px=0.01186725757 kg·m²`
- `py=0.01257530376 kg·m²`
- `pz=0.01286669027 kg·m²`

Converted orbit values:

- Semimajor axis: `6928140.0 m`
- Period: `5739.0 s`

## Inertia Construction Method

The runtime loader builds the body-frame inertia tensor from source principal data:

1. Normalize source axis vectors.
2. Orthonormalize with Gram-Schmidt.
3. Build `R` from orthonormal basis vectors.
4. Build `D = diag(px, py, pz)`.
5. Compute `I_body = R * D * R^T`.
6. Enforce symmetry numerically and validate positive-definite eigenvalues.

## Current Assumptions

- COM values are stored for traceability but not yet applied to translational/gravity-gradient models.
- Magnetic field model still uses a simplified tilted-dipole field with configured orbit period/inclination/RAAN.
- Host and firmware must be version-matched for telemetry packet v2.
