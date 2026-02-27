import json
import os
from dataclasses import dataclass

import numpy as np


DEFAULT_STRUCTURAL_CONFIG_PATH = os.path.join(
    os.path.dirname(__file__), "config", "structural_constants_w26.json"
)


def _require(mapping, key, path):
    if key not in mapping:
        raise ValueError(f"Missing required key '{path}.{key}' in structural config")
    return mapping[key]


def _as_vec3(value, name):
    arr = np.asarray(value, dtype=float)
    if arr.shape != (3,):
        raise ValueError(f"{name} must be length-3, got shape {arr.shape}")
    return arr


def _normalize(vec, name):
    norm = float(np.linalg.norm(vec))
    if norm < 1e-12:
        raise ValueError(f"{name} has near-zero norm and cannot be normalized")
    return vec / norm


def _orthonormal_basis_from_raw(ix_raw, iy_raw):
    e1 = _normalize(ix_raw, "principal_axes.ix")

    iy_proj = iy_raw - float(np.dot(iy_raw, e1)) * e1
    e2 = _normalize(iy_proj, "principal_axes.iy after projection")

    e3 = np.cross(e1, e2)
    e3 = _normalize(e3, "derived principal_axes.iz")

    # Recompute e2 to remove numeric drift and enforce right-handed basis.
    e2 = _normalize(np.cross(e3, e1), "re-orthogonalized principal_axes.iy")

    return np.column_stack((e1, e2, e3))


def _build_inertia_tensor(r_bp, moments_kg_m2):
    d = np.diag(moments_kg_m2)
    inertia = r_bp @ d @ r_bp.T
    inertia = 0.5 * (inertia + inertia.T)

    eigvals = np.linalg.eigvalsh(inertia)
    if np.any(eigvals <= 0.0):
        raise ValueError(
            "Inertia tensor is not positive-definite. "
            f"Eigenvalues={eigvals.tolist()}"
        )
    return inertia


@dataclass
class StructuralConstants:
    source_doc_path: str
    semimajor_axis_m: float
    inclination_deg: float
    period_s: float
    raan_deg: float
    center_of_mass_mm: np.ndarray
    principal_moments_kg_m2: np.ndarray
    principal_axes_body: np.ndarray
    inertia_tensor_body: np.ndarray



def load_structural_constants(config_path=None):
    path = config_path or DEFAULT_STRUCTURAL_CONFIG_PATH
    if not os.path.exists(path):
        raise FileNotFoundError(f"Structural config not found: {path}")

    with open(path, "r", encoding="utf-8") as fh:
        raw = json.load(fh)

    metadata = _require(raw, "metadata", "root")
    raw_source = _require(raw, "raw_source_values", "root")
    si_values = _require(raw, "si_values", "root")

    source_doc_path = str(_require(metadata, "source_doc_path", "metadata"))

    orbit_si = _require(si_values, "orbit", "si_values")
    semimajor_axis_m = float(_require(orbit_si, "semimajor_axis_m", "si_values.orbit"))
    inclination_deg = float(_require(orbit_si, "inclination_deg", "si_values.orbit"))
    period_s = float(_require(orbit_si, "period_s", "si_values.orbit"))
    raan_deg = float(orbit_si.get("raan_deg", 0.0))

    com_mm_raw = _require(raw_source, "center_of_mass_mm", "raw_source_values")
    center_of_mass_mm = np.array(
        [
            float(_require(com_mm_raw, "x", "raw_source_values.center_of_mass_mm")),
            float(_require(com_mm_raw, "y", "raw_source_values.center_of_mass_mm")),
            float(_require(com_mm_raw, "z", "raw_source_values.center_of_mass_mm")),
        ],
        dtype=float,
    )

    moments_si = _require(si_values, "principal_moments_kg_m2", "si_values")
    principal_moments_kg_m2 = np.array(
        [
            float(_require(moments_si, "px", "si_values.principal_moments_kg_m2")),
            float(_require(moments_si, "py", "si_values.principal_moments_kg_m2")),
            float(_require(moments_si, "pz", "si_values.principal_moments_kg_m2")),
        ],
        dtype=float,
    )

    axes_raw = _require(raw_source, "principal_axes", "raw_source_values")
    ix_raw = _as_vec3(_require(axes_raw, "ix", "raw_source_values.principal_axes"), "principal_axes.ix")
    iy_raw = _as_vec3(_require(axes_raw, "iy", "raw_source_values.principal_axes"), "principal_axes.iy")

    principal_axes_body = _orthonormal_basis_from_raw(ix_raw, iy_raw)
    inertia_tensor_body = _build_inertia_tensor(principal_axes_body, principal_moments_kg_m2)

    return StructuralConstants(
        source_doc_path=source_doc_path,
        semimajor_axis_m=semimajor_axis_m,
        inclination_deg=inclination_deg,
        period_s=period_s,
        raan_deg=raan_deg,
        center_of_mass_mm=center_of_mass_mm,
        principal_moments_kg_m2=principal_moments_kg_m2,
        principal_axes_body=principal_axes_body,
        inertia_tensor_body=inertia_tensor_body,
    )
