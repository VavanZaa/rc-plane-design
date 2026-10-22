"""V-201 Phase 4: constrained multipoint Kulfan / NeuralFoil optimization.

The optimization keeps the accepted Phase 3 root chord, wing position, spar
locations, and fuselage interface fixed. IPOPT with MA97 optimizes the
outboard planform, twist, dihedral, and a smooth three-control-set Kulfan
airfoil field. Numerical post-processing independently verifies mesh
convergence, section stall order, full-aircraft trim/stability, and the carbon
tube load requirement.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
BASELINE_CONFIG_PATH = SCRIPT_DIR / "v201_baseline_config.json"
PHASE4_CONFIG_PATH = SCRIPT_DIR / "phase4_optimization_config.json"
RESULTS_DIR = SCRIPT_DIR / "results"
OPTIMIZATION_DIR = PROJECT_DIR / "optimization_results"
AIRFOIL_DIR = OPTIMIZATION_DIR / "airfoils"
CAD_REFERENCE_DIR = PROJECT_DIR / "cad_reference"

HSL_PATH = Path(
    r"D:\Apps\CoinHSL.v2024.5.15.x86_64-w64-mingw32-libgfortran5"
    r"\CoinHSL.v2024.5.15.x86_64-w64-mingw32-libgfortran5\bin"
)
if HSL_PATH.exists():
    os.environ["PATH"] = str(HSL_PATH) + os.pathsep + os.environ.get("PATH", "")
    _HSL_DLL_HANDLE = (
        os.add_dll_directory(str(HSL_PATH))
        if hasattr(os, "add_dll_directory")
        else None
    )
else:
    _HSL_DLL_HANDLE = None

import aerosandbox as asb
import aerosandbox.numpy as np
import matplotlib.pyplot as plt
import numpy as onp
from scipy.optimize import brentq


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def scalar(value: Any) -> float:
    return float(onp.asarray(value).reshape(-1)[0])


def json_compatible(value: Any) -> Any:
    """Convert NumPy scalar/array values during JSON serialization."""
    if isinstance(value, onp.generic):
        return value.item()
    if isinstance(value, onp.ndarray):
        return value.tolist()
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def linear_interpolate_stations(
    baseline_stations: list[dict[str, Any]], y_mm: float, key: str
) -> float:
    for inner, outer in zip(baseline_stations[:-1], baseline_stations[1:]):
        if inner["y_mm"] <= y_mm <= outer["y_mm"]:
            fraction = (y_mm - inner["y_mm"]) / (outer["y_mm"] - inner["y_mm"])
            return inner[key] + fraction * (outer[key] - inner[key])
    raise ValueError(f"Station {y_mm} mm is outside the baseline wing.")


def quadratic_bernstein(control_values: Any, eta: float) -> Any:
    return (
        control_values[0] * (1 - eta) ** 2
        + 2 * control_values[1] * eta * (1 - eta)
        + control_values[2] * eta**2
    )


def cubic_bernstein(control_values: Any, eta: float) -> Any:
    return (
        control_values[0] * (1 - eta) ** 3
        + 3 * control_values[1] * eta * (1 - eta) ** 2
        + 3 * control_values[2] * eta**2 * (1 - eta)
        + control_values[3] * eta**3
    )


def integrate_planform(y_m: Any, chords_m: Any) -> tuple[Any, Any]:
    half_area = 0
    half_c2 = 0
    for index in range(len(y_m) - 1):
        dy = y_m[index + 1] - y_m[index]
        c0, c1 = chords_m[index], chords_m[index + 1]
        half_area += 0.5 * (c0 + c1) * dy
        half_c2 += dy * (c0**2 + c0 * c1 + c1**2) / 3
    area = 2 * half_area
    mac = 2 * half_c2 / area
    return area, mac


def make_symbolic_problem(
    baseline: dict[str, Any], phase4: dict[str, Any]
) -> tuple[asb.Opti, dict[str, Any]]:
    opti = asb.Opti()
    conditions = baseline["operating_conditions"]
    constraints = phase4["constraints"]
    regularization = phase4["regularization"]
    method = phase4["method"]
    y_mm = onp.asarray(phase4["optimization_stations_mm"], dtype=float)
    y_m = y_mm / 1000
    eta = y_mm / (baseline["wing"]["span_mm"] / 2)
    span_m = baseline["wing"]["span_mm"] / 1000
    weight_n = (
        baseline["mass_budget"]["design_auw_g"]
        / 1000
        * conditions["gravity_m_s2"]
    )
    rho = conditions["air_density_kg_m3"]
    mu = 1.81e-5

    baseline_stations = baseline["wing"]["stations"]
    initial_chords = onp.asarray(
        [linear_interpolate_stations(baseline_stations, y, "chord_mm") for y in y_mm]
    ) / 1000
    initial_twists = onp.asarray(
        [linear_interpolate_stations(baseline_stations, y, "twist_deg") for y in y_mm]
    )
    chord_bounds = onp.asarray(phase4["chord_bounds_mm"], dtype=float) / 1000
    twist_bounds = onp.asarray(phase4["twist_bounds_deg"], dtype=float)

    chord_controls = opti.variable(init_guess=[0.250, 0.250, 0.220, 0.145])
    chords = np.stack(
        [cubic_bernstein(chord_controls, eta_i) for eta_i in eta]
    )
    twists = opti.variable(init_guess=initial_twists)
    dihedral = opti.variable(
        init_guess=baseline["wing"]["outer_panel_dihedral_deg"]
    )
    opti.subject_to(
        [
            chords >= chord_bounds[:, 0],
            chords <= chord_bounds[:, 1],
            chord_controls[0] == constraints["root_chord_fixed_mm"] / 1000,
            chord_controls[1] >= 0.235,
            chord_controls[1] <= 0.260,
            chord_controls[2] >= 0.180,
            chord_controls[2] <= 0.235,
            chord_controls[3] >= chord_bounds[-1, 0],
            chord_controls[3] <= chord_bounds[-1, 1],
            chord_controls[:-1] >= chord_controls[1:],
            twists >= twist_bounds[:, 0],
            twists <= twist_bounds[:, 1],
            chords[:-1] >= chords[1:],
            twists[:-1] >= twists[1:],
            dihedral >= constraints["outer_panel_dihedral_min_deg"],
            dihedral <= constraints["outer_panel_dihedral_max_deg"],
        ]
    )

    anchor_names = ["naca2414", "naca2413", "naca2412"]
    anchor_airfoils = [asb.Airfoil(name).to_kulfan_airfoil() for name in anchor_names]
    initial_upper = onp.stack([onp.asarray(af.upper_weights) for af in anchor_airfoils])
    initial_lower = onp.stack([onp.asarray(af.lower_weights) for af in anchor_airfoils])
    weight_delta = regularization["kulfan_weight_delta_limit"]

    upper_controls = []
    lower_controls = []
    for weight_index in range(8):
        upper = opti.variable(init_guess=initial_upper[:, weight_index])
        lower = opti.variable(init_guess=initial_lower[:, weight_index])
        opti.subject_to(
            [
                upper >= initial_upper[:, weight_index] - weight_delta,
                upper <= initial_upper[:, weight_index] + weight_delta,
                lower >= initial_lower[:, weight_index] - weight_delta,
                lower <= initial_lower[:, weight_index] + weight_delta,
                upper >= lower + 0.03,
            ]
        )
        upper_controls.append(upper)
        lower_controls.append(lower)

    opti.subject_to(
        [
            upper_controls[0] >= 0.18,
            lower_controls[0] <= -0.16,
        ]
    )

    root_le_x = constraints["root_le_fixed_mm"] / 1000
    root_z = constraints["root_z_fixed_mm"] / 1000
    quarter_chord_x = root_le_x + 0.25 * constraints["root_chord_fixed_mm"] / 1000
    spar_centerline_x_m = [
        baseline["wing"]["structure"]["forward_spar_centerline_x_mm"] / 1000,
        baseline["wing"]["structure"]["rear_spar_centerline_x_mm"] / 1000,
    ]
    spar_centerline_z_offset_m = [
        (
            baseline["wing"]["structure"]["forward_spar_centerline_z_mm"]
            - constraints["root_z_fixed_mm"]
        )
        / 1000,
        (
            baseline["wing"]["structure"]["rear_spar_centerline_z_mm"]
            - constraints["root_z_fixed_mm"]
        )
        / 1000,
    ]
    te_physical_m = constraints["trailing_edge_physical_mm"] / 1000
    airfoils = []
    upper_at_stations = []
    lower_at_stations = []
    for index, eta_i in enumerate(eta):
        upper = [quadratic_bernstein(values, eta_i) for values in upper_controls]
        lower = [quadratic_bernstein(values, eta_i) for values in lower_controls]
        airfoil = asb.KulfanAirfoil(
            name=f"v201_phase4_{int(y_mm[index]):04d}",
            upper_weights=upper,
            lower_weights=lower,
            TE_thickness=te_physical_m / chords[index],
            leading_edge_weight=0.0,
        )
        airfoils.append(airfoil)
        upper_at_stations.append(upper)
        lower_at_stations.append(lower)

        thickness_30 = airfoil.local_thickness(0.30)
        opti.subject_to(
            [
                thickness_30 >= constraints["minimum_normalized_thickness"],
                thickness_30 <= constraints["maximum_normalized_thickness"],
                airfoil.local_camber(0.40) >= 0.005,
                airfoil.local_camber(0.40) <= 0.035,
            ]
        )
        for x_sample in [0.02, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 0.95]:
            opti.subject_to(airfoil.local_thickness(x_sample) >= 0.0025)

        if y_mm[index] <= constraints["spar_end_semispan_mm"]:
            station_le_x = quarter_chord_x - 0.25 * chords[index]
            required_center_to_skin_m = (
                baseline["wing"]["structure"]["carbon_member_outer_mm"] / 2000
                + 0.002
            )
            for spar_index, spar_x in enumerate(spar_centerline_x_m):
                spar_fraction = (spar_x - station_le_x) / chords[index]
                local_thickness = airfoil.local_thickness(spar_fraction)
                opti.subject_to(
                    local_thickness * chords[index]
                    >= constraints["spar_channel_depth_min_mm"] / 1000
                )
                local_camber = airfoil.local_camber(spar_fraction)
                vertical_midline = (
                    -(spar_fraction - 0.25)
                    * chords[index]
                    * np.sind(twists[index])
                    + local_camber * chords[index] * np.cosd(twists[index])
                )
                vertical_half_thickness = (
                    0.5
                    * local_thickness
                    * chords[index]
                    * np.cosd(twists[index])
                )
                opti.subject_to(
                    [
                        spar_centerline_z_offset_m[spar_index]
                        - (vertical_midline - vertical_half_thickness)
                        >= required_center_to_skin_m,
                        vertical_midline
                        + vertical_half_thickness
                        - spar_centerline_z_offset_m[spar_index]
                        >= required_center_to_skin_m,
                    ]
                )
            rear_hinge_x = station_le_x + baseline["wing"][
                "aileron_hinge_fraction"
            ] * chords[index]
            opti.subject_to(rear_hinge_x - spar_centerline_x_m[1] >= 0.008)

        if y_mm[index] == constraints["servo_station_mm"]:
            opti.subject_to(
                thickness_30 * chords[index]
                >= constraints["servo_depth_min_mm"] / 1000
            )

    area, mac = integrate_planform(y_m, chords)
    opti.subject_to(
        [
            area >= constraints["wing_area_min_m2"],
            area <= constraints["wing_area_max_m2"],
        ]
    )

    wing_xsecs = [
        asb.WingXSec(
            xyz_le=[
                quarter_chord_x - 0.25 * chords[index],
                y_m[index],
                root_z
                + max(
                    0.0,
                    y_m[index] - baseline["wing"]["dihedral_break_y_mm"] / 1000,
                )
                * np.tand(dihedral),
            ],
            chord=chords[index],
            twist=twists[index],
            airfoil=airfoils[index],
        )
        for index in range(len(y_m))
    ]
    wing = asb.Wing(name="V-201 Phase 4 Wing", symmetric=True, xsecs=wing_xsecs)
    airplane = asb.Airplane(
        name="V-201 Phase 4 Optimization Model",
        xyz_ref=[
            root_le_x + baseline["reference"]["cg_aft_of_root_le_mm"] / 1000,
            0.0,
            0.0,
        ],
        wings=[wing],
        s_ref=area,
        c_ref=mac,
        b_ref=span_m,
    )

    alphas = []
    point_expressions = []
    weighted_drag = 0
    for point in phase4["design_points"]:
        speed = point["speed_m_s"]
        q = 0.5 * rho * speed**2
        alpha = opti.variable(init_guess=point["alpha_initial_deg"])
        opti.subject_to([alpha >= -4.0, alpha <= 12.0])
        if point["name"] == "takeoff":
            opti.subject_to(alpha <= 10.0)
        alphas.append(alpha)

        vlm_results = asb.VortexLatticeMethod(
            airplane=airplane,
            op_point=asb.OperatingPoint(velocity=speed, alpha=alpha),
            spanwise_resolution=1,
            chordwise_resolution=5,
        ).run()
        opti.subject_to(vlm_results["L"] / weight_n == 1.0)

        target_cl = weight_n / (q * area)
        aspect_ratio = span_m**2 / area
        induced_angle_deg = np.degrees(target_cl / (np.pi * aspect_ratio * 0.85))
        cd_sections = []
        confidence_sections = []
        for index, airfoil in enumerate(airfoils):
            local_re = rho * speed * chords[index] / mu
            nf = airfoil.get_aero_from_neuralfoil(
                alpha=alpha + twists[index] - induced_angle_deg,
                Re=local_re,
                mach=speed / 343.0,
                n_crit=method["n_crit"],
                model_size=method["optimization_neuralfoil_model"],
                include_360_deg_effects=False,
            )
            cd_sections.append(nf["CD"])
            confidence_sections.append(nf["analysis_confidence"])
            opti.subject_to(
                [
                    nf["CD"] >= 0.004,
                    nf["analysis_confidence"]
                    >= constraints["minimum_neuralfoil_confidence"],
                ]
            )

        half_drag_area = 0
        for index in range(len(y_m) - 1):
            dy = y_m[index + 1] - y_m[index]
            half_drag_area += (
                0.5
                * (
                    cd_sections[index] * chords[index]
                    + cd_sections[index + 1] * chords[index + 1]
                )
                * dy
            )
        profile_drag = q * 2 * half_drag_area
        induced_drag = vlm_results["D"]
        ideal_induced_drag = weight_n**2 / (q * math.pi * span_m**2)
        opti.subject_to(
            induced_drag
            >= constraints["coarse_vlm_induced_drag_floor_factor"]
            * ideal_induced_drag
        )
        total_drag = profile_drag + induced_drag
        weighted_drag += point["objective_weight"] * total_drag
        point_expressions.append(
            {
                "name": point["name"],
                "speed_m_s": speed,
                "alpha": alpha,
                "lift": vlm_results["L"],
                "induced_drag": induced_drag,
                "profile_drag": profile_drag,
                "total_drag": total_drag,
                "minimum_confidence": np.min(np.stack(confidence_sections)),
            }
        )

    stall_re_speed = phase4["design_points"][0]["speed_m_s"]
    stall_slopes = []
    for index, airfoil in enumerate(airfoils):
        local_re = rho * stall_re_speed * chords[index] / mu
        nf_low = airfoil.get_aero_from_neuralfoil(
            alpha=14.0 + twists[index],
            Re=local_re,
            mach=0.0,
            n_crit=method["n_crit"],
            model_size=method["optimization_neuralfoil_model"],
            include_360_deg_effects=False,
        )
        nf_high = airfoil.get_aero_from_neuralfoil(
            alpha=15.0 + twists[index],
            Re=local_re,
            mach=0.0,
            n_crit=method["n_crit"],
            model_size=method["optimization_neuralfoil_model"],
            include_360_deg_effects=False,
        )
        stall_slopes.append(nf_high["CL"] - nf_low["CL"])
        opti.subject_to(
            [
                nf_low["analysis_confidence"]
                >= constraints["minimum_neuralfoil_confidence"],
                nf_high["analysis_confidence"]
                >= constraints["minimum_neuralfoil_confidence"],
            ]
        )
    opti.subject_to(stall_slopes[0] <= 0.01)
    for index, y in enumerate(y_mm):
        if y >= baseline["wing"]["aileron_start_mm"]:
            opti.subject_to(
                [
                    stall_slopes[index] >= 0.0,
                    stall_slopes[index] >= stall_slopes[0] + 0.003,
                ]
            )

    shape_regularization = 0
    for weight_index in range(8):
        shape_regularization += np.sum(
            ((upper_controls[weight_index] - initial_upper[:, weight_index]) / weight_delta)
            ** 2
        )
        shape_regularization += np.sum(
            ((lower_controls[weight_index] - initial_lower[:, weight_index]) / weight_delta)
            ** 2
        )
    shape_regularization /= 48
    planform_regularization = np.mean(((chords - initial_chords) / 0.010) ** 2)
    twist_smoothness = 0
    for index in range(1, len(y_m) - 1):
        slope_in = (twists[index] - twists[index - 1]) / (
            y_m[index] - y_m[index - 1]
        )
        slope_out = (twists[index + 1] - twists[index]) / (
            y_m[index + 1] - y_m[index]
        )
        twist_smoothness += ((slope_out - slope_in) / 10) ** 2
    dihedral_regularization = (
        (dihedral - baseline["wing"]["outer_panel_dihedral_deg"]) / 1.0
    ) ** 2

    objective = (
        weighted_drag
        + regularization["shape_weight"] * shape_regularization
        + regularization["planform_weight"] * planform_regularization
        + regularization["twist_smoothness_weight"] * twist_smoothness
        + regularization["dihedral_weight"] * dihedral_regularization
    )
    opti.minimize(objective)

    expressions = {
        "y_mm": y_mm,
        "chords": chords,
        "chord_controls": chord_controls,
        "twists": twists,
        "dihedral": dihedral,
        "airfoils": airfoils,
        "upper_at_stations": upper_at_stations,
        "lower_at_stations": lower_at_stations,
        "area": area,
        "mac": mac,
        "alphas": alphas,
        "points": point_expressions,
        "stall_slopes": stall_slopes,
        "objective": objective,
        "weighted_drag": weighted_drag,
        "initial_chords": initial_chords,
        "initial_twists": initial_twists,
    }
    return opti, expressions


def solve_optimization(
    baseline: dict[str, Any], phase4: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    if _HSL_DLL_HANDLE is None:
        raise RuntimeError(f"MA97 DLL directory was not found: {HSL_PATH}")
    opti, expressions = make_symbolic_problem(baseline, phase4)
    print("Solving V-201 Phase 4 with IPOPT / MA97...")
    solution = opti.solve(
        verbose=False,
        max_iter=1500,
        options={
            "ipopt.linear_solver": "ma97",
            "ipopt.tol": 2e-5,
            "ipopt.acceptable_tol": 2e-4,
            "ipopt.acceptable_iter": 8,
            "ipopt.hessian_approximation": "limited-memory",
            "ipopt.print_level": 0,
            "print_time": False,
        },
    )

    y_mm = expressions["y_mm"]
    chords_m = onp.asarray(solution(expressions["chords"]), dtype=float).reshape(-1)
    twists = onp.asarray(solution(expressions["twists"]), dtype=float).reshape(-1)
    outer_panel_dihedral = scalar(solution(expressions["dihedral"]))
    stations = []
    te_m = phase4["constraints"]["trailing_edge_physical_mm"] / 1000
    for index, y in enumerate(y_mm):
        upper = [
            scalar(solution(value)) for value in expressions["upper_at_stations"][index]
        ]
        lower = [
            scalar(solution(value)) for value in expressions["lower_at_stations"][index]
        ]
        stations.append(
            {
                "y_mm": float(y),
                "chord_mm": float(chords_m[index] * 1000),
                "twist_deg": float(twists[index]),
                "upper_weights": upper,
                "lower_weights": lower,
                "te_thickness_normalized": float(te_m / chords_m[index]),
                "te_thickness_mm": phase4["constraints"]["trailing_edge_physical_mm"],
            }
        )

    optimized_points = []
    for point in expressions["points"]:
        optimized_points.append(
            {
                "name": point["name"],
                "speed_m_s": point["speed_m_s"],
                "alpha_deg": scalar(solution(point["alpha"])),
                "lift_n": scalar(solution(point["lift"])),
                "induced_drag_n": scalar(solution(point["induced_drag"])),
                "profile_drag_n": scalar(solution(point["profile_drag"])),
                "total_drag_n": scalar(solution(point["total_drag"])),
                "lift_to_drag": scalar(solution(point["lift"] / point["total_drag"])),
                "minimum_neuralfoil_confidence": scalar(
                    solution(point["minimum_confidence"])
                ),
            }
        )

    design = {
        "project": baseline["project"],
        "phase": 4,
        "solver": "IPOPT with MA97",
        "root_geometry_locked": {
            "leading_edge_x_mm": baseline["reference"]["wing_root_le_x_mm"],
            "z_mm": baseline["reference"]["wing_root_z_mm"],
            "chord_mm": phase4["constraints"]["root_chord_fixed_mm"],
            "mount_interface_unchanged": True,
        },
        "span_mm": baseline["wing"]["span_mm"],
        "parallel_spar_centerlines_x_mm": [
            baseline["wing"]["structure"]["forward_spar_centerline_x_mm"],
            baseline["wing"]["structure"]["rear_spar_centerline_x_mm"],
        ],
        "parallel_spar_centerlines": [
            {
                "name": "forward",
                "x_mm": baseline["wing"]["structure"][
                    "forward_spar_centerline_x_mm"
                ],
                "z_mm": baseline["wing"]["structure"][
                    "forward_spar_centerline_z_mm"
                ],
                "y_start_mm": -baseline["wing"]["structure"][
                    "covered_semispan_mm"
                ],
                "y_end_mm": baseline["wing"]["structure"][
                    "covered_semispan_mm"
                ],
            },
            {
                "name": "rear",
                "x_mm": baseline["wing"]["structure"][
                    "rear_spar_centerline_x_mm"
                ],
                "z_mm": baseline["wing"]["structure"][
                    "rear_spar_centerline_z_mm"
                ],
                "y_start_mm": -baseline["wing"]["structure"][
                    "covered_semispan_mm"
                ],
                "y_end_mm": baseline["wing"]["structure"][
                    "covered_semispan_mm"
                ],
            },
        ],
        "center_panel_dihedral_deg": baseline["wing"]["center_panel_dihedral_deg"],
        "dihedral_break_y_mm": baseline["wing"]["dihedral_break_y_mm"],
        "outer_panel_dihedral_deg": outer_panel_dihedral,
        "wing_area_m2": scalar(solution(expressions["area"])),
        "mean_aerodynamic_chord_mm": scalar(solution(expressions["mac"])) * 1000,
        "stations": stations,
        "optimization_mesh_per_interval": {"spanwise": 1, "chordwise": 5},
        "optimization_points": optimized_points,
        "objective_value": scalar(solution(expressions["objective"])),
        "weighted_drag_component_n": scalar(solution(expressions["weighted_drag"])),
        "optimizer_stall_slope_proxy": [
            scalar(solution(value)) for value in expressions["stall_slopes"]
        ],
    }
    initial = {
        "chords_mm": (expressions["initial_chords"] * 1000).tolist(),
        "twists_deg": expressions["initial_twists"].tolist(),
    }
    return design, initial


def station_airfoil(station: dict[str, Any]) -> asb.KulfanAirfoil:
    return asb.KulfanAirfoil(
        name=f"v201_y{int(station['y_mm']):04d}",
        upper_weights=onp.asarray(station["upper_weights"]),
        lower_weights=onp.asarray(station["lower_weights"]),
        TE_thickness=station["te_thickness_normalized"],
        leading_edge_weight=0.0,
    )


def numeric_wing(design: dict[str, Any], baseline: dict[str, Any]) -> asb.Wing:
    root_le_x = baseline["reference"]["wing_root_le_x_mm"] / 1000
    root_z = baseline["reference"]["wing_root_z_mm"] / 1000
    root_chord = design["stations"][0]["chord_mm"] / 1000
    quarter_chord_x = root_le_x + 0.25 * root_chord
    return asb.Wing(
        name="V-201 Phase 4 Wing",
        symmetric=True,
        xsecs=[
            asb.WingXSec(
                xyz_le=[
                    quarter_chord_x - 0.25 * station["chord_mm"] / 1000,
                    station["y_mm"] / 1000,
                    root_z
                    + max(
                        0.0,
                        station["y_mm"] - design["dihedral_break_y_mm"],
                    )
                    / 1000
                    * math.tan(
                        math.radians(design["outer_panel_dihedral_deg"])
                    ),
                ],
                chord=station["chord_mm"] / 1000,
                twist=station["twist_deg"],
                airfoil=station_airfoil(station),
            )
            for station in design["stations"]
        ],
    )


def numeric_wing_airplane(
    design: dict[str, Any], baseline: dict[str, Any]
) -> asb.Airplane:
    return asb.Airplane(
        name="V-201 Phase 4 Wing Verification",
        xyz_ref=[
            (
                baseline["reference"]["wing_root_le_x_mm"]
                + baseline["reference"]["cg_aft_of_root_le_mm"]
            )
            / 1000,
            0.0,
            0.0,
        ],
        wings=[numeric_wing(design, baseline)],
        s_ref=design["wing_area_m2"],
        c_ref=design["mean_aerodynamic_chord_mm"] / 1000,
        b_ref=design["span_mm"] / 1000,
    )


def profile_drag_numeric(
    design: dict[str, Any],
    baseline: dict[str, Any],
    phase4: dict[str, Any],
    speed: float,
    alpha_deg: float,
) -> tuple[float, float]:
    rho = baseline["operating_conditions"]["air_density_kg_m3"]
    mu = 1.81e-5
    q = 0.5 * rho * speed**2
    weight_n = (
        baseline["mass_budget"]["design_auw_g"]
        / 1000
        * baseline["operating_conditions"]["gravity_m_s2"]
    )
    target_cl = weight_n / (q * design["wing_area_m2"])
    aspect_ratio = (design["span_mm"] / 1000) ** 2 / design["wing_area_m2"]
    induced_angle_deg = math.degrees(target_cl / (math.pi * aspect_ratio * 0.85))
    drag_integrands = []
    confidences = []
    for station in design["stations"]:
        chord_m = station["chord_mm"] / 1000
        nf = station_airfoil(station).get_aero_from_neuralfoil(
            alpha=alpha_deg + station["twist_deg"] - induced_angle_deg,
            Re=rho * speed * chord_m / mu,
            mach=speed / 343.0,
            n_crit=phase4["method"]["n_crit"],
            model_size=phase4["method"]["verification_neuralfoil_model"],
            include_360_deg_effects=False,
        )
        drag_integrands.append(scalar(nf["CD"]) * chord_m)
        confidences.append(scalar(nf["analysis_confidence"]))
    half_drag_area = onp.trapezoid(
        drag_integrands,
        [station["y_mm"] / 1000 for station in design["stations"]],
    )
    return q * 2 * half_drag_area, min(confidences)


def solve_vlm_point(
    design: dict[str, Any],
    baseline: dict[str, Any],
    phase4: dict[str, Any],
    speed: float,
    mesh: dict[str, Any],
) -> dict[str, float]:
    airplane = numeric_wing_airplane(design, baseline)
    weight_n = (
        baseline["mass_budget"]["design_auw_g"]
        / 1000
        * baseline["operating_conditions"]["gravity_m_s2"]
    )

    def run(alpha_deg: float) -> dict[str, Any]:
        return asb.VortexLatticeMethod(
            airplane=airplane,
            op_point=asb.OperatingPoint(velocity=speed, alpha=alpha_deg),
            spanwise_resolution=mesh["spanwise_resolution"],
            chordwise_resolution=mesh["chordwise_resolution"],
        ).run()

    alpha = brentq(lambda angle: scalar(run(angle)["L"]) - weight_n, -5.0, 15.0)
    result = run(alpha)
    profile_drag, confidence = profile_drag_numeric(
        design, baseline, phase4, speed, alpha
    )
    induced_drag = scalar(result["D"])
    total_drag = induced_drag + profile_drag
    return {
        "alpha_deg": alpha,
        "lift_n": scalar(result["L"]),
        "induced_drag_n": induced_drag,
        "profile_drag_n": profile_drag,
        "total_drag_n": total_drag,
        "lift_to_drag": weight_n / total_drag,
        "minimum_neuralfoil_confidence": confidence,
    }


def baseline_design(
    baseline: dict[str, Any], phase4: dict[str, Any]
) -> dict[str, Any]:
    stations = []
    y_values = phase4["optimization_stations_mm"]
    for y_mm in y_values:
        chord_mm = linear_interpolate_stations(baseline["wing"]["stations"], y_mm, "chord_mm")
        twist_deg = linear_interpolate_stations(baseline["wing"]["stations"], y_mm, "twist_deg")
        if y_mm <= 350:
            name = "naca2414"
        elif y_mm <= 525:
            name = "naca2413"
        else:
            name = "naca2412"
        airfoil = asb.Airfoil(name).to_kulfan_airfoil()
        stations.append(
            {
                "y_mm": y_mm,
                "chord_mm": chord_mm,
                "twist_deg": twist_deg,
                "upper_weights": onp.asarray(airfoil.upper_weights).tolist(),
                "lower_weights": onp.asarray(airfoil.lower_weights).tolist(),
                "te_thickness_normalized": scalar(airfoil.TE_thickness),
                "te_thickness_mm": scalar(airfoil.TE_thickness) * chord_mm,
            }
        )
    y_m = onp.asarray(y_values) / 1000
    chords_m = onp.asarray([station["chord_mm"] for station in stations]) / 1000
    area, mac = integrate_planform(y_m, chords_m)
    return {
        "project": baseline["project"],
        "phase": 3,
        "span_mm": baseline["wing"]["span_mm"],
        "center_panel_dihedral_deg": baseline["wing"]["center_panel_dihedral_deg"],
        "dihedral_break_y_mm": baseline["wing"]["dihedral_break_y_mm"],
        "outer_panel_dihedral_deg": baseline["wing"]["outer_panel_dihedral_deg"],
        "wing_area_m2": float(area),
        "mean_aerodynamic_chord_mm": float(mac * 1000),
        "stations": stations,
    }


def section_stall_verification(
    design: dict[str, Any], baseline: dict[str, Any], phase4: dict[str, Any]
) -> dict[str, Any]:
    verification = phase4["verification"]
    speed = phase4["design_points"][0]["speed_m_s"]
    rho = baseline["operating_conditions"]["air_density_kg_m3"]
    mu = 1.81e-5
    section_alphas = onp.arange(
        verification["section_stall_scan_min_deg"],
        verification["section_stall_scan_max_deg"]
        + 0.5 * verification["section_stall_scan_step_deg"],
        verification["section_stall_scan_step_deg"],
    )
    sections = []
    for station in design["stations"]:
        chord_m = station["chord_mm"] / 1000
        reynolds = rho * speed * chord_m / mu
        nf = station_airfoil(station).get_aero_from_neuralfoil(
            alpha=section_alphas,
            Re=reynolds,
            mach=speed / 343.0,
            n_crit=phase4["method"]["n_crit"],
            model_size=phase4["method"]["verification_neuralfoil_model"],
            include_360_deg_effects=False,
        )
        cl = onp.asarray(nf["CL"]).reshape(-1)
        confidence = onp.asarray(nf["analysis_confidence"]).reshape(-1)
        maximum_index = int(onp.argmax(cl))
        section_stall_alpha = float(section_alphas[maximum_index])
        sections.append(
            {
                "y_mm": station["y_mm"],
                "twist_deg": station["twist_deg"],
                "reynolds": reynolds,
                "section_stall_alpha_deg": section_stall_alpha,
                "aircraft_incidence_at_section_stall_deg": section_stall_alpha
                - station["twist_deg"],
                "section_cl_max": float(cl[maximum_index]),
                "minimum_scan_confidence": float(onp.min(confidence)),
            }
        )
    center_stall = min(
        section["aircraft_incidence_at_section_stall_deg"]
        for section in sections
        if section["y_mm"] <= 175
    )
    aileron_stall = min(
        section["aircraft_incidence_at_section_stall_deg"]
        for section in sections
        if section["y_mm"] >= baseline["wing"]["aileron_start_mm"]
    )
    first = min(sections, key=lambda section: section["aircraft_incidence_at_section_stall_deg"])
    margin = aileron_stall - center_stall
    return {
        "sections": sections,
        "first_stall_station_y_mm": first["y_mm"],
        "center_to_aileron_stall_margin_deg": margin,
        "required_margin_deg": phase4["constraints"]["aileron_stall_margin_min_deg"],
        "root_first_stall_pass": first["y_mm"] <= 175
        and margin >= phase4["constraints"]["aileron_stall_margin_min_deg"],
    }


def build_full_airplane(
    design: dict[str, Any],
    baseline: dict[str, Any],
    elevator_deg: Any = 0.0,
) -> asb.Airplane:
    main_wing = numeric_wing(design, baseline)
    horizontal = baseline["horizontal_tail"]
    elevator = [
        asb.ControlSurface(
            name="Elevator",
            symmetric=True,
            deflection=elevator_deg,
            hinge_point=horizontal["elevator_hinge_fraction"],
        )
    ]
    horizontal_tail = asb.Wing(
        name="Horizontal Tail",
        symmetric=True,
        xsecs=[
            asb.WingXSec(
                xyz_le=[horizontal["root_le_x_mm"] / 1000, 0.0, horizontal["z_mm"] / 1000],
                chord=horizontal["root_chord_mm"] / 1000,
                airfoil=asb.Airfoil(horizontal["airfoil"]),
                control_surfaces=elevator,
            ),
            asb.WingXSec(
                xyz_le=[
                    horizontal["tip_le_x_mm"] / 1000,
                    horizontal["span_mm"] / 2000,
                    horizontal["z_mm"] / 1000,
                ],
                chord=horizontal["tip_chord_mm"] / 1000,
                airfoil=asb.Airfoil(horizontal["airfoil"]),
                control_surfaces=elevator,
            ),
        ],
    )
    vertical = baseline["vertical_tail"]
    rudder = [
        asb.ControlSurface(
            name="Rudder",
            symmetric=True,
            deflection=0.0,
            hinge_point=vertical["rudder_hinge_fraction"],
        )
    ]
    vertical_tail = asb.Wing(
        name="Vertical Tail",
        symmetric=False,
        xsecs=[
            asb.WingXSec(
                xyz_le=[vertical["root_le_x_mm"] / 1000, 0.0, 0.0],
                chord=vertical["root_chord_mm"] / 1000,
                airfoil=asb.Airfoil(vertical["airfoil"]),
                control_surfaces=rudder,
            ),
            asb.WingXSec(
                xyz_le=[
                    vertical["tip_le_x_mm"] / 1000,
                    0.0,
                    vertical["height_mm"] / 1000,
                ],
                chord=vertical["tip_chord_mm"] / 1000,
                airfoil=asb.Airfoil(vertical["airfoil"]),
                control_surfaces=rudder,
            ),
        ],
    )
    fuselage = asb.Fuselage(
        name="Fuselage",
        xsecs=[
            asb.FuselageXSec(
                xyz_c=[section["x_mm"] / 1000, 0.0, section["z_mm"] / 1000],
                width=section["width_mm"] / 1000,
                height=section["height_mm"] / 1000,
            )
            for section in baseline["fuselage"]["sections"]
        ],
    )
    return asb.Airplane(
        name="V-201 Phase 4 Full Aircraft",
        xyz_ref=[
            (
                baseline["reference"]["wing_root_le_x_mm"]
                + baseline["reference"]["cg_aft_of_root_le_mm"]
            )
            / 1000,
            0.0,
            0.0,
        ],
        wings=[main_wing, horizontal_tail, vertical_tail],
        fuselages=[fuselage],
        s_ref=design["wing_area_m2"],
        c_ref=design["mean_aerodynamic_chord_mm"] / 1000,
        b_ref=design["span_mm"] / 1000,
    )


def full_aircraft_aero(
    design: dict[str, Any],
    baseline: dict[str, Any],
    speed: float,
    alpha_deg: float,
    elevator_deg: float,
    beta_deg: float = 0.0,
) -> dict[str, Any]:
    return asb.AeroBuildup(
        airplane=build_full_airplane(design, baseline, elevator_deg),
        op_point=asb.OperatingPoint(
            velocity=speed, alpha=alpha_deg, beta=beta_deg
        ),
    ).run()


def full_aircraft_trim_and_stability(
    design: dict[str, Any],
    baseline: dict[str, Any],
    speed: float,
    include_stability: bool = False,
) -> dict[str, Any]:
    weight_n = (
        baseline["mass_budget"]["design_auw_g"]
        / 1000
        * baseline["operating_conditions"]["gravity_m_s2"]
    )
    opti = asb.Opti()
    alpha = opti.variable(init_guess=1.0, lower_bound=-4.0, upper_bound=12.0)
    elevator = opti.variable(init_guess=-2.0, lower_bound=-15.0, upper_bound=15.0)
    result = asb.AeroBuildup(
        airplane=build_full_airplane(design, baseline, elevator),
        op_point=asb.OperatingPoint(velocity=speed, alpha=alpha),
    ).run()
    opti.subject_to([result["L"] / weight_n == 1.0, result["Cm"] == 0.0])
    opti.minimize(1e-3 * elevator**2 + 1e-5 * alpha**2)
    solution = opti.solve(verbose=False, max_iter=400)
    trim_alpha = scalar(solution(alpha))
    trim_elevator = scalar(solution(elevator))
    trim = {
        key: scalar(solution(result[key]))
        for key in ["L", "D", "CL", "CD", "Cm", "D_induced", "D_profile"]
    }
    trim["L_D"] = trim["L"] / trim["D"]

    output = {
        "speed_m_s": speed,
        "alpha_deg": trim_alpha,
        "elevator_deg": trim_elevator,
        **trim,
    }
    if include_stability:
        step = 0.5
        total_step_rad = math.radians(2 * step)
        alpha_plus = full_aircraft_aero(
            design, baseline, speed, trim_alpha + step, trim_elevator
        )
        alpha_minus = full_aircraft_aero(
            design, baseline, speed, trim_alpha - step, trim_elevator
        )
        beta_plus = full_aircraft_aero(
            design, baseline, speed, trim_alpha, trim_elevator, step
        )
        beta_minus = full_aircraft_aero(
            design, baseline, speed, trim_alpha, trim_elevator, -step
        )
        output["static_stability"] = {
            "Cm_alpha_per_rad": (
                scalar(alpha_plus["Cm"]) - scalar(alpha_minus["Cm"])
            )
            / total_step_rad,
            "Cl_beta_per_rad": (
                scalar(beta_plus["Cl"]) - scalar(beta_minus["Cl"])
            )
            / total_step_rad,
            "Cn_beta_per_rad": (
                scalar(beta_plus["Cn"]) - scalar(beta_minus["Cn"])
            )
            / total_step_rad,
        }
    return output


def structural_verification(
    design: dict[str, Any], baseline: dict[str, Any], phase4: dict[str, Any]
) -> dict[str, Any]:
    constraints = phase4["constraints"]
    structure = baseline["wing"]["structure"]
    station_checks = []
    root_quarter_chord_x_mm = (
        baseline["reference"]["wing_root_le_x_mm"]
        + 0.25 * design["stations"][0]["chord_mm"]
    )
    for station in design["stations"]:
        airfoil = station_airfoil(station)
        chord_mm = station["chord_mm"]
        leading_edge_x_mm = root_quarter_chord_x_mm - 0.25 * chord_mm
        forward_fraction = (
            structure["forward_spar_centerline_x_mm"] - leading_edge_x_mm
        ) / chord_mm
        rear_fraction = (
            structure["rear_spar_centerline_x_mm"] - leading_edge_x_mm
        ) / chord_mm
        hinge_x_mm = (
            leading_edge_x_mm
            + baseline["wing"]["aileron_hinge_fraction"] * chord_mm
        )
        twist_rad = math.radians(station["twist_deg"])

        def vertical_tube_clearance(
            spar_fraction: float, spar_centerline_z_mm: float
        ) -> float:
            thickness = scalar(airfoil.local_thickness(spar_fraction))
            camber = scalar(airfoil.local_camber(spar_fraction))
            vertical_midline_mm = (
                -(spar_fraction - 0.25) * chord_mm * math.sin(twist_rad)
                + camber * chord_mm * math.cos(twist_rad)
            )
            vertical_half_thickness_mm = (
                0.5 * thickness * chord_mm * math.cos(twist_rad)
            )
            tube_z_offset_mm = (
                spar_centerline_z_mm - baseline["reference"]["wing_root_z_mm"]
            )
            lower_distance = tube_z_offset_mm - (
                vertical_midline_mm - vertical_half_thickness_mm
            )
            upper_distance = (
                vertical_midline_mm + vertical_half_thickness_mm
            ) - tube_z_offset_mm
            return (
                min(lower_distance, upper_distance)
                - structure["carbon_member_outer_mm"] / 2
            )

        check = {
            "y_mm": station["y_mm"],
            "thickness_30_mm": scalar(airfoil.local_thickness(0.30)) * chord_mm,
            "max_thickness_ratio": scalar(airfoil.max_thickness()),
            "forward_spar_local_chord_fraction": forward_fraction,
            "rear_spar_local_chord_fraction": rear_fraction,
            "forward_spar_depth_mm": scalar(
                airfoil.local_thickness(forward_fraction)
            )
            * chord_mm,
            "rear_spar_depth_mm": scalar(
                airfoil.local_thickness(rear_fraction)
            )
            * chord_mm,
            "rear_spar_to_aileron_hinge_mm": hinge_x_mm
            - structure["rear_spar_centerline_x_mm"],
            "forward_tube_to_nearest_skin_clearance_mm": vertical_tube_clearance(
                forward_fraction, structure["forward_spar_centerline_z_mm"]
            ),
            "rear_tube_to_nearest_skin_clearance_mm": vertical_tube_clearance(
                rear_fraction, structure["rear_spar_centerline_z_mm"]
            ),
        }
        station_checks.append(check)

    tube_outer_m = structure["carbon_member_outer_mm"] / 1000
    tube_inner_m = structure["carbon_member_inner_mm"] / 1000
    second_moment_single = math.pi / 64 * (tube_outer_m**4 - tube_inner_m**4)
    span_m = design["span_mm"] / 1000
    weight_n = (
        baseline["mass_budget"]["design_auw_g"]
        / 1000
        * baseline["operating_conditions"]["gravity_m_s2"]
    )
    root_moment = (
        constraints["design_load_factor_g"] * weight_n * span_m / (3 * math.pi)
    )
    forward_stress_mpa = (
        root_moment
        * constraints["forward_spar_load_share"]
        * (tube_outer_m / 2)
        / second_moment_single
        / 1e6
    )
    rear_stress_mpa = (
        root_moment
        * constraints["rear_spar_load_share"]
        * (tube_outer_m / 2)
        / second_moment_single
        / 1e6
    )
    required_strength_mpa = (
        max(forward_stress_mpa, rear_stress_mpa)
        * constraints["structural_safety_factor"]
    )
    spar_region = [
        check
        for check in station_checks
        if check["y_mm"] <= constraints["spar_end_semispan_mm"]
    ]
    servo = next(
        check
        for check in station_checks
        if check["y_mm"] == constraints["servo_station_mm"]
    )
    return {
        "station_geometry": station_checks,
        "tube_second_moment_each_m4": second_moment_single,
        "design_load_factor_g": constraints["design_load_factor_g"],
        "elliptical_lift_root_bending_moment_nm": root_moment,
        "assumed_forward_rear_load_share": [
            constraints["forward_spar_load_share"],
            constraints["rear_spar_load_share"],
        ],
        "forward_spar_bending_stress_mpa": forward_stress_mpa,
        "rear_spar_bending_stress_mpa": rear_stress_mpa,
        "required_carbon_bending_strength_with_safety_factor_mpa": required_strength_mpa,
        "provisional_allowable_mpa": constraints[
            "provisional_carbon_bending_allowable_mpa"
        ],
        "spar_strength_provisional_pass": required_strength_mpa
        <= constraints["provisional_carbon_bending_allowable_mpa"],
        "spar_channel_geometry_pass": min(
            min(check["forward_spar_depth_mm"], check["rear_spar_depth_mm"])
            for check in spar_region
        )
        >= constraints["spar_channel_depth_min_mm"],
        "parallel_spar_geometry_pass": structure["constant_global_x"]
        and structure["arrangement"] == "parallel spanwise, centered on aircraft",
        "rear_spar_hinge_clearance_pass": min(
            check["rear_spar_to_aileron_hinge_mm"] for check in spar_region
        )
        >= 8.0,
        "straight_tube_vertical_envelope_pass": min(
            min(
                check["forward_tube_to_nearest_skin_clearance_mm"],
                check["rear_tube_to_nearest_skin_clearance_mm"],
            )
            for check in spar_region
        )
        >= 2.0,
        "required_tube_to_skin_clearance_mm": 2.0,
        "servo_depth_pass": servo["thickness_30_mm"]
        >= constraints["servo_depth_min_mm"] - 0.01,
        "warning": "The carbon allowable is provisional; verify the purchased tube specification and complete the printed-wing proof test.",
    }


def takeoff_landing_estimate(
    design: dict[str, Any],
    baseline: dict[str, Any],
    phase4: dict[str, Any],
    stall: dict[str, Any],
    full_aircraft_takeoff: dict[str, Any],
) -> dict[str, Any]:
    """Return a conservative, conditional ground-roll and approach estimate."""
    assumptions = phase4["verification"]["takeoff_ground_roll"]
    mass_kg = baseline["mass_budget"]["design_auw_g"] / 1000
    gravity = baseline["operating_conditions"]["gravity_m_s2"]
    weight_n = mass_kg * gravity
    rho = baseline["operating_conditions"]["air_density_kg_m3"]
    rotation_speed = next(
        point["speed_m_s"]
        for point in phase4["design_points"]
        if point["name"] == "takeoff"
    )
    minimum_section_clmax = min(
        section["section_cl_max"] for section in stall["sections"]
    )
    conservative_3d_clmax = (
        assumptions["conservative_3d_clmax_fraction_of_min_section_clmax"]
        * minimum_section_clmax
    )
    stall_speed = math.sqrt(
        2 * weight_n / (rho * design["wing_area_m2"] * conservative_3d_clmax)
    )
    rolling_force = assumptions["asphalt_rolling_resistance_coefficient"] * weight_n
    average_drag = (
        assumptions["average_drag_fraction_of_rotation_drag"]
        * full_aircraft_takeoff["D"]
    )
    thrust_cases = []
    for label, thrust_g in [
        ("minimum_required", baseline["propulsion"]["minimum_static_thrust_g"]),
        ("target", baseline["propulsion"]["target_static_thrust_g"]),
    ]:
        average_thrust = (
            thrust_g
            / 1000
            * gravity
            * assumptions["average_thrust_fraction_of_static"]
        )
        net_force = average_thrust - rolling_force - average_drag
        acceleration = net_force / mass_kg
        ground_roll = (
            rotation_speed**2 / (2 * acceleration) if acceleration > 0 else math.inf
        )
        thrust_cases.append(
            {
                "name": label,
                "static_thrust_g": thrust_g,
                "assumed_average_thrust_n": average_thrust,
                "net_average_acceleration_m_s2": acceleration,
                "estimated_ground_roll_m": ground_roll,
                "within_5_m_runway": ground_roll <= assumptions["maximum_runway_m"],
            }
        )
    return {
        "assumptions": assumptions,
        "conservative_3d_clmax": conservative_3d_clmax,
        "estimated_stall_speed_m_s": stall_speed,
        "target_rotation_speed_m_s": rotation_speed,
        "rotation_to_stall_speed_ratio": rotation_speed / stall_speed,
        "recommended_approach_speed_m_s": assumptions[
            "approach_to_stall_speed_ratio"
        ]
        * stall_speed,
        "thrust_cases": thrust_cases,
        "rotation_margin_pass": rotation_speed / stall_speed
        >= assumptions["minimum_rotation_to_stall_speed_ratio"],
        "minimum_thrust_case_runway_pass": thrust_cases[0]["within_5_m_runway"],
        "condition": "Ground-roll result is valid only if a thrust-stand test confirms at least the stated static thrust with the installed battery and propeller.",
    }


def export_outputs(
    design: dict[str, Any],
    initial: dict[str, Any],
    baseline: dict[str, Any],
    phase4: dict[str, Any],
    summary: dict[str, Any],
) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    OPTIMIZATION_DIR.mkdir(parents=True, exist_ok=True)
    AIRFOIL_DIR.mkdir(parents=True, exist_ok=True)
    CAD_REFERENCE_DIR.mkdir(parents=True, exist_ok=True)

    with (RESULTS_DIR / "v201_phase4_optimized_design.json").open(
        "w", encoding="utf-8"
    ) as stream:
        json.dump(design, stream, indent=2, default=json_compatible)
    with (RESULTS_DIR / "v201_phase4_summary.json").open(
        "w", encoding="utf-8"
    ) as stream:
        json.dump(summary, stream, indent=2, default=json_compatible)

    with (OPTIMIZATION_DIR / "v201_phase4_wing_stations.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "y_mm",
                "chord_mm",
                "twist_deg",
                "panel_dihedral_deg",
                "leading_edge_x_mm",
                "leading_edge_z_mm",
            ]
        )
        root_le = baseline["reference"]["wing_root_le_x_mm"]
        quarter_chord = root_le + 0.25 * design["stations"][0]["chord_mm"]
        for station in design["stations"]:
            writer.writerow(
                [
                    station["y_mm"],
                    station["chord_mm"],
                    station["twist_deg"],
                    (
                        design["center_panel_dihedral_deg"]
                        if station["y_mm"] <= design["dihedral_break_y_mm"]
                        else design["outer_panel_dihedral_deg"]
                    ),
                    quarter_chord - 0.25 * station["chord_mm"],
                    baseline["reference"]["wing_root_z_mm"]
                    + max(
                        0.0,
                        station["y_mm"] - design["dihedral_break_y_mm"],
                    )
                    * math.tan(
                        math.radians(design["outer_panel_dihedral_deg"])
                    ),
                ]
            )

    with (OPTIMIZATION_DIR / "v201_phase4_spar_rails.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "name",
                "x_mm",
                "z_mm",
                "y_start_mm",
                "y_end_mm",
                "length_mm",
                "outer_diameter_mm",
                "inner_diameter_mm",
            ],
        )
        writer.writeheader()
        for rail in design["parallel_spar_centerlines"]:
            writer.writerow(
                {
                    **rail,
                    "length_mm": rail["y_end_mm"] - rail["y_start_mm"],
                    "outer_diameter_mm": baseline["wing"]["structure"][
                        "carbon_member_outer_mm"
                    ],
                    "inner_diameter_mm": baseline["wing"]["structure"][
                        "carbon_member_inner_mm"
                    ],
                }
            )

    with (OPTIMIZATION_DIR / "v201_phase4_kulfan_coefficients.json").open(
        "w", encoding="utf-8"
    ) as stream:
        json.dump(
            {
                "parameterization": phase4["method"]["airfoil_parameterization"],
                "stations": [
                    {
                        key: station[key]
                        for key in [
                            "y_mm",
                            "upper_weights",
                            "lower_weights",
                            "te_thickness_normalized",
                            "te_thickness_mm",
                        ]
                    }
                    for station in design["stations"]
                ],
            },
            stream,
            indent=2,
            default=json_compatible,
        )

    for index, station in enumerate(design["stations"]):
        coordinates = onp.asarray(station_airfoil(station).coordinates)
        path = AIRFOIL_DIR / f"v201_phase4_station_{index:02d}_y{int(station['y_mm']):04d}.dat"
        with path.open("w", encoding="utf-8") as stream:
            stream.write(f"V-201 Phase 4 y={station['y_mm']:.1f} mm\n")
            for x_value, z_value in coordinates:
                stream.write(f"{x_value:.8f} {z_value:.8f}\n")

    figure, axes = plt.subplots(2, 1, figsize=(9, 8), constrained_layout=True)
    for station in design["stations"]:
        coordinates = onp.asarray(station_airfoil(station).coordinates)
        axes[0].plot(
            coordinates[:, 0], coordinates[:, 1], label=f"y={station['y_mm']:.0f} mm"
        )
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_title("Optimized Kulfan sections (normalized and collapsed)")
    axes[0].set_xlabel("x/c")
    axes[0].set_ylabel("z/c")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(ncol=2, fontsize=8)

    y_values = [station["y_mm"] for station in design["stations"]]
    axes[1].plot(y_values, initial["chords_mm"], "--", label="Phase 3 chord")
    axes[1].plot(
        y_values,
        [station["chord_mm"] for station in design["stations"]],
        "o-",
        label="Phase 4 chord",
    )
    axes[1].set_xlabel("Semispan y (mm)")
    axes[1].set_ylabel("Chord (mm)")
    axes[1].grid(True, alpha=0.3)
    twist_axis = axes[1].twinx()
    twist_axis.plot(
        y_values,
        [station["twist_deg"] for station in design["stations"]],
        "s-",
        color="tab:red",
        label="Phase 4 twist",
    )
    twist_axis.set_ylabel("Twist (deg)", color="tab:red")
    axes[1].legend(loc="upper left")
    figure.savefig(OPTIMIZATION_DIR / "v201_phase4_geometry.png", dpi=180)
    plt.close(figure)

    stall_sections = summary["section_stall_verification"]["sections"]
    figure, axis = plt.subplots(figsize=(8, 4.5), constrained_layout=True)
    axis.plot(
        [section["y_mm"] for section in stall_sections],
        [section["aircraft_incidence_at_section_stall_deg"] for section in stall_sections],
        "o-",
    )
    axis.axvspan(
        baseline["wing"]["aileron_start_mm"],
        baseline["wing"]["aileron_end_mm"],
        alpha=0.15,
        color="tab:orange",
        label="Aileron span",
    )
    axis.set_xlabel("Semispan y (mm)")
    axis.set_ylabel("Aircraft incidence at local section stall (deg)")
    axis.set_title("Root-first stall verification")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.savefig(OPTIMIZATION_DIR / "v201_phase4_stall_order.png", dpi=180)
    plt.close(figure)

    airplane = numeric_wing_airplane(design, baseline)
    airplane.draw(backend="plotly", show=False).write_html(
        str(CAD_REFERENCE_DIR / "v201_phase4_optimized_wing.html")
    )
    three_view_axes = airplane.draw_three_view(show=False)
    three_view_axes.reshape(-1)[0].figure.savefig(
        CAD_REFERENCE_DIR / "v201_phase4_optimized_wing_three_view.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(three_view_axes.reshape(-1)[0].figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="Reuse the saved optimized-design checkpoint and rerun verification/exports.",
    )
    arguments = parser.parse_args()
    baseline = load_json(BASELINE_CONFIG_PATH)
    phase4 = load_json(PHASE4_CONFIG_PATH)
    if arguments.verify_only:
        checkpoint = RESULTS_DIR / "v201_phase4_optimized_design.json"
        if not checkpoint.exists():
            raise FileNotFoundError(f"No Phase 4 checkpoint exists: {checkpoint}")
        print(f"Reusing optimized-design checkpoint: {checkpoint}")
        design = load_json(checkpoint)
        baseline_geometry_for_initial = baseline_design(baseline, phase4)
        initial = {
            "chords_mm": [
                station["chord_mm"]
                for station in baseline_geometry_for_initial["stations"]
            ],
            "twists_deg": [
                station["twist_deg"]
                for station in baseline_geometry_for_initial["stations"]
            ],
        }
    else:
        design, initial = solve_optimization(baseline, phase4)

    high_mesh = phase4["verification"]["vlm_meshes"][-1]
    optimized_performance = {}
    for point in phase4["design_points"]:
        print(f"High-resolution verification: {point['name']}...")
        optimized_performance[point["name"]] = solve_vlm_point(
            design, baseline, phase4, point["speed_m_s"], high_mesh
        )

    baseline_geometry = baseline_design(baseline, phase4)
    baseline_performance = {}
    for point in phase4["design_points"]:
        baseline_performance[point["name"]] = solve_vlm_point(
            baseline_geometry, baseline, phase4, point["speed_m_s"], high_mesh
        )

    convergence = {}
    cruise_speed = next(
        point["speed_m_s"]
        for point in phase4["design_points"]
        if point["name"] == "cruise"
    )
    for mesh in phase4["verification"]["vlm_meshes"]:
        print(f"Cruise convergence mesh: {mesh['name']}...")
        convergence[mesh["name"]] = solve_vlm_point(
            design, baseline, phase4, cruise_speed, mesh
        )
    medium, high = convergence["medium"], convergence["high"]
    convergence["medium_to_high_difference_percent"] = {
        "induced_drag": 100
        * abs(medium["induced_drag_n"] - high["induced_drag_n"])
        / high["induced_drag_n"],
        "total_drag": 100
        * abs(medium["total_drag_n"] - high["total_drag_n"])
        / high["total_drag_n"],
    }

    stall = section_stall_verification(design, baseline, phase4)
    full_aircraft = {}
    for point in phase4["design_points"]:
        full_aircraft[point["name"]] = full_aircraft_trim_and_stability(
            design,
            baseline,
            point["speed_m_s"],
            include_stability=point["name"] == "cruise",
        )
    cruise_aircraft = full_aircraft["cruise"]
    structure = structural_verification(design, baseline, phase4)
    field_performance = takeoff_landing_estimate(
        design,
        baseline,
        phase4,
        stall,
        full_aircraft["takeoff"],
    )
    differences = convergence["medium_to_high_difference_percent"]
    checks = {
        "optimizer_used_ma97": design["solver"] == "IPOPT with MA97",
        "root_mount_geometry_unchanged": design["root_geometry_locked"][
            "mount_interface_unchanged"
        ],
        "wing_area_within_bounds": phase4["constraints"]["wing_area_min_m2"]
        <= design["wing_area_m2"]
        <= phase4["constraints"]["wing_area_max_m2"],
        "root_first_stall": stall["root_first_stall_pass"],
        "high_resolution_confidence": min(
            point["minimum_neuralfoil_confidence"]
            for point in optimized_performance.values()
        )
        >= phase4["constraints"]["minimum_neuralfoil_confidence"],
        "stall_scan_confidence": min(
            section["minimum_scan_confidence"] for section in stall["sections"]
        )
        >= phase4["constraints"]["minimum_neuralfoil_confidence"],
        "vlm_total_drag_converged": differences["total_drag"]
        <= phase4["verification"]["medium_high_total_drag_difference_max_percent"],
        "vlm_induced_drag_converged": differences["induced_drag"]
        <= phase4["verification"]["medium_high_induced_drag_difference_max_percent"],
        "servo_depth_preserved": structure["servo_depth_pass"],
        "both_spar_channels_fit": structure["spar_channel_geometry_pass"],
        "parallel_spar_centerlines": structure["parallel_spar_geometry_pass"],
        "rear_spar_clears_aileron_hinge": structure[
            "rear_spar_hinge_clearance_pass"
        ],
        "straight_spars_clear_wing_skin": structure[
            "straight_tube_vertical_envelope_pass"
        ],
        "carbon_strength_provisional": structure["spar_strength_provisional_pass"],
        "full_aircraft_pitch_stable": cruise_aircraft["static_stability"][
            "Cm_alpha_per_rad"
        ]
        < 0,
        "full_aircraft_roll_stable": cruise_aircraft["static_stability"][
            "Cl_beta_per_rad"
        ]
        < 0,
        "full_aircraft_yaw_stable": cruise_aircraft["static_stability"][
            "Cn_beta_per_rad"
        ]
        > 0,
        "all_design_points_trimmed": all(
            abs(full_aircraft[point["name"]]["elevator_deg"])
            <= point["elevator_limit_deg"]
            and -4.0 < full_aircraft[point["name"]]["alpha_deg"] < 12.0
            for point in phase4["design_points"]
        ),
        "rotation_speed_has_stall_margin": field_performance["rotation_margin_pass"],
        "five_metre_runway_at_minimum_thrust": field_performance[
            "minimum_thrust_case_runway_pass"
        ],
    }
    summary = {
        "project": baseline["project"],
        "phase": 4,
        "status": "pass" if all(checks.values()) else "review_required",
        "method": phase4["method"],
        "optimized_geometry": {
            "span_mm": design["span_mm"],
            "wing_area_m2": design["wing_area_m2"],
            "mean_aerodynamic_chord_mm": design["mean_aerodynamic_chord_mm"],
            "center_panel_dihedral_deg": design["center_panel_dihedral_deg"],
            "dihedral_break_y_mm": design["dihedral_break_y_mm"],
            "outer_panel_dihedral_deg": design["outer_panel_dihedral_deg"],
        },
        "optimized_high_resolution_performance": optimized_performance,
        "phase3_high_resolution_performance": baseline_performance,
        "vlm_convergence": convergence,
        "section_stall_verification": stall,
        "full_aircraft_trim": full_aircraft,
        "takeoff_and_landing_estimate": field_performance,
        "structural_verification": structure,
        "acceptance_checks": checks,
        "limitations": [
            "NeuralFoil and VLM are engineering models, not flight certification.",
            "Printed-surface roughness, propeller slipstream, and landing-gear drag are not resolved.",
            "The carbon strength check uses a provisional allowable and must be replaced by the purchased tube data.",
            "The printed wing, saddle, adhesive joints, and inserts still require the documented 3 g and 4 g proof tests.",
        ],
    }
    export_outputs(design, initial, baseline, phase4, summary)

    print(f"V-201 Phase 4 status: {summary['status']}")
    print(
        f"Geometry: area={design['wing_area_m2']:.4f} m^2, "
        f"MAC={design['mean_aerodynamic_chord_mm']:.1f} mm, "
        f"outer dihedral={design['outer_panel_dihedral_deg']:.2f} deg"
    )
    print(
        "Root-first stall margin: "
        f"{stall['center_to_aileron_stall_margin_deg']:.2f} deg"
    )
    print(
        "Cruise high-resolution L/D: "
        f"{optimized_performance['cruise']['lift_to_drag']:.2f}"
    )
    print(f"Summary: {RESULTS_DIR / 'v201_phase4_summary.json'}")


if __name__ == "__main__":
    main()
