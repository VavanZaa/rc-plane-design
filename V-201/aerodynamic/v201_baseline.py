"""V-201 Phase 3: full-aircraft aerodynamic baseline.

This script deliberately keeps optimization out of Phase 3. It creates the
baseline geometry, trims the complete aircraft at cruise, evaluates basic
static-stability derivatives, and writes reproducible reference outputs.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
CONFIG_PATH = SCRIPT_DIR / "v201_baseline_config.json"
RESULTS_DIR = SCRIPT_DIR / "results"
CAD_REFERENCE_DIR = PROJECT_DIR / "cad_reference"


def mm(value: float) -> float:
    """Convert millimetres to metres for AeroSandbox geometry."""
    return value / 1000.0


def scalar(value: Any) -> float:
    """Convert an AeroSandbox scalar or one-element array to a Python float."""
    array = np.asarray(value).reshape(-1)
    return float(array[0])


def load_config(path: Path = CONFIG_PATH) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def integrate_planform(stations: list[dict[str, Any]]) -> tuple[float, float]:
    """Return full-wing area and mean aerodynamic chord in SI units."""
    half_area = 0.0
    half_chord_squared_integral = 0.0

    for inner, outer in zip(stations[:-1], stations[1:]):
        y0, y1 = mm(inner["y_mm"]), mm(outer["y_mm"])
        c0, c1 = mm(inner["chord_mm"]), mm(outer["chord_mm"])
        dy = y1 - y0
        half_area += 0.5 * (c0 + c1) * dy
        half_chord_squared_integral += dy * (c0**2 + c0 * c1 + c1**2) / 3

    area = 2 * half_area
    mean_aerodynamic_chord = 2 * half_chord_squared_integral / area
    return area, mean_aerodynamic_chord


def interpolate_fuselage_width(config: dict[str, Any], x_mm: float) -> float:
    """Linearly interpolate fuselage width at a longitudinal station."""
    sections = config["fuselage"]["sections"]
    for forward, aft in zip(sections[:-1], sections[1:]):
        if forward["x_mm"] <= x_mm <= aft["x_mm"]:
            fraction = (x_mm - forward["x_mm"]) / (
                aft["x_mm"] - forward["x_mm"]
            )
            return forward["width_mm"] + fraction * (
                aft["width_mm"] - forward["width_mm"]
            )
    raise ValueError(f"Mount point X={x_mm} mm lies outside the fuselage definition.")


def mounting_interface_summary(config: dict[str, Any]) -> dict[str, Any]:
    """Evaluate the removable wing saddle and fastener packaging."""
    reference = config["reference"]
    wing = config["wing"]
    structure = wing["structure"]
    interface = config["wing_fuselage_interface"]
    fasteners = interface["fasteners"]
    root_le_x = reference["wing_root_le_x_mm"]
    root_chord = wing["stations"][0]["chord_mm"]
    root_te_x = root_le_x + root_chord
    spar_x_positions = [
        structure["forward_spar_centerline_x_mm"],
        structure["rear_spar_centerline_x_mm"],
    ]

    boss_radius = fasteners["hardpoint_boss_outer_mm"] / 2
    bolt_positions = fasteners["positions_mm"]
    side_margins = [
        interpolate_fuselage_width(config, bolt["x_mm"]) / 2
        - abs(bolt["y_mm"])
        - boss_radius
        for bolt in bolt_positions
    ]
    chord_edge_margins = [
        min(bolt["x_mm"] - root_le_x, root_te_x - bolt["x_mm"])
        - boss_radius
        for bolt in bolt_positions
    ]
    bolt_to_spar_clearances = [
        abs(bolt["x_mm"] - spar_x)
        - boss_radius
        - structure["carbon_member_outer_mm"] / 2
        for bolt in bolt_positions
        for spar_x in spar_x_positions
    ]
    equipment_corridor_width = 2 * min(
        abs(bolt["y_mm"]) - boss_radius for bolt in bolt_positions
    )

    return {
        "spar_root_x_positions_mm": spar_x_positions,
        "spar_centerline_z_positions_mm": [
            structure["forward_spar_centerline_z_mm"],
            structure["rear_spar_centerline_z_mm"],
        ],
        "fastener_count": fasteners["count"],
        "fastener_positions_mm": bolt_positions,
        "minimum_boss_to_fuselage_side_margin_mm": min(side_margins),
        "minimum_boss_to_root_chord_edge_margin_mm": min(chord_edge_margins),
        "minimum_boss_to_spar_clearance_mm": min(bolt_to_spar_clearances),
        "equipment_clear_corridor_width_mm": equipment_corridor_width,
        "required_equipment_corridor_width_mm": interface[
            "equipment_clear_corridor_width_mm"
        ],
        "saddle_contact_half_width_mm": interface["saddle_contact_half_width_mm"],
    }


def build_airplane(
    config: dict[str, Any],
    elevator_deflection_deg: float = 0.0,
    aileron_deflection_deg: float = 0.0,
    rudder_deflection_deg: float = 0.0,
) -> asb.Airplane:
    wing_config = config["wing"]
    reference = config["reference"]
    root_le_x = mm(reference["wing_root_le_x_mm"])
    root_z = mm(reference["wing_root_z_mm"])
    root_chord = mm(wing_config["stations"][0]["chord_mm"])
    quarter_chord_x = root_le_x + 0.25 * root_chord
    dihedral_break = mm(wing_config["dihedral_break_y_mm"])
    outer_dihedral_deg = wing_config["outer_panel_dihedral_deg"]

    main_xsecs = []
    for station in wing_config["stations"]:
        y = mm(station["y_mm"])
        chord = mm(station["chord_mm"])
        controls = []
        if wing_config["aileron_start_mm"] <= station["y_mm"] <= wing_config["aileron_end_mm"]:
            controls.append(
                asb.ControlSurface(
                    name="Aileron",
                    symmetric=False,
                    deflection=aileron_deflection_deg,
                    hinge_point=wing_config["aileron_hinge_fraction"],
                )
            )

        main_xsecs.append(
            asb.WingXSec(
                xyz_le=[
                    quarter_chord_x - 0.25 * chord,
                    y,
                    root_z
                    + max(0.0, y - dihedral_break) * np.tand(outer_dihedral_deg),
                ],
                chord=chord,
                twist=station["twist_deg"],
                airfoil=asb.Airfoil(station["airfoil"]),
                control_surfaces=controls,
            )
        )

    main_wing = asb.Wing(name="Main Wing", symmetric=True, xsecs=main_xsecs)

    horizontal = config["horizontal_tail"]
    elevator = [
        asb.ControlSurface(
            name="Elevator",
            symmetric=True,
            deflection=elevator_deflection_deg,
            hinge_point=horizontal["elevator_hinge_fraction"],
        )
    ]
    horizontal_tail = asb.Wing(
        name="Horizontal Tail",
        symmetric=True,
        xsecs=[
            asb.WingXSec(
                xyz_le=[mm(horizontal["root_le_x_mm"]), 0.0, mm(horizontal["z_mm"])],
                chord=mm(horizontal["root_chord_mm"]),
                airfoil=asb.Airfoil(horizontal["airfoil"]),
                control_surfaces=elevator,
            ),
            asb.WingXSec(
                xyz_le=[
                    mm(horizontal["tip_le_x_mm"]),
                    mm(horizontal["span_mm"]) / 2,
                    mm(horizontal["z_mm"]),
                ],
                chord=mm(horizontal["tip_chord_mm"]),
                airfoil=asb.Airfoil(horizontal["airfoil"]),
                control_surfaces=elevator,
            ),
        ],
    )

    vertical = config["vertical_tail"]
    rudder = [
        asb.ControlSurface(
            name="Rudder",
            symmetric=True,
            deflection=rudder_deflection_deg,
            hinge_point=vertical["rudder_hinge_fraction"],
        )
    ]
    vertical_tail = asb.Wing(
        name="Vertical Tail",
        symmetric=False,
        xsecs=[
            asb.WingXSec(
                xyz_le=[mm(vertical["root_le_x_mm"]), 0.0, 0.0],
                chord=mm(vertical["root_chord_mm"]),
                airfoil=asb.Airfoil(vertical["airfoil"]),
                control_surfaces=rudder,
            ),
            asb.WingXSec(
                xyz_le=[
                    mm(vertical["tip_le_x_mm"]),
                    0.0,
                    mm(vertical["height_mm"]),
                ],
                chord=mm(vertical["tip_chord_mm"]),
                airfoil=asb.Airfoil(vertical["airfoil"]),
                control_surfaces=rudder,
            ),
        ],
    )

    fuselage = asb.Fuselage(
        name="Fuselage",
        xsecs=[
            asb.FuselageXSec(
                xyz_c=[mm(section["x_mm"]), 0.0, mm(section["z_mm"])],
                width=mm(section["width_mm"]),
                height=mm(section["height_mm"]),
            )
            for section in config["fuselage"]["sections"]
        ],
    )

    area, mean_aerodynamic_chord = integrate_planform(wing_config["stations"])
    cg_x = root_le_x + mm(reference["cg_aft_of_root_le_mm"])

    return asb.Airplane(
        name=config["project"],
        xyz_ref=[cg_x, 0.0, 0.0],
        wings=[main_wing, horizontal_tail, vertical_tail],
        fuselages=[fuselage],
        s_ref=area,
        c_ref=mean_aerodynamic_chord,
        b_ref=mm(wing_config["span_mm"]),
    )


def run_aero(
    config: dict[str, Any],
    alpha_deg: float,
    elevator_deg: float,
    beta_deg: float = 0.0,
) -> dict[str, Any]:
    airplane = build_airplane(config, elevator_deflection_deg=elevator_deg)
    operating_point = asb.OperatingPoint(
        velocity=config["operating_conditions"]["cruise_speed_m_s"],
        alpha=alpha_deg,
        beta=beta_deg,
    )
    return asb.AeroBuildup(airplane=airplane, op_point=operating_point).run()


def solve_cruise_trim(config: dict[str, Any]) -> tuple[float, float, dict[str, float]]:
    conditions = config["operating_conditions"]
    design_mass_kg = config["mass_budget"]["design_auw_g"] / 1000
    design_weight_n = design_mass_kg * conditions["gravity_m_s2"]

    opti = asb.Opti()
    alpha = opti.variable(init_guess=2.0, lower_bound=-4.0, upper_bound=12.0)
    elevator = opti.variable(init_guess=0.0, lower_bound=-15.0, upper_bound=15.0)
    airplane = build_airplane(config, elevator_deflection_deg=elevator)
    results = asb.AeroBuildup(
        airplane=airplane,
        op_point=asb.OperatingPoint(velocity=conditions["cruise_speed_m_s"], alpha=alpha),
    ).run()

    opti.subject_to([results["L"] == design_weight_n, results["Cm"] == 0.0])
    opti.minimize(1e-3 * elevator**2 + 1e-5 * alpha**2)
    solution = opti.solve(verbose=False, max_iter=400)

    trim_alpha = scalar(solution(alpha))
    trim_elevator = scalar(solution(elevator))
    trim_results = {
        key: scalar(solution(results[key]))
        for key in ["L", "D", "CL", "CD", "Cm", "D_induced", "D_profile"]
    }
    trim_results["L_D"] = trim_results["L"] / trim_results["D"]
    return trim_alpha, trim_elevator, trim_results


def finite_difference_derivatives(
    config: dict[str, Any], trim_alpha: float, trim_elevator: float
) -> dict[str, float]:
    angle_step_deg = 0.5
    total_step_rad = math.radians(2 * angle_step_deg)

    alpha_plus = run_aero(config, trim_alpha + angle_step_deg, trim_elevator)
    alpha_minus = run_aero(config, trim_alpha - angle_step_deg, trim_elevator)
    beta_plus = run_aero(config, trim_alpha, trim_elevator, beta_deg=angle_step_deg)
    beta_minus = run_aero(config, trim_alpha, trim_elevator, beta_deg=-angle_step_deg)

    return {
        "Cm_alpha_per_rad": (scalar(alpha_plus["Cm"]) - scalar(alpha_minus["Cm"])) / total_step_rad,
        "Cl_beta_per_rad": (scalar(beta_plus["Cl"]) - scalar(beta_minus["Cl"])) / total_step_rad,
        "Cn_beta_per_rad": (scalar(beta_plus["Cn"]) - scalar(beta_minus["Cn"])) / total_step_rad,
    }


def scan_clean_stall(
    config: dict[str, Any], elevator_deg: float
) -> dict[str, float]:
    samples = []
    for alpha in np.linspace(-4, 20, 49):
        results = run_aero(config, float(alpha), elevator_deg)
        samples.append((float(alpha), scalar(results["CL"])))

    stall_alpha, maximum_cl = max(samples, key=lambda item: item[1])
    conditions = config["operating_conditions"]
    area, _ = integrate_planform(config["wing"]["stations"])
    mass_kg = config["mass_budget"]["design_auw_g"] / 1000
    weight = mass_kg * conditions["gravity_m_s2"]
    stall_speed = math.sqrt(
        2 * weight / (conditions["air_density_kg_m3"] * area * maximum_cl)
    )

    return {
        "estimated_stall_alpha_deg": stall_alpha,
        "estimated_CL_max": maximum_cl,
        "estimated_stall_speed_m_s": stall_speed,
        "estimated_rotation_speed_m_s": 1.15 * stall_speed,
    }


def geometry_summary(config: dict[str, Any]) -> dict[str, float]:
    wing = config["wing"]
    area, mean_aerodynamic_chord = integrate_planform(wing["stations"])
    span = mm(wing["span_mm"])
    aspect_ratio = span**2 / area
    loading = config["mass_budget"]["design_auw_g"] / (area * 100)

    horizontal = config["horizontal_tail"]
    horizontal_area = mm(horizontal["span_mm"]) * 0.5 * (
        mm(horizontal["root_chord_mm"]) + mm(horizontal["tip_chord_mm"])
    )
    vertical = config["vertical_tail"]
    vertical_area = mm(vertical["height_mm"]) * 0.5 * (
        mm(vertical["root_chord_mm"]) + mm(vertical["tip_chord_mm"])
    )

    wing_quarter_chord_x = mm(config["reference"]["wing_root_le_x_mm"]) + 0.25 * mm(
        wing["stations"][0]["chord_mm"]
    )
    horizontal_quarter_chord_x = 0.5 * (
        mm(horizontal["root_le_x_mm"]) + 0.25 * mm(horizontal["root_chord_mm"])
        + mm(horizontal["tip_le_x_mm"]) + 0.25 * mm(horizontal["tip_chord_mm"])
    )
    vertical_quarter_chord_x = 0.5 * (
        mm(vertical["root_le_x_mm"]) + 0.25 * mm(vertical["root_chord_mm"])
        + mm(vertical["tip_le_x_mm"]) + 0.25 * mm(vertical["tip_chord_mm"])
    )
    horizontal_arm = horizontal_quarter_chord_x - wing_quarter_chord_x
    vertical_arm = vertical_quarter_chord_x - wing_quarter_chord_x

    return {
        "wing_area_m2": area,
        "wing_area_dm2": area * 100,
        "mean_aerodynamic_chord_mm": mean_aerodynamic_chord * 1000,
        "aspect_ratio": aspect_ratio,
        "wing_loading_g_dm2": loading,
        "tip_rise_mm": (
            wing["span_mm"] / 2 - wing["dihedral_break_y_mm"]
        )
        * math.tan(math.radians(wing["outer_panel_dihedral_deg"])),
        "horizontal_tail_area_dm2": horizontal_area * 100,
        "vertical_tail_area_dm2": vertical_area * 100,
        "horizontal_tail_arm_mm": horizontal_arm * 1000,
        "vertical_tail_arm_mm": vertical_arm * 1000,
        "horizontal_tail_volume": horizontal_area * horizontal_arm / (area * mean_aerodynamic_chord),
        "vertical_tail_volume": vertical_area * vertical_arm / (area * span),
        "servo_available_depth_mm": wing["servo"]["available_gross_section_depth_mm"],
        "servo_required_depth_mm": wing["servo"]["required_gross_section_depth_mm"],
    }


def acceptance_checks(
    config: dict[str, Any],
    trim_alpha: float,
    trim_elevator: float,
    trim_results: dict[str, float],
    derivatives: dict[str, float],
    geometry: dict[str, float],
    mounting: dict[str, Any],
) -> dict[str, bool]:
    conditions = config["operating_conditions"]
    design_weight = (
        config["mass_budget"]["design_auw_g"]
        / 1000
        * conditions["gravity_m_s2"]
    )
    structure = config["wing"]["structure"]
    interface = config["wing_fuselage_interface"]
    return {
        "design_mass_within_declared_budget": config["mass_budget"]["design_auw_g"]
        <= config["mass_budget"]["component_mass_g"]
        + config["mass_budget"]["printed_airframe_max_g"],
        "lift_matches_design_weight": abs(trim_results["L"] - design_weight) < 0.01,
        "cruise_pitch_is_trimmed": abs(trim_results["Cm"]) < 1e-5,
        "trim_alpha_within_8_deg": abs(trim_alpha) < 8.0,
        "trim_elevator_within_8_deg": abs(trim_elevator) < 8.0,
        "pitch_statically_stable": derivatives["Cm_alpha_per_rad"] < 0.0,
        "dihedral_effect_is_stable": derivatives["Cl_beta_per_rad"] < 0.0,
        "directionally_stable": derivatives["Cn_beta_per_rad"] > 0.0,
        "mg90s_depth_requirement_met": geometry["servo_available_depth_mm"]
        >= geometry["servo_required_depth_mm"],
        "high_wing_mount_selected": config["reference"]["wing_mount"] == "high-wing"
        and config["reference"]["wing_root_z_mm"] > 0.0,
        "two_identical_6x4x1000_spars_specified": structure["spar_count"] == 2
        and structure["carbon_member_outer_mm"] == 6.0
        and structure["carbon_member_inner_mm"] == 4.0
        and structure["carbon_member_length_mm"] == 1000.0,
        "both_spars_supported_by_bulkheads": interface["spar_support_bulkheads"][
            "count"
        ]
        == structure["spar_count"]
        and interface["spar_support_bulkheads"][
            "aligned_with_spar_centerline_x_mm"
        ]
        == [
            structure["forward_spar_centerline_x_mm"],
            structure["rear_spar_centerline_x_mm"],
        ]
        and interface["spar_support_bulkheads"][
            "aligned_with_spar_centerline_z_mm"
        ]
        == [
            structure["forward_spar_centerline_z_mm"],
            structure["rear_spar_centerline_z_mm"],
        ],
        "four_m3_fasteners_specified": interface["fasteners"]["count"] == 4
        and interface["fasteners"]["thread"] == "M3"
        and len(interface["fasteners"]["positions_mm"]) == 4,
        "mount_bosses_inside_fuselage": mounting[
            "minimum_boss_to_fuselage_side_margin_mm"
        ]
        >= 2.0,
        "mount_bosses_clear_wing_edges": mounting[
            "minimum_boss_to_root_chord_edge_margin_mm"
        ]
        >= 20.0,
        "mount_bosses_clear_spars": mounting["minimum_boss_to_spar_clearance_mm"]
        >= 20.0,
        "equipment_corridor_preserved": mounting["equipment_clear_corridor_width_mm"]
        >= mounting["required_equipment_corridor_width_mm"],
    }


def main() -> None:
    config = load_config()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    CAD_REFERENCE_DIR.mkdir(parents=True, exist_ok=True)

    trim_alpha, trim_elevator, trim_results = solve_cruise_trim(config)
    derivatives = finite_difference_derivatives(config, trim_alpha, trim_elevator)
    stall = scan_clean_stall(config, trim_elevator)
    geometry = geometry_summary(config)
    mounting = mounting_interface_summary(config)
    checks = acceptance_checks(
        config,
        trim_alpha,
        trim_elevator,
        trim_results,
        derivatives,
        geometry,
        mounting,
    )

    summary = {
        "project": config["project"],
        "phase": config["phase"],
        "status": "pass" if all(checks.values()) else "review_required",
        "configuration": {
            "wing_mount": config["reference"]["wing_mount"],
            "wing_root_z_mm": config["reference"]["wing_root_z_mm"],
            "wing_structure": config["wing"]["structure"],
            "wing_fuselage_interface": config["wing_fuselage_interface"],
        },
        "mass_budget": config["mass_budget"],
        "geometry": geometry,
        "mounting_interface_checks": mounting,
        "cruise_trim": {
            "speed_m_s": config["operating_conditions"]["cruise_speed_m_s"],
            "alpha_deg": trim_alpha,
            "elevator_deg": trim_elevator,
            **trim_results,
        },
        "static_stability": derivatives,
        "clean_stall_estimate": stall,
        "acceptance_checks": checks,
        "notes": [
            "Phase 3 uses AeroBuildup for the complete-aircraft baseline.",
            "Stall values are preliminary and must be replaced by the Phase 4 multipoint and section-margin analysis.",
            "Propeller performance is not inferred from the 2212 motor size; measured thrust and current remain required.",
        ],
    }

    summary_path = RESULTS_DIR / "v201_baseline_summary.json"
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2)

    reference_airplane = build_airplane(config, elevator_deflection_deg=trim_elevator)
    figure = reference_airplane.draw(backend="plotly", show=False)
    html_path = CAD_REFERENCE_DIR / "v201_baseline.html"
    figure.write_html(str(html_path))

    three_view_axes = reference_airplane.draw_three_view(show=False)
    three_view_path = CAD_REFERENCE_DIR / "v201_baseline_three_view.png"
    three_view_axes.reshape(-1)[0].figure.savefig(
        three_view_path, dpi=180, bbox_inches="tight"
    )

    print(f"V-201 Phase 3 status: {summary['status']}")
    print(f"Cruise trim: alpha={trim_alpha:.3f} deg, elevator={trim_elevator:.3f} deg")
    print(f"Wing: {geometry['wing_area_dm2']:.3f} dm^2, AR={geometry['aspect_ratio']:.3f}")
    print(
        "Stability: "
        f"Cm_alpha={derivatives['Cm_alpha_per_rad']:.4f}/rad, "
        f"Cl_beta={derivatives['Cl_beta_per_rad']:.4f}/rad, "
        f"Cn_beta={derivatives['Cn_beta_per_rad']:.4f}/rad"
    )
    print(f"Summary: {summary_path}")
    print(f"CAD reference: {html_path}")
    print(f"Three-view reference: {three_view_path}")


if __name__ == "__main__":
    main()
