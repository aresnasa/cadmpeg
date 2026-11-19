"""Reduced-order gear-pump analysis driven by decoded NX part geometry.

The model is a steady-state external gear pump with laminar clearance
leakage, a lumped mechanical-loss model, and closed-form gear, shaft,
bearing and housing checks. It is an engineering screening tool. It does
not replace NX Motion or NX CAE; the accompanying NX Open journal drives
those solvers inside NX.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable

TAU = 2.0 * math.pi


@dataclass
class GearGeometry:
    teeth: int
    module_mm: float
    pressure_angle_deg: float
    face_width_mm: float
    pitch_radius_mm: float
    tip_radius_mm: float
    root_radius_mm: float
    center_distance_mm: float

    def validate(self) -> None:
        if self.teeth < 6:
            raise ValueError("gear teeth must be at least 6")
        if not (0.2 <= self.module_mm <= 20.0):
            raise ValueError("gear module is outside 0.2..20 mm")
        if not (5.0 <= self.pressure_angle_deg <= 35.0):
            raise ValueError("pressure angle is outside 5..35 degrees")
        if not (0.5 <= self.face_width_mm <= 500.0):
            raise ValueError("face width is outside 0.5..500 mm")
        expected_pitch = 0.5 * self.module_mm * self.teeth
        if abs(self.pitch_radius_mm - expected_pitch) > 1e-6 * max(1.0, expected_pitch):
            raise ValueError("pitch radius disagrees with module and tooth count")
        expected_center = 2.0 * self.pitch_radius_mm
        if abs(self.center_distance_mm - expected_center) > 1e-6 * expected_center:
            raise ValueError("center distance disagrees with pitch radius")

    def displacement_cc(self) -> float:
        """Nominal displaced volume per revolution for the pair, in cm³."""
        tip = self.tip_radius_mm
        pitch = self.pitch_radius_mm
        return TAU * self.face_width_mm * (tip * tip - pitch * pitch) / 1000.0

    def theoretical_flow_lpm(self, rpm: float) -> float:
        return self.displacement_cc() * rpm / 1000.0


@dataclass
class OilProperties:
    name: str = "ISO VG 46"
    density_kg_m3: float = 870.0
    specific_heat_j_kgk: float = 1900.0
    viscosity_40c_pas: float = 0.040
    viscosity_temperature_k: float = 3000.0
    bulk_modulus_pa: float = 1.5e9
    vapour_pressure_bar: float = 0.02

    def viscosity_pas(self, temperature_c: float) -> float:
        temperature_k = temperature_c + 273.15
        reference_k = 313.15
        exponent = self.viscosity_temperature_k * (1.0 / temperature_k - 1.0 / reference_k)
        return self.viscosity_40c_pas * math.exp(exponent)


@dataclass
class Clearances:
    tip_mm: float = 0.020
    side_mm: float = 0.030
    shaft_seal_mm: float = 0.010
    thermal_expansion_gear_1_k: float = 11.5e-6
    thermal_expansion_housing_1_k: float = 10.0e-6

    def validate(self) -> None:
        for name, value in (
            ("tip", self.tip_mm),
            ("side", self.side_mm),
            ("shaft_seal", self.shaft_seal_mm),
        ):
            if not (0.0 < value <= 1.0):
                raise ValueError(f"{name} clearance is outside 0..1 mm")


@dataclass
class Shaft:
    diameter_mm: float = 18.0
    bearing_span_mm: float = 80.0
    material: str = "40Cr"
    youngs_modulus_pa: float = 206.0e9
    yield_mpa: float = 785.0
    allowable_mpa: float = 300.0
    density_kg_m3: float = 7850.0


@dataclass
class Bearing:
    designation: str = "6206"
    dynamic_load_n: float = 19500.0
    static_load_n: float = 11200.0
    count: int = 2


@dataclass
class Housing:
    material: str = "HT250"
    wall_thickness_mm: float = 10.0
    bore_diameter_mm: float = 48.0
    bolt_count: int = 8
    bolt_diameter_mm: float = 8.0
    bolt_allowable_mpa: float = 400.0
    allowable_hoop_mpa: float = 80.0


@dataclass
class Limits:
    max_volumetric_efficiency_drop: float = 0.25
    min_volumetric_efficiency: float = 0.70
    min_bending_safety: float = 1.5
    min_contact_safety: float = 1.2
    min_shaft_safety: float = 2.0
    min_bearing_life_h: float = 5000.0
    min_housing_safety: float = 1.5
    max_oil_rise_k: float = 45.0
    min_npsh_margin_m: float = 1.0
    max_pressure_ripple_pct: float = 8.0


@dataclass
class GearPumpModel:
    name: str
    gear: GearGeometry
    oil: OilProperties = field(default_factory=OilProperties)
    clearances: Clearances = field(default_factory=Clearances)
    shaft: Shaft = field(default_factory=Shaft)
    bearing: Bearing = field(default_factory=Bearing)
    housing: Housing = field(default_factory=Housing)
    limits: Limits = field(default_factory=Limits)
    rated_pressure_bar: float = 100.0
    peak_pressure_bar: float = 160.0
    rated_speed_rpm: float = 1500.0
    nominal_volumetric_efficiency: float = 0.92
    provenance: dict[str, Any] = field(default_factory=dict)

    def validate(self) -> None:
        self.gear.validate()
        self.clearances.validate()
        if self.shaft.diameter_mm <= 0:
            raise ValueError("shaft diameter must be positive")
        if self.housing.wall_thickness_mm <= 0:
            raise ValueError("housing wall thickness must be positive")
        if self.bearing.count <= 0:
            raise ValueError("bearing count must be positive")

    def leakage_conductance(self) -> tuple[float, float]:
        """Return tip and side leakage conductances in m³/(s·Pa).

        Tip leakage is modelled as pressure-driven flow through the tooth-tip
        land. Side leakage is radial flow between each gear face and the
        housing. The side term uses the gear face annulus from shaft radius to
        tip radius.
        """
        gear = self.gear
        b = gear.face_width_mm / 1000.0
        tip = gear.tip_radius_mm / 1000.0
        shaft = 0.5 * self.shaft.diameter_mm / 1000.0
        land = max(0.001, 0.3 * gear.module_mm / 1000.0)
        # Two gears, both flanks of each tooth tip participate over the
        # pressure half of the circumference.
        tip_conductance = 2.0 * gear.teeth * b / (12.0 * land)
        # Four face gaps (two gears, two faces each), radial annulus.
        ratio = max(1.000001, tip / max(shaft, 1.0e-4))
        side_conductance = 4.0 * math.pi / (6.0 * math.log(ratio))
        return tip_conductance, side_conductance

    def slip_lpm(
        self,
        pressure_bar: float,
        viscosity_pas: float,
        tip_clearance_mm: float | None = None,
        side_clearance_mm: float | None = None,
    ) -> float:
        tip_conductance, side_conductance = self.leakage_conductance()
        pressure_pa = pressure_bar * 1.0e5
        tip = (
            self.clearances.tip_mm if tip_clearance_mm is None else tip_clearance_mm
        ) / 1000.0
        side = (
            self.clearances.side_mm if side_clearance_mm is None else side_clearance_mm
        ) / 1000.0
        flow_m3_s = (
            pressure_pa
            * (tip_conductance * tip**3 + side_conductance * side**3)
            / max(viscosity_pas, 1.0e-6)
        )
        return flow_m3_s * 60000.0

    def calibrated_slip_scale(self) -> float:
        """Scale the analytical leakage to the nominal volumetric efficiency."""
        viscosity = self.oil.viscosity_pas(40.0)
        theoretical = self.gear.theoretical_flow_lpm(self.rated_speed_rpm)
        slip = self.slip_lpm(self.rated_pressure_bar, viscosity)
        target = theoretical * (1.0 - self.nominal_volumetric_efficiency)
        return 1.0 if slip <= 0.0 else target / slip


@dataclass
class Condition:
    name: str
    speed_rpm: float
    discharge_pressure_bar: float
    inlet_pressure_bar: float = 1.0
    oil_temperature_c: float = 40.0
    aeration_percent: float = 0.0
    tip_clearance_mm: float | None = None
    side_clearance_mm: float | None = None
    outlet_volume_l: float = 0.5
    duty_cycle_percent: float = 100.0
    note: str = ""

    def validate(self) -> None:
        if self.speed_rpm < 0:
            raise ValueError("speed must be non-negative")
        if self.discharge_pressure_bar < 0:
            raise ValueError("discharge pressure must be non-negative")
        if self.oil_temperature_c < -40 or self.oil_temperature_c > 200:
            raise ValueError("oil temperature is outside -40..200 C")
        if not (0.0 <= self.aeration_percent <= 20.0):
            raise ValueError("aeration is outside 0..20 percent")


@dataclass
class Check:
    name: str
    value: float
    limit: float
    unit: str
    margin: float
    passed: bool
    note: str = ""


@dataclass
class SimulationResult:
    condition: Condition
    flow_lpm: float
    theoretical_flow_lpm: float
    slip_lpm: float
    volumetric_efficiency: float
    input_power_kw: float
    hydraulic_power_kw: float
    loss_power_kw: float
    input_torque_nm: float
    oil_rise_k: float
    oil_viscosity_pas: float
    reynolds_number: float
    tooth_load_n: float
    radial_load_n: float
    bending_stress_mpa: float
    contact_stress_mpa: float
    shaft_torsion_mpa: float
    shaft_bending_mpa: float
    shaft_von_mises_mpa: float
    shaft_deflection_mm: float
    bearing_load_n: float
    bearing_life_h: float
    housing_hoop_mpa: float
    bolt_stress_mpa: float
    npsh_available_m: float
    npsh_required_m: float
    flow_ripple_pct: float
    pressure_ripple_pct: float
    checks: list[Check]
    passed: bool = True

    def to_row(self) -> dict[str, Any]:
        row = asdict(self.condition)
        row.update(
            {
                key: value
                for key, value in asdict(self).items()
                if key not in ("condition", "checks")
            }
        )
        row["checks"] = len(self.checks)
        row["failed_checks"] = sum(1 for check in self.checks if not check.passed)
        return row


def simulate(model: GearPumpModel, condition: Condition) -> SimulationResult:
    model.validate()
    condition.validate()
    gear = model.gear
    oil = model.oil
    rpm = condition.speed_rpm
    omega = TAU * rpm / 60.0
    pressure_pa = condition.discharge_pressure_bar * 1.0e5
    viscosity = oil.viscosity_pas(condition.oil_temperature_c)
    density = oil.density_kg_m3
    tip_clearance = (
        model.clearances.tip_mm
        if condition.tip_clearance_mm is None
        else condition.tip_clearance_mm
    )
    side_clearance = (
        model.clearances.side_mm
        if condition.side_clearance_mm is None
        else condition.side_clearance_mm
    )

    theoretical = gear.theoretical_flow_lpm(rpm)
    scale = model.calibrated_slip_scale()
    slip = (
        model.slip_lpm(
            condition.discharge_pressure_bar,
            viscosity,
            tip_clearance,
            side_clearance,
        )
        * scale
    )
    if condition.discharge_pressure_bar <= 0.0:
        slip = 0.0
    flow = max(0.0, theoretical - slip)
    eta_v = 1.0 if theoretical <= 0.0 else flow / theoretical
    eta_v *= max(0.0, 1.0 - 0.015 * condition.aeration_percent)
    flow = theoretical * eta_v

    hydraulic_power = pressure_pa * flow / 60000.0
    no_load_torque = 0.8 + 0.25 * (rpm / 1000.0) ** 2
    viscous_torque = (
        4.0
        * viscosity
        * omega
        * math.pi
        * (gear.tip_radius_mm / 1000.0) ** 4
        / (2.0 * max(side_clearance / 1000.0, 1.0e-6))
    )
    mechanical_torque = 0.01 * hydraulic_power / max(omega, 1.0e-9)
    input_torque = hydraulic_power / max(omega, 1.0e-9) + no_load_torque + viscous_torque + mechanical_torque
    input_power = input_torque * omega
    loss_power = max(0.0, input_power - hydraulic_power)
    eta_total = 1.0 if input_power <= 0.0 else hydraulic_power / input_power

    mass_flow = flow / 60000.0 * density
    oil_rise = (
        0.0
        if mass_flow <= 1.0e-9
        else loss_power / (mass_flow * oil.specific_heat_j_kgk)
    )

    pitch_m = gear.pitch_radius_mm / 1000.0
    face_m = gear.face_width_mm / 1000.0
    module_m = gear.module_mm / 1000.0
    pressure_angle = math.radians(gear.pressure_angle_deg)
    tooth_load = input_torque / max(pitch_m, 1.0e-9)
    radial_load = tooth_load * math.tan(pressure_angle)
    lewis_factor = 0.32
    bending_stress = tooth_load / max(face_m * module_m * lewis_factor, 1.0e-12) / 1.0e6
    contact_zone = math.sqrt(
        max(
            0.0,
            tooth_load
            / max(gear.face_width_mm * (2.0 * gear.pitch_radius_mm), 1.0e-12),
        )
    )
    contact_stress = 189.8 * contact_zone * math.sqrt(2.0)

    shaft_d = model.shaft.diameter_mm / 1000.0
    torsion = 16.0 * input_torque / (math.pi * shaft_d**3) / 1.0e6
    span = model.shaft.bearing_span_mm / 1000.0
    bending_moment = radial_load * span / 4.0
    bending = 32.0 * bending_moment / (math.pi * shaft_d**3) / 1.0e6
    von_mises = math.sqrt(bending**2 + 3.0 * torsion**2)
    inertia = math.pi * shaft_d**4 / 64.0
    deflection = (
        radial_load * span**3 / (48.0 * model.shaft.youngs_modulus_pa * inertia) * 1000.0
    )

    bearing_load = math.hypot(tooth_load, radial_load)
    dynamic = model.bearing.dynamic_load_n
    bearing_life = (
        (dynamic / max(bearing_load, 1.0)) ** 3 * 1.0e6 / max(60.0 * rpm, 1.0)
    )

    hoop = (
        pressure_pa
        * (model.housing.bore_diameter_mm / 1000.0)
        / (2.0 * (model.housing.wall_thickness_mm / 1000.0))
        / 1.0e6
    )
    bolt_area = math.pi * (model.housing.bolt_diameter_mm / 1000.0) ** 2 / 4.0
    end_load = pressure_pa * math.pi * (model.housing.bore_diameter_mm / 1000.0) ** 2 / 4.0
    bolt_stress = end_load / max(model.housing.bolt_count * bolt_area, 1.0e-9) / 1.0e6

    inlet_pa = condition.inlet_pressure_bar * 1.0e5
    vapour_pa = oil.vapour_pressure_bar * 1.0e5
    inlet_velocity = 2.0
    npsh_available = max(
        0.0, (inlet_pa - vapour_pa) / (density * 9.80665)
    ) + inlet_velocity**2 / (2.0 * 9.80665)
    npsh_required = 1.5 + 0.4 * (rpm / 1000.0) ** 1.5 + 0.1 * condition.discharge_pressure_bar / 10.0

    flow_ripple = 100.0 * 0.5 / gear.teeth
    pressure_ripple = flow_ripple * (0.15 + 0.002 * condition.discharge_pressure_bar)
    if condition.outlet_volume_l > 0.0:
        pressure_ripple *= min(2.0, 0.5 / condition.outlet_volume_l) ** 0.5

    checks = [
        Check(
            "volumetric_efficiency",
            eta_v * 100.0,
            model.limits.min_volumetric_efficiency * 100.0,
            "%",
            eta_v - model.limits.min_volumetric_efficiency,
            eta_v >= model.limits.min_volumetric_efficiency,
        ),
        Check(
            "gear_bending_stress",
            bending_stress,
            300.0 / model.limits.min_bending_safety,
            "MPa",
            (300.0 / model.limits.min_bending_safety - bending_stress) / 300.0,
            bending_stress <= 300.0 / model.limits.min_bending_safety,
            "allowable 300 MPa / safety factor",
        ),
        Check(
            "gear_contact_stress",
            contact_stress,
            1200.0 / model.limits.min_contact_safety,
            "MPa",
            (1200.0 / model.limits.min_contact_safety - contact_stress) / 1200.0,
            contact_stress <= 1200.0 / model.limits.min_contact_safety,
            "allowable 1200 MPa / safety factor",
        ),
        Check(
            "shaft_von_mises",
            von_mises,
            model.shaft.allowable_mpa / model.limits.min_shaft_safety,
            "MPa",
            (model.shaft.allowable_mpa / model.limits.min_shaft_safety - von_mises)
            / model.shaft.allowable_mpa,
            von_mises <= model.shaft.allowable_mpa / model.limits.min_shaft_safety,
        ),
        Check(
            "shaft_deflection",
            deflection,
            0.05,
            "mm",
            (0.05 - deflection) / 0.05,
            deflection <= 0.05,
            "gear-mesh deflection budget",
        ),
        Check(
            "bearing_life",
            bearing_life,
            model.limits.min_bearing_life_h,
            "h",
            (bearing_life - model.limits.min_bearing_life_h)
            / model.limits.min_bearing_life_h,
            bearing_life >= model.limits.min_bearing_life_h,
        ),
        Check(
            "housing_hoop",
            hoop,
            model.housing.allowable_hoop_mpa / model.limits.min_housing_safety,
            "MPa",
            (model.housing.allowable_hoop_mpa / model.limits.min_housing_safety - hoop)
            / model.housing.allowable_hoop_mpa,
            hoop <= model.housing.allowable_hoop_mpa / model.limits.min_housing_safety,
        ),
        Check(
            "bolt_stress",
            bolt_stress,
            model.housing.bolt_allowable_mpa,
            "MPa",
            (model.housing.bolt_allowable_mpa - bolt_stress)
            / model.housing.bolt_allowable_mpa,
            bolt_stress <= model.housing.bolt_allowable_mpa,
        ),
        Check(
            "npsh_margin",
            npsh_available - npsh_required,
            model.limits.min_npsh_margin_m,
            "m",
            npsh_available - npsh_required - model.limits.min_npsh_margin_m,
            npsh_available - npsh_required >= model.limits.min_npsh_margin_m,
        ),
        Check(
            "oil_temperature_rise",
            oil_rise,
            model.limits.max_oil_rise_k,
            "K",
            (model.limits.max_oil_rise_k - oil_rise)
            / model.limits.max_oil_rise_k,
            oil_rise <= model.limits.max_oil_rise_k,
        ),
        Check(
            "pressure_ripple",
            pressure_ripple,
            model.limits.max_pressure_ripple_pct,
            "%",
            (model.limits.max_pressure_ripple_pct - pressure_ripple)
            / model.limits.max_pressure_ripple_pct,
            pressure_ripple <= model.limits.max_pressure_ripple_pct,
        ),
    ]
    passed = all(check.passed for check in checks)
    return SimulationResult(
        condition=condition,
        flow_lpm=flow,
        theoretical_flow_lpm=theoretical,
        slip_lpm=slip,
        volumetric_efficiency=eta_v,
        input_power_kw=input_power / 1000.0,
        hydraulic_power_kw=hydraulic_power / 1000.0,
        loss_power_kw=loss_power / 1000.0,
        input_torque_nm=input_torque,
        oil_rise_k=oil_rise,
        oil_viscosity_pas=viscosity,
        reynolds_number=(
            density * omega * pitch_m * (2.0 * pitch_m) / max(viscosity, 1.0e-9)
        ),
        tooth_load_n=tooth_load,
        radial_load_n=radial_load,
        bending_stress_mpa=bending_stress,
        contact_stress_mpa=contact_stress,
        shaft_torsion_mpa=torsion,
        shaft_bending_mpa=bending,
        shaft_von_mises_mpa=von_mises,
        shaft_deflection_mm=deflection,
        bearing_load_n=bearing_load,
        bearing_life_h=bearing_life,
        housing_hoop_mpa=hoop,
        bolt_stress_mpa=bolt_stress,
        npsh_available_m=npsh_available,
        npsh_required_m=npsh_required,
        flow_ripple_pct=flow_ripple,
        pressure_ripple_pct=pressure_ripple,
        checks=checks,
        passed=passed,
    )


def default_model() -> GearPumpModel:
    model = GearPumpModel(
        name="NX10 gear pump 齿轮泵UG10.0",
        gear=GearGeometry(
            teeth=14,
            module_mm=3.0,
            pressure_angle_deg=20.0,
            face_width_mm=25.0,
            pitch_radius_mm=21.0,
            tip_radius_mm=24.0,
            root_radius_mm=17.25,
            center_distance_mm=42.0,
        ),
        provenance={
            "corpus": r"C:\RE\artifacts\齿轮泵UG10.0",
            "driving_gear": "2.主动齿轮轴.prt",
            "driven_gear": "1.从动齿轮轴.prt",
            "housing": "10泵体.prt",
            "method": (
                "nxview gear measured 14 teeth, 24 mm tip radius, 25 mm face "
                "width on both shafts; nxview surfaces found the 17.25 mm root "
                "cylinder and the 42 mm housing bore spacing; module and pitch "
                "radius follow z=14 and r_a=24 mm."
            ),
            "limitations": (
                "Reduced-order screening model. Pressure ripple, contact ratio, "
                "and mechanical losses use engineering correlations, not a "
                "transient CFD or flexible-body solve. Use the NX Open journal "
                "for NX Motion and NX CAE."
            ),
        },
    )
    model.validate()
    return model


def model_from_dict(data: dict[str, Any]) -> GearPumpModel:
    model = GearPumpModel(
        name=data["name"],
        gear=GearGeometry(**data["gear"]),
        oil=OilProperties(**data.get("oil", {})),
        clearances=Clearances(**data.get("clearances", {})),
        shaft=Shaft(**data.get("shaft", {})),
        bearing=Bearing(**data.get("bearing", {})),
        housing=Housing(**data.get("housing", {})),
        limits=Limits(**data.get("limits", {})),
        rated_pressure_bar=data.get("rated_pressure_bar", 100.0),
        peak_pressure_bar=data.get("peak_pressure_bar", 160.0),
        rated_speed_rpm=data.get("rated_speed_rpm", 1500.0),
        nominal_volumetric_efficiency=data.get("nominal_volumetric_efficiency", 0.92),
        provenance=data.get("provenance", {}),
    )
    model.validate()
    return model


def load_model(path: str | Path | None = None) -> GearPumpModel:
    if path is None:
        return default_model()
    return model_from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def model_to_dict(model: GearPumpModel) -> dict[str, Any]:
    return asdict(model)


def standard_conditions() -> list[Condition]:
    return [
        Condition("cold_start", 300.0, 0.0, 1.0, 20.0, note="no-load cold start"),
        Condition("idle", 800.0, 10.0, 1.0, 40.0),
        Condition("rated", 1500.0, 100.0, 1.0, 40.0),
        Condition("rated_hot", 1500.0, 100.0, 1.0, 80.0),
        Condition("peak", 1000.0, 160.0, 1.0, 60.0),
        Condition("overspeed", 3000.0, 50.0, 1.0, 40.0),
        Condition("low_inlet", 2000.0, 100.0, 0.5, 60.0, 2.0),
        Condition("high_aeration", 1200.0, 80.0, 0.8, 50.0, 5.0),
        Condition("large_clearance", 1500.0, 100.0, 1.0, 60.0, 0.0, 0.035, 0.045),
    ]


def load_conditions(path: str | Path | None = None) -> list[Condition]:
    if path is None:
        return standard_conditions()
    rows = json.loads(Path(path).read_text(encoding="utf-8"))
    return [Condition(**row) for row in rows]


def sweep(
    model: GearPumpModel,
    speeds_rpm: Iterable[float] = (500, 1000, 1500, 2000, 2500, 3000),
    pressures_bar: Iterable[float] = (0, 25, 63, 100, 160),
    temperature_c: float = 40.0,
    inlet_pressure_bar: float = 1.0,
) -> list[SimulationResult]:
    results = []
    for speed in speeds_rpm:
        for pressure in pressures_bar:
            condition = Condition(
                name=f"{int(speed)}rpm_{int(pressure)}bar",
                speed_rpm=float(speed),
                discharge_pressure_bar=float(pressure),
                inlet_pressure_bar=inlet_pressure_bar,
                oil_temperature_c=temperature_c,
            )
            results.append(simulate(model, condition))
    return results


def results_to_json(
    model: GearPumpModel, results: list[SimulationResult]
) -> dict[str, Any]:
    return {
        "model": model_to_dict(model),
        "results": [
            {
                **asdict(result),
                "condition": asdict(result.condition),
                "checks": [asdict(check) for check in result.checks],
            }
            for result in results
        ],
    }


def write_csv(results: list[SimulationResult], path: str | Path) -> None:
    rows = [result.to_row() for result in results]
    if not rows:
        return
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_markdown(
    model: GearPumpModel, results: list[SimulationResult], path: str | Path
) -> None:
    lines = [
        f"# {model.name} — operating-condition screening",
        "",
        (
            f"Gear: z={model.gear.teeth}, m={model.gear.module_mm:g} mm, "
            f"b={model.gear.face_width_mm:g} mm, "
            f"r_a={model.gear.tip_radius_mm:g} mm, "
            f"a={model.gear.center_distance_mm:g} mm, "
            f"V={model.gear.displacement_cc():.2f} cm³/rev."
        ),
        "",
        "| Condition | Speed | p | T | Q | ηv | P_in | T_in | σ_F | σ_H | Bearing | ΔT | Result |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for result in results:
        c = result.condition
        lines.append(
            "| {name} | {speed:.0f} rpm | {p:.0f} bar | {t:.0f} °C | "
            "{q:.2f} L/min | {ev:.1f}% | {pin:.2f} kW | {torque:.1f} N·m | "
            "{sf:.0f} MPa | {sh:.0f} MPa | {life:.0f} h | {dt:.1f} K | {verdict} |".format(
                name=c.name,
                speed=c.speed_rpm,
                p=c.discharge_pressure_bar,
                t=c.oil_temperature_c,
                q=result.flow_lpm,
                ev=result.volumetric_efficiency * 100.0,
                pin=result.input_power_kw,
                torque=result.input_torque_nm,
                sf=result.bending_stress_mpa,
                sh=result.contact_stress_mpa,
                life=result.bearing_life_h,
                dt=result.oil_rise_k,
                verdict="PASS" if result.passed else "FAIL",
            )
        )
    lines.extend(
        [
            "",
            "## Failed checks",
            "",
        ]
    )
    failed = [
        (result.condition.name, check)
        for result in results
        for check in result.checks
        if not check.passed
    ]
    if not failed:
        lines.append("None.")
    else:
        for name, check in failed:
            lines.append(
                f"- `{name}` {check.name}: {check.value:.2f} {check.unit} "
                f"(limit {check.limit:.2f}, margin {check.margin:.2f})"
            )
    lines.extend(
        [
            "",
            "## Scope",
            "",
            model.provenance.get("limitations", ""),
            "",
            "The companion NX Open journal runs NX Motion and records the CAE "
            "configuration when an NX session and license are available.",
            "",
        ]
    )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")
