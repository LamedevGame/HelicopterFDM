# Helicopter FDM Plugin

Game-oriented helicopter flight dynamics model for Unreal Engine 5. A single Blueprint component drives realistic-feeling helicopter physics with intuitive parameters.

<img width="2554" height="1436" alt="image" src="https://github.com/user-attachments/assets/26ea3f43-573b-47c7-90cf-043b49b1e4c8" />

<img width="2559" height="1439" alt="image" src="https://github.com/user-attachments/assets/5fb79e1f-125d-4b67-b450-43cee48ce150" />

<img width="744" height="363" alt="image" src="https://github.com/user-attachments/assets/1cad6b05-ab49-43f5-b951-514e061ddbb9" />

https://www.youtube.com/watch?v=0exTaOAsJRo

Test Build Link: https://drive.google.com/file/d/1NSQLvvOrSDtkSRo9yvpTUv1pziJNpIC-/view?usp=sharing

## Tech Stack

| | |
|---|---|
| **Engine** | Unreal Engine 5.7 (C++) |
| **Physics** | UE Physics (`AddForce`, `AddTorqueInRadians`), custom FDM |
| **Models** | Cheeseman-Bennett ground effect, ETL, VRS, PID governor, physical tail rotor thrust |
| **Architecture** | Single Blueprint component, data-driven configuration |
| **Exposure** | Full Blueprint API (`BlueprintCallable`, `BlueprintReadOnly`) |

## Features

- Rotor thrust with ground effect (Cheeseman-Bennett 1955 model)
- Engine simulation with smoothstep spool curves and PID governor with collective feedforward
- Cyclic, collective, and pedal controls with asymmetric pitch smoothing
- Tail rotor with physical thrust formula: `T = ρ · A_disc · V_tip² · CT · pedal`
- Altitude-dependent thrust and power (`exp(-h / ceiling)`)
- Effective Translational Lift (ETL) — thrust kick around 15–25 knots
- Vortex Ring State (VRS) — thrust loss in fast vertical descent at low forward speed
- Stability Augmentation System (SAS) with rate damping and two-layer attitude hold
- Envelope protection (cubic pushback at pitch/roll limits)
- Hull and landing gear parasitic drag — body-frame anisotropic (lateral ≫ forward) with aerodynamic weather-cock yaw stability
- Auto-collective compensation and reactive-torque auto-trim toggles
- Lightweight per-tick: a single `AddForce` + a single `AddTorqueInRadians` call

## Installation

1. Copy the `HelicopterFDM` folder into your project's `Plugins/` directory
2. Regenerate project files
3. Enable the plugin in Edit > Plugins if needed
4. Add `"HelicopterFDM"` to your `.uproject` plugins list:

```json
{
  "Name": "HelicopterFDM",
  "Enabled": true
}
```

## Quick Start

1. Create an Actor or Pawn with a **Static Mesh** as root component (physics enabled, gravity on)
2. Add the **Helicopter FDM** component
3. Set **bEnabled = true**
4. Call `StartEngine()` to begin rotor spool-up
5. Use `SetCollective()`, `SetCyclicLongitudinal()`, `SetCyclicLateral()`, `SetPedals()` to control

### Minimal Blueprint Setup

```
BeginPlay:
  → StartEngine()

Tick:
  → SetCollective(value)         // 0..1, accumulated from input
  → SetCyclicLongitudinal(value) // -1..1
  → SetCyclicLateral(value)      // -1..1
  → SetPedals(value)             // -1..1
```

### Visual Blade Rotation

In your Blueprint Tick, rotate the rotor meshes:

```
RotorYaw     += GetMainRotorRPM() * 6.0 * DeltaTime
TailRotation += GetTailRotorRPM() * 6.0 * DeltaTime
```

`RPM * 6.0` converts RPM to degrees/second (360 / 60).

## Configuration

All parameters are exposed as Blueprint properties grouped by category. Universal physical constants (reactive-torque fraction, tail thrust coefficient, attitude-hold internals, envelope pushback strength) are baked into the implementation — they don't vary between airframes and don't clutter the Details panel.

### General

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bEnabled` | false | Master switch — FDM does nothing while false |
| `FieldElevationFeet` | 0 | Reference altitude for the altimeter zero |
| `ServiceCeilingFeet` | 15000 | Altitude where thrust + power drop to 37% (`exp(-h / ceiling)`) |

### Physics

| Parameter | Default | Description |
|-----------|---------|-------------|
| `InertiaTensor` | (2200, 7500, 6800) | Diagonal Ixx, Iyy, Izz (kg·m²) — applied via `InertiaTensorScale` |

Mass is read automatically from the root physics component.

### Main Rotor

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MainRotor.Position` | (0,0,0) | Hub position in cm. **Only Z is read** — moment arm above CoG for cyclic |
| `MainRotor.Radius` | 7.3 | Blade radius (m). Used for IGE, tip speed, torque arm |
| `ThrustMultiplier` | 1.25 | Max thrust at full collective as multiple of weight (1.0 = hover, 1.3 = climb) |
| `ThrustResponseTime` | 0.05 | Thrust smoothing time constant (s). 0 = instant |
| `CyclicMaxTilt` | 10.0 | Max disc tilt from cyclic input (degrees) |
| `MainRotorRPMNominal` | 394 | Governor target RPM |
| `bMainRotorClockwise` | true | true = CW from above (Mil/Eurocopter), false = CCW (Bell/Sikorsky) |

### Tail Rotor

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TailRotor.Position` | (0,0,0) | Hub position in cm. **Only X is read** — yaw moment arm |
| `TailRotor.Radius` | 7.3 | Tail blade radius (m) — used in physical thrust formula |
| `TailRotorGearRatio` | 5.32 | TailRPM = MainRPM × ratio |
| `PedalYawAuthority` | 1.0 | Fraction of available physical thrust used at full pedal |
| `bEnableTranslatingTendency` | true | Tail rotor thrust pushes body sideways (realistic). Disable for arcade feel without lateral drift |

Tail thrust is computed each tick as `T = ρ · π·R² · V_tip² · CT · pedal · authority`, with `V_tip = TailRPM × 2π/60 × R` and CT baked at 0.008 (universal). Yaw moment is `T · |Position.X|`. When translating tendency is enabled, the same `T` produces the lateral body push, so falling RPM attenuates both yaw control and the sideways push naturally.

### Forward Flight

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bEnableVRS` | true | Vortex Ring State — thrust collapses on fast descent at low airspeed |
| `ETLBoost` | 0.12 | Peak thrust gain at full ETL (0 disables, 0.10–0.15 typical) |
| `VRSStrength` | 0.30 | Peak thrust loss when VRS is fully developed |

### Engine

| Parameter | Default | Description |
|-----------|---------|-------------|
| `EngineMaxPowerHP` | 315 | Max shaft power (HP) |
| `EngineToRotorRatio` | 7.05 | EngineRPM / MainRotorRPM |
| `EngineMomentOfInertia` | 950 | Drivetrain inertia (kg·m²). Higher = more RPM droop under load |
| `EngineSpoolUpTime` | 12 | Seconds from zero to nominal RPM |
| `EngineSpoolDownTime` | 8 | Seconds from nominal to zero RPM |

The governor is a PI controller on RPM error with collective feedforward, capped at full throttle.

### Hull Aerodynamics

| Parameter | Default | Description |
|-----------|---------|-------------|
| `HullDragCdA` | 1.75 | Forward fuselage drag area Cd·A (m²). Light heli ≈ 1.5, Apache ≈ 6 |
| `LandingGearDragCdA` | 0.30 | Gear drag area Cd·A (m²). Set 0 for retractable gear up |
| `LateralDragMultiplier` | 3.0 | Lateral-to-forward drag ratio. Real fuselages produce ~3× more drag sideways. 1.0 = isotropic |
| `SideslipYawGain` | 0.5 | Weather-cock yaw stability — restoring moment that aligns nose with airflow in forward flight. 0 = disabled |

Drag is computed in the body frame as a per-axis signed quadratic: `F_axis = -½·ρ·v_axis·|v_axis|·CdA`, with the lateral component multiplied by `LateralDragMultiplier`. The weather-cock effect adds a yaw torque proportional to forward speed × lateral velocity, pulling the nose back to the direction of motion during pedal-induced skids.

### SAS (Stability Augmentation System)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SAS_RateDampingPitch` | 6 | Pitch rate damping gain |
| `SAS_RateDampingRoll` | 5 | Roll rate damping gain |
| `SAS_RateDampingYaw` | 6 | Yaw rate damping gain |
| `SAS_AttitudeHoldStrength` | 2.5 | Strength of return-to-level when cyclic released |
| `bAutoCollectiveCompensation` | true | Hold altitude through banked turns automatically (arcade feel) |
| `bEnableAutoTrim` | true | Auto-trim tail rotor against reactive torque (no constant pedal needed) |

Attitude hold runs in two layers: a base linear restoring torque that fades out as cyclic input increases (so the pilot retains free authority on the stick), plus a quadratic progressive component that's always active and resists big tilts.

### Velocity Limits

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MaxClimbRate` | 8 | Max climb rate before vertical damping engages (m/s) |

### Envelope Protection

| Parameter | Default | Description |
|-----------|---------|-------------|
| `EnvelopeMaxPitch` | 30 | Maximum allowed pitch (degrees) |
| `EnvelopeMaxRoll` | 45 | Maximum allowed roll (degrees) |

Onset is at 80% of the limit; pushback is cubic in the overshoot fraction so it's gentle near the edge and firm at the limit.

### Input

Player-side input tuning is grouped in a single `InputTuning` struct (type `FInputTuning`) so it stays out of aircraft profile fields.

| Field | Default | Description |
|-------|---------|-------------|
| `InputTuning.CyclicSensitivityPitch` | 1.0 | Forward/back input scale |
| `InputTuning.CyclicSensitivityRoll` | 1.0 | Left/right input scale |
| `InputTuning.PedalSensitivity` | 1.0 | Pedal input scale |
| `InputTuning.CollectiveSensitivity` | 1.0 | Collective input scale |
| `InputTuning.SmoothingPitch` | 0.80 | Pitch attack time constant (s) — slower = more gradual nose lift |
| `InputTuning.SmoothingPitchRelease` | 0.12 | Pitch release time constant (s) — faster = snappier centering |
| `InputTuning.SmoothingOther` | 0.12 | Roll, yaw, collective smoothing (s) |

## Blueprint API

### Engine Control

| Function | Description |
|----------|-------------|
| `StartEngine()` | Begin engine spool-up (smoothstep curve over `EngineSpoolUpTime`) |
| `StopEngine()` | Begin engine spool-down |
| `IsEngineRunning()` | Returns true if the engine is on |

### Control Input

| Function | Description |
|----------|-------------|
| `SetControls(FHelicopterControls)` | Set all four channels at once |
| `SetCyclicLongitudinal(float)` | Pitch: -1 (back) to +1 (forward) |
| `SetCyclicLateral(float)` | Roll: -1 (left) to +1 (right) |
| `SetCollective(float)` | Thrust: 0 (min) to 1 (max) |
| `SetPedals(float)` | Yaw: -1 (left) to +1 (right) |

### Flight State (Getters)

| Function | Returns |
|----------|---------|
| `GetAirspeed()` | Airspeed in knots |
| `GetAltitude()` | Altitude in feet MSL (relative to BeginPlay Z + `FieldElevationFeet`) |
| `GetVerticalSpeed()` | Vertical speed in feet/min |
| `GetRotorRPMPercent()` | Main rotor RPM as % of nominal |
| `GetEngineRPMPercent()` | Engine RPM as % of nominal |
| `GetTailRotorRPMPercent()` | Tail rotor RPM as % of nominal |
| `GetMainRotorRPM()` | Raw main rotor RPM (for blade visuals) |
| `GetTailRotorRPM()` | Raw tail rotor RPM (for blade visuals) |
| `GetMainRotorThrust()` | Main rotor thrust in Newtons |
| `GetPitch()` | Pitch in degrees |
| `GetRoll()` | Roll in degrees |
| `GetHeading()` | Heading 0–360 |
| `GetGroundHeight()` | Height above ground (m) |
| `GetAdvanceRatio()` | Advance ratio μ = V_forward / V_tip |

`Controls` is a public `FHelicopterControls` struct — you can read it directly to query the latest unsmoothed input.

## Example: AH-64 Apache Settings

```
Physics:
  InertiaTensor: (8000, 25000, 20000)

Main Rotor:
  Position: (0, 0, 200)
  Radius: 7.3
  ThrustMultiplier: 1.3
  CyclicMaxTilt: 10
  MainRotorRPMNominal: 289
  bMainRotorClockwise: false   // Apache rotor turns CCW from above

Tail Rotor:
  Position: (-900, 0, 0)
  Radius: 1.4
  TailRotorGearRatio: 5.45
  PedalYawAuthority: 1.0
  bEnableTranslatingTendency: true

Engine:
  EngineMaxPowerHP: 3000
  EngineToRotorRatio: 9.34
  EngineMomentOfInertia: 1500
  EngineSpoolUpTime: 15
  EngineSpoolDownTime: 10

Hull:
  HullDragCdA: 6.0
  LandingGearDragCdA: 0.6
  LateralDragMultiplier: 3.0
  SideslipYawGain: 0.5

SAS:
  RateDampingPitch: 6
  RateDampingRoll: 5
  RateDampingYaw: 7
  AttitudeHoldStrength: 2.5

Envelope:
  MaxPitch: 30
  MaxRoll: 60
```

Set the root mesh mass to ~8000 kg in the physics body settings.

## Architecture

The simulation runs each tick in `TG_PrePhysics` in this order:

1. **FilterControlInputs** — exponential smoothing of raw input (asymmetric on pitch: slow attack, fast release)
2. **TickPowerplant** — smoothstep spool curve + PI governor torque balance
3. **SolveRotorDisc** — thrust scaled by RPM², collective, IGE, altitude, ETL, VRS; cyclic disc tilt; reactive torque
4. **CommitForcesAndTorques** — accumulates all forces/torques and applies them in a single `AddForce` + `AddTorqueInRadians` call

Forces accumulated:
- Main rotor disc thrust (tilted by cyclic, optional auto-collective compensation)
- Cyclic tilt moment
- Vertical/lateral/backward velocity damping
- Hull parasitic drag — body-frame anisotropic, with weather-cock yaw stability
- Reactive torque + tail rotor auto-trim + pilot pedal moment + translating tendency
- SAS rate damping
- Two-layer attitude hold (linear-fading + always-on progressive)
- Envelope protection (cubic pushback)

## Requirements

- Unreal Engine 5.5+
- Root component must be a `UPrimitiveComponent` with physics enabled
