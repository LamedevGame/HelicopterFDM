# Helicopter FDM Plugin

Game-oriented helicopter flight dynamics model for Unreal Engine 5. Provides realistic-feeling helicopter physics through a single Blueprint component with intuitive parameters.

<img width="2554" height="1436" alt="image" src="https://github.com/user-attachments/assets/26ea3f43-573b-47c7-90cf-043b49b1e4c8" />

<img width="2559" height="1439" alt="image" src="https://github.com/user-attachments/assets/5fb79e1f-125d-4b67-b450-43cee48ce150" />

<img width="744" height="363" alt="image" src="https://github.com/user-attachments/assets/1cad6b05-ab49-43f5-b951-514e061ddbb9" />

https://www.youtube.com/watch?v=0exTaOAsJRo

Test Build Link: https://drive.google.com/file/d/1NSQLvvOrSDtkSRo9yvpTUv1pziJNpIC-/view?usp=sharing

## Tech Stack

  | | |
  |---|---|
  | **Engine** | Unreal Engine 5.5+ (C++) |
  | **Physics** | UE Physics (`AddForce`, `AddTorqueInRadians`), custom FDM |
  | **Systems** | Engine governor, SAS, envelope protection, ground effect |
  | **Architecture** | Single Blueprint component, data-driven configuration |
  | **Exposure** | Full Blueprint API (`BlueprintCallable`, `BlueprintReadOnly`) |
  
## Features

- Rotor thrust with ground effect (Cheeseman-Bennett model)
- Engine simulation with PID governor and spool up/down curves
- Cyclic, collective, and pedal controls with sensitivity and smoothing
- Stability Augmentation System (SAS) with rate damping and attitude hold
- Envelope protection (pitch/roll limits)
- Velocity limits (climb, descent, lateral, backward)
- Hull parasitic drag
- Lightweight per-tick: single `AddForce` + single `AddTorqueInRadians` call

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
  → SetCollective(value)        // 0-1, accumulated from input
  → SetCyclicLongitudinal(value) // -1 to 1
  → SetCyclicLateral(value)      // -1 to 1
  → SetPedals(value)             // -1 to 1
```

### Visual Blade Rotation

In your Blueprint Tick, rotate the main rotor mesh:

```
RotorYaw += GetMainRotorRPM() * 6.0 * DeltaTime
```

For the tail rotor:

```
TailRotation += GetTailRotorRPM() * 6.0 * DeltaTime
```

`RPM * 6.0` converts RPM to degrees/sec (360 / 60).

## Configuration

All parameters are exposed as Blueprint properties organized by category.

### General

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bEnabled` | false | Enable/disable FDM simulation |
| `FieldElevationFeet` | 0 | Airport elevation for altimeter zero reference |

### Physics

| Parameter | Default | Description |
|-----------|---------|-------------|
| `InertiaTensor` | (2200, 7500, 6800) | Rotational inertia Ixx, Iyy, Izz (kg*m^2) |

Mass is read automatically from the root physics component.

### Main Rotor

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MainRotor.Position` | (0,0,0) | Hub position in cm. Z = height above CoG (affects cyclic moment) |
| `MainRotor.Radius` | 7.3 | Blade radius in meters (affects ground effect) |
| `ThrustMultiplier` | 1.25 | Thrust at full collective as multiplier of weight. 1.0 = hover |
| `TorqueFraction` | 0.07 | Fraction of thrust that becomes reactive torque |
| `ThrustResponseTime` | 0.05 | Thrust smoothing in seconds. 0 = instant |
| `CyclicMaxTilt` | 10.0 | Max rotor disc tilt from cyclic input (degrees) |
| `MainRotorRPMNominal` | 394 | Governor target RPM |
| `MainRotorDirection` | -1.0 | +1 = CCW from above (US standard), -1 = CW (Eurocopter convention) |

### Tail Rotor

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TailRotor.Position` | (0,0,0) | Position in cm. X = moment arm for yaw |
| `TailRotor.Radius` | 7.3 | Blade radius in meters |
| `TailRotorGearRatio` | 5.32 | TailRPM = MainRPM * ratio |
| `PedalYawAuthority` | 1.2 | Pedal yaw force multiplier |

### Engine

| Parameter | Default | Description |
|-----------|---------|-------------|
| `EngineIdleRPM` | 550 | Idle RPM |
| `EngineMaxRPM` | 2500 | Maximum RPM |
| `EngineMaxPowerHP` | 315 | Max power in horsepower |
| `EngineToRotorRatio` | 7.05 | Engine RPM / Rotor RPM |
| `EngineMomentOfInertia` | 950 | Rotational inertia of drivetrain (kg*m^2). Higher = more RPM droop under load |
| `GovernorBaseThrottle` | 0.35 | Base throttle at nominal RPM with zero collective |
| `EngineSpoolUpTime` | 12 | Seconds from zero to nominal RPM |
| `EngineSpoolDownTime` | 8 | Seconds from nominal to zero RPM |

### Hull Aerodynamics

| Parameter | Default | Description |
|-----------|---------|-------------|
| `HullReferenceArea` | 5.0 | Fuselage reference area (m^2) |
| `HullDragCoefficient` | 0.35 | Fuselage drag coefficient |
| `LandingGearDragCoefficient` | 0.25 | Landing gear drag coefficient |
| `LandingGearReferenceArea` | 1.2 | Landing gear reference area (m^2) |

### SAS (Stability Augmentation System)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SAS_RateDampingPitch` | 6 | Pitch rotation damping. Higher = more stable |
| `SAS_RateDampingRoll` | 5 | Roll rotation damping |
| `SAS_RateDampingYaw` | 6 | Yaw rotation damping |
| `SAS_AttitudeHoldStrength` | 2.5 | Return-to-level strength when cyclic released |
| `SAS_AttitudeHoldDeadzone` | 0.1 | Cyclic deadzone for attitude hold activation |
| `SAS_AttitudeHoldProgressive` | 1.5 | Stronger correction at larger angles |
| `MinCyclicAuthorityFraction` | 0.15 | Minimum cyclic control at low thrust |
| `MaxCyclicAcceleration` | 3.0 | Max cyclic angular acceleration (rad/s^2) |

### Velocity Limits

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MaxClimbRate` | 8 | Max climb rate before damping (m/s) |
| `MaxDescentRate` | 12 | Max descent rate before damping (m/s) |
| `VerticalDampingGain` | 2.0 | Vertical damping strength |
| `LateralDampingGain` | 2.0 | Lateral drift damping strength |
| `BackwardDampingGain` | 1.5 | Backward flight damping strength |

### Envelope Protection

| Parameter | Default | Description |
|-----------|---------|-------------|
| `EnvelopeMaxPitch` | 30 | Maximum pitch angle (degrees) |
| `EnvelopeMaxRoll` | 45 | Maximum roll angle (degrees) |
| `EnvelopeStrength` | 8.0 | Pushback force at envelope limits |

### Input

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CyclicSensitivityPitch` | 1.0 | Forward/backward input scale |
| `CyclicSensitivityRoll` | 1.0 | Left/right input scale |
| `PedalSensitivity` | 1.0 | Pedal input scale |
| `CollectiveSensitivity` | 1.0 | Collective input scale |
| `InputSmoothingCyclic` | 0.12 | Cyclic smoothing time constant (seconds) |
| `InputSmoothingPedals` | 0.2 | Pedal smoothing time constant |
| `InputSmoothingCollective` | 0.05 | Collective smoothing time constant |

### Optimization

| Parameter | Default | Description |
|-----------|---------|-------------|
| `GroundTraceInterval` | 0.2 | Ground height check frequency (seconds) |

## Blueprint API

### Engine Control

| Function | Description |
|----------|-------------|
| `StartEngine()` | Begin engine spool-up |
| `StopEngine()` | Begin engine spool-down |
| `IsEngineRunning()` | Returns true if engine is on |

### Control Input

| Function | Description |
|----------|-------------|
| `SetControls(FHelicopterControls)` | Set all controls at once |
| `SetCyclicLongitudinal(float)` | Pitch: -1 (back) to 1 (forward) |
| `SetCyclicLateral(float)` | Roll: -1 (left) to 1 (right) |
| `SetCollective(float)` | Thrust: 0 (min) to 1 (max) |
| `SetPedals(float)` | Yaw: -1 (left) to 1 (right) |

### Flight State (Getters)

| Function | Returns |
|----------|---------|
| `GetAirspeed()` | Airspeed in knots |
| `GetAltitude()` | Altitude in feet MSL (0 until engine started) |
| `GetVerticalSpeed()` | Vertical speed in feet/min |
| `GetRotorRPMPercent()` | Main rotor RPM as % of nominal |
| `GetEngineRPMPercent()` | Engine RPM as % of max |
| `GetTailRotorRPMPercent()` | Tail rotor RPM as % of nominal |
| `GetMainRotorRPM()` | Raw main rotor RPM (for blade rotation visuals) |
| `GetTailRotorRPM()` | Raw tail rotor RPM (for blade rotation visuals) |
| `GetMainRotorThrust()` | Thrust in Newtons |
| `GetPitch()` | Pitch angle in degrees |
| `GetRoll()` | Roll angle in degrees |
| `GetHeading()` | Heading 0-360 degrees |
| `GetGroundHeight()` | Height above ground in meters |

`Controls` struct is also public — read `Controls.Throttle` directly for current collective position.

## Example: AH-64 Apache Settings

```
Physics:
  InertiaTensor: (8000, 25000, 20000)

Main Rotor:
  Position: (0, 0, 200)
  Radius: 7.3
  ThrustMultiplier: 1.3
  TorqueFraction: 0.06
  ThrustResponseTime: 0.08
  CyclicMaxTilt: 10
  MainRotorRPMNominal: 289

Tail Rotor:
  Position: (-900, 0, 250)
  Radius: 1.4
  TailRotorGearRatio: 5.45
  PedalYawAuthority: 1.5

Engine:
  EngineIdleRPM: 600
  EngineMaxRPM: 2700
  EngineMaxPowerHP: 3000
  EngineToRotorRatio: 9.34
  EngineMomentOfInertia: 1500
  GovernorBaseThrottle: 0.4
  EngineSpoolUpTime: 15
  EngineSpoolDownTime: 10

Hull:
  HullReferenceArea: 12.0
  HullDragCoefficient: 0.5
  LandingGearDragCoefficient: 0.2
  LandingGearReferenceArea: 3.0

SAS:
  RateDampingPitch: 6
  RateDampingRoll: 5
  RateDampingYaw: 7
  AttitudeHoldStrength: 2.5
  MaxCyclicAcceleration: 4.0

Envelope:
  MaxPitch: 30
  MaxRoll: 60
  Strength: 8.0
```

Set the root mesh mass to ~8000 kg in the physics body settings.

## Architecture

The simulation runs each tick in this order:

1. **FilterControlInputs** — smooths raw input with exponential filtering
2. **TickPowerplant** — smoothstep spool curve + PID governor torque balance
3. **SolveRotorDisc** — thrust, torque, and cyclic tilt from RPM and collective
4. **CommitForcesAndTorques** — accumulates all forces/torques and applies in a single `AddForce` + `AddTorqueInRadians` call

Forces applied:
- Rotor disc thrust (tilted by cyclic)
- Cyclic tilt moment
- Vertical/lateral/backward velocity damping
- Hull parasitic drag
- Pedal yaw moment
- SAS rate damping
- SAS attitude hold + envelope protection (cubic pushback)

## Requirements

- Unreal Engine 5.5+
- Root component must be a `UPrimitiveComponent` with physics enabled
