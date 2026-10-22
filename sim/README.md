# Position lock simulation

A host-side simulation of the SimpleFOC angle-mode position lock used by this
project, so the control chain can be exercised without hardware.

It reproduces, faithfully:

- `Sensor::update()` / `getAngle()` / `getVelocity()`  — `common/base_classes/Sensor.cpp`
- `FOCMotor::shaftAngle()` / `shaftVelocity()`         — `common/base_classes/FOCMotor.cpp`
- `BLDCMotor::move()` (angle mode) / `loopFOC()`       — `BLDCMotor.cpp`
- `PIDController::operator()`                          — `common/pid.cpp`
- `LowPassFilter`                                       — `common/lowpass_filter.cpp`

The plant is a BLDC with an absolute encoder, viscous friction and mechanical
travel limits. The two encoder timestamp conventions differ: SimpleFOC uses
`_micros()` for the sensor and `millis()` for the PID, the simulation keeps
them apart.

## Build and run

```bash
g++ -O2 -o servo_sim servo_sim.cpp
./servo_sim
```

## What it shows

The calibration has to sample `motor.shaftAngle()`, not the raw encoder
angle, because

```
shaftAngle() = sensor_direction * getAngle() - sensor_offset
```

and `sensor_direction` is `+1` or `-1` depending on the motor's phase order
and the encoder mounting. Using the raw angle mirrors the whole travel when
the encoder counts the other way: the servo still locks, but at the opposite
end of its range.

| case | calibration frame | `sensor_direction` | result |
| ---- | ----------------- | ------------------ | ------ |
| A | raw | `+1` | locked, 0.4 deg residual |
| B | raw | `-1` | locked, but at the **mirrored** position |
| C | `shaftAngle()` | `+1` | locked, 0.4 deg residual |
| D | `shaftAngle()` | `-1` | locked at the **correct** position |
