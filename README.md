# BLDCServo

A CRSF servo for ELRS radios. It reads **RC channel 2 from an ELRS radio over
CRSF** and drives a brushless servo to the commanded angle. The mechanical
range is **auto-calibrated at boot**, then the channel is mapped onto it.

Runs on a **Waveshare RP2040 Zero** (Raspberry Pi Pico) with the Arduino/mbed
framework, using [SimpleFOC](https://simplefoc.dev/) and a local fork of the
CRSF protocol library.

---

## How it works

```
   ELRS radio / receiver ── CRSF 420 kbaud ──►  BLDCServo (RP2040)
              │  addr 0xC8                            │  addr 0xC8
              │  0x16 RC channels                     │  0x08 battery telemetry
              └───────────────────────────────────────┘
```

The servo behaves as a CRSF **flight-controller-side** device: it consumes
`0x16` (RC channels) frames and answers with `0x08` (battery sensor)
telemetry.

### Boot calibration

The servo discovers its own mechanical range on every power-up. The travel
differs from unit to unit (103.9 deg on the bench unit) and there is a hard
stop at each end, so the sketch finds them by stalling the motor against them:

1. Switch to `MotionControlType::angle` and lower `voltage_limit` to
   `CAL_VOLTAGE_LIMIT` (4 V), so a motor that is held against a stop pushes
   gently instead of fighting it at full torque.
2. Ramp the target at `CAL_RAMP_SPEED` (2 rad/s) towards one stop.
3. Judge movement over a fixed 100 ms window: two consecutive windows with
   less than `CAL_STALL_MOVE` (0.02 rad) of movement mean the rotor has
   stopped following the ramp, so that is the stop. A per-iteration delta
   would not work - a 2 rad/s ramp moves only about 1 mrad per control
   iteration.
4. Record that angle (`motor.shaftAngle()`) and ramp the other way for the
   second stop.
5. `min_angle` / `max_angle` are the two stops, `center_angle` their midpoint.
   If the span comes out under 0.2 rad the linkage is jammed or disconnected;
   the motor is then left disabled instead of mapping the channel onto
   almost no travel.
6. Restore the 12 V limit, stay in angle mode and move to `center_angle`.

`CAL_TIMEOUT_MS` (8 s) bounds each sweep. On the bench the two sweeps take
about two seconds, on top of the 5 s power-up delay, the 1 s `motor.init()`
and the ~3 s `initFOC()` alignment. The result is printed on the debug port:

```
calibrate: finding the end stops
  stop A : 1.2180
  stop B : 3.0311
  travel : 1.8132 rad (103.89 deg)
  min    : 1.2180
  max    : 3.0311
  center : 2.1246
  at center
```

Channel 2 is then mapped linearly onto that travel:

```
motor.target = (steering_us - 1000.0f) * (max_angle - min_angle) / 1000.0f + min_angle;
```

so 1000 us is the min stop, 1500 us the centre and 2000 us the max stop. The
handset sends its own boot position as 1500 us, so the servo starts centred,
and one full turn of the handset wheel spans this whole calibrated travel.

### Measured behaviour

From `test.csv` (745.8 s of traffic):

| metric                            | value                    |
| --------------------------------- | ------------------------ |
| RX `0x16` channel frames          | 184 688 @ 4002 µs median |
| TX `0x08` battery frames          | 184 688 @ 4001 µs median |
| Response latency (RX → TX)        | ≈ 1.3 ms                 |
| Channel value range               | 172…1811 (raw CRSF)      |

Clean 1:1 request/response — one telemetry frame per received channel frame.

---

## Hardware

| Function                | RP2040 pin | Notes                              |
| ----------------------- | ---------- | ---------------------------------- |
| CRSF UART (Serial1)     | GP4 TX / GP5 RX | 420 000 baud, shared CRSF bus |
| Encoder SPI (MISO/MOSI/SCK) | 12 / 11 / 10 | AS5048A, SPI_MODE1 @ 10 MHz   |
| Encoder CS              | 13         |                                    |
| Motor driver 3-PWM      | 3, 2, 6, 7 |                                   |
| Debug serial (USB)      | —          | 115 200 baud                       |

Motor: 11 pole pairs. Supply: 12 V. Driver PWM: 24 kHz.

---

## Build

```bash
pio run                # build
pio run -t upload      # flash
pio device monitor     # 115200 baud debug output
```

Set `upload_port` / `monitor_port` in `platformio.ini` if you want to pin the
port to a specific board (the file ships with them commented out).

---

## Repository layout

```
src/main.cpp                    active sketch
variants/                       version snapshots of earlier sketches
lib/CrsfSerial/                 CRSF protocol library (local fork)
lib/crc8/                       CRC-8 (poly 0xD5) used by CRSF
serplot.py                      captures the debug serial stream to test.csv
test.csv                        captured CRSF trace
```

Column format of `test.csv`: `timestamp_us, direction, addr, type, length[, value]`,
where direction `<` = received and `>` = transmitted.

---

## The position report

The servo reports its **actual** position back to the handset through the
ELRS telemetry, so the handset's steering wheel can follow it. It travels in
the battery sensor frame (`0x08`), in the raw CRSF channel scale:

| raw value | servo position |
| --------- | -------------- |
| `CRSF_CHANNEL_VALUE_MIN` (172) | `min_angle` |
| `CRSF_CHANNEL_VALUE_MAX` (1811) | `max_angle` |
| the midpoint (991) | `center_angle` |

```cpp
float t = (motor.shaft_angle - min_angle) / (max_angle - min_angle);
raw = CRSF_CHANNEL_VALUE_MIN + t * (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN);
```

The frame's `current` field carries back the **latency token** the handset
sent in channel 2, so the handset can measure the round trip (see the
BLDCHandset README). The token is read with `getChannelRaw()`, i.e. without
the microseconds mapping, which would otherwise mangle it.

The frame is sent immediately on every received channel frame. The 20 ms
timer from the main loop is only a fallback for when channel frames stop
arriving for 100 ms, so it does not add extra telemetry traffic (and extra
delay) while the link is healthy.

The battery frame's values are big endian on the wire (per the CRSF spec),
so the handset swaps them back with `be16toh()` when it decodes them.

The field is being used as a position carrier rather than a voltage, so the
radio will show an odd battery reading - that is the price of the CRSF
telemetry path.

## The position lock

The servo runs SimpleFOC's angle mode, so the chain is:

```
motor.target  ->  P_angle(target - motor.shaft_angle)  ->  velocity setpoint
               ->  PID_velocity(setpoint - motor.shaft_velocity)  ->  voltage
               ->  loopFOC(): commutation against motor.electricalAngle()
```

`motor.shaft_angle` is **not** the raw encoder angle:

```cpp
shaftAngle() = sensor_direction * LPF_angle(sensor->getAngle()) - sensor_offset;
```

`sensor_direction` is `+1` or `-1` and is decided by SimpleFOC's
`alignSensor()` from the way the rotor moves during alignment - it depends on
the motor's phase order and how the encoder is mounted. The *raw* encoder
angle and the *controller's* angle are therefore only the same frame when
`sensor_direction == +1`.

**The calibration must sample `motor.shaftAngle()`, not
`sensor1.getAngle()`.** Sampling the raw angle mirrors the whole travel when
the encoder counts the other way: the servo still reaches its commanded
position, but at the opposite end of the range.

A host-side simulation of the exact SimpleFOC control law
(`Sensor::update/getAngle/getVelocity`, `FOCMotor::shaftAngle/shaftVelocity`,
`BLDCMotor::move`, `PIDController::operator`) shows the difference:

| case | calibration frame | `sensor_direction` | result |
| ---- | ----------------- | ------------------ | ------ |
| A | raw (as written) | `+1` | locked, 0.4 deg residual |
| B | raw (as written) | `-1` | locked, but at the **mirrored** position |
| C | `motor.shaftAngle()` | `+1` | locked, 0.4 deg residual |
| D | `motor.shaftAngle()` | `-1` | locked at the **correct** position |

The sketch now uses `motor.shaftAngle()` for the calibration.

Note that the encoder's zero position is *not* a problem: `Sensor::getAngle()`
returns the **unwrapped** angle (`full_rotations * 2pi + angle_prev`), so
`min`/`max` over a continuous sweep are correct even when the range
straddles the encoder's zero.

## Known issues

Verified from the code and the captured trace.

1. **The battery telemetry is hijacked.** The `0x08` frame's voltage field
   carries the **position** instead of a voltage, and `capacity` / `remaining`
   are zero, so the radio's battery readout is nonsense. It also fires a
   frame on **every** received channel frame (248 Hz), which contends for the
   shared CRSF bus - consider sending it less often.

2. **The encoder is 11-bit.** The source comments note that 11 bits "causes
   rumbling" and that 10 may be better.

3. **The CSV-instrumented build is not behaviourally identical to
   production.** In `lib/CrsfSerial/CrsfSerial.cpp` the channel→µs scaling is
   wrapped in `#ifndef CRSF_PERFORMANCE_CSV`, so with logging enabled
   `getChannel()` returns **raw** values instead of microseconds. Any
   conclusion drawn from a capture must account for this.

4. **Startup takes ~13 s before the servo is usable.** `_delay(5000)`, then
   `motor.init()` (2 x 500 ms), then `initFOC()` (~3 s of alignment sweeps),
   then the two stall-detected travel sweeps (~2 s). The link carries on in
   the meantime, but the channel frames that arrive during setup are only
   processed once `loop()` starts.

---

## Capturing a new trace

1. Remove the `#undef CRSF_PERFORMANCE_CSV` line at the top of
   `lib/CrsfSerial/CrsfSerial.cpp` (it disables the CSV logger).
2. Flash and run `serplot.py` to capture the debug serial stream to `test.csv`.
3. Restore the `#undef`.

## License

Public domain / unlicensed — add your preferred license.
