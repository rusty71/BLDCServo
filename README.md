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

On power-up the servo discovers its own mechanical range:

1. Drive the motor to `target = -3` for 1 s. Because the torque controller is
   in *voltage* mode and no phase resistance is configured, SimpleFOC maps
   this to `voltage.q = -3 V`.
2. Disable the motor and coast for 1 s, then sample the encoder →
   `min_angle`.
3. Repeat with `target = +3` → `max_angle`.
4. `center_angle = min_angle + (max_angle - min_angle) / 2`.
5. Switch to `MotionControlType::angle` and map channel 2 linearly onto
   `[min_angle, max_angle]`.

```
motor.target = (steering_us - 1000.0f) * (max_angle - min_angle) / 1000.0f + min_angle;
```

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

## Known issues

Verified from the code and the captured trace. **Not** fixed here because each
needs hardware validation.

1. **The battery telemetry is hijacked.** `checkVbatt(steering - 989)` reports
   the *steering position* as a battery voltage (1.1 V … 101.1 V) and leaves
   `capacity` and `remaining` at zero, so the radio's battery readout shows
   0 %. It also fires a response on **every** received channel frame (248 Hz),
   which contends for the shared CRSF bus.

2. **The calibration has no wrap-around handling.** `min_angle` / `max_angle`
   are raw encoder angles (0…2π). If the mechanical range straddles 0/2π then
   `min_angle > max_angle` and the channel→angle mapping inverts.

3. **The calibration may not reach the mechanical stops.** ±3 V for 1 s is a
   modest torque; if the servo does not hit its limits the calibrated range
   will be wrong. The range is also measured with the motor *coasting*, so
   back-drive and load-dependent deflection are included.

4. **The encoder is 11-bit.** The source comments note that 11 bits "causes
   rumbling" and that 10 may be better.

5. **The CSV-instrumented build is not behaviourally identical to
   production.** In `lib/CrsfSerial/CrsfSerial.cpp` the channel→µs scaling is
   wrapped in `#ifndef CRSF_PERFORMANCE_CSV`, so with logging enabled
   `getChannel()` returns **raw** values instead of microseconds. Any
   conclusion drawn from a capture must account for this.

---

## Capturing a new trace

1. Remove the `#undef CRSF_PERFORMANCE_CSV` line at the top of
   `lib/CrsfSerial/CrsfSerial.cpp` (it disables the CSV logger).
2. Flash and run `serplot.py` to capture the debug serial stream to `test.csv`.
3. Restore the `#undef`.

## License

Public domain / unlicensed — add your preferred license.
