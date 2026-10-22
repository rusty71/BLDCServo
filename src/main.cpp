#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"
#include <CrsfSerial.h>


#define SPI_2_MISO  12
#define SPI_2_MOSI  11
#define SPI_2_SCK   10


#define SENSOR1_CS  13 // some digital pin that you're using as the nCS pin

MagneticSensorSPIConfig_s myAS5048_SPI = {
  .spi_mode = SPI_MODE1,
  .clock_speed = 10000000,
  .bit_resolution = 11,   //Too high causes rumbling? Maybe 10 better
  .angle_register = 0x3FFF,
  .data_start_bit = 13,
  .command_rw_bit = 14,
  .command_parity_bit = 15
};
MagneticSensorSPI sensor1 = MagneticSensorSPI(myAS5048_SPI, SENSOR1_CS);

// MagneticSensorAS5048A sensor1(SENSOR1_CS, true);

MbedSPI spi2(SPI_2_MISO, SPI_2_MOSI, SPI_2_SCK);

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 2, 6, 7);
CrsfSerial crsf(Serial1, 420000);

static float min_angle, max_angle, center_angle;

// Holding stiffness in normal operation (after the calibration sweep, which
// always runs at CAL_VOLTAGE_LIMIT). Both motors run the same position loop,
// so the servo has to actually reach the position the handset commands within
// the frame interval; too soft and the wheel pulls back before it gets there.
// Menu 'v' and 'q' change these live.
#define SERVO_VOLTAGE_LIMIT 12.0f
#define SERVO_P_ANGLE       20.0f


// The servo reports its *actual* position back to the handset through the
// ELRS telemetry, so the handset's steering wheel can follow it.
//
// It travels in the battery sensor frame (0x08), which is a plain big endian
// uint16 - the whole range is used, because the loop is unity gain: the
// handset's target is derived from this number, and any quantisation step
// left in it is integrated by the loop and slowly walks the wheel (at the
// 11 bit channel scale a step was 0.0018 of the travel = 0.006 rad of wheel,
// and the wheel crawled to a stop within seconds). Full scale is
// 0 = min_angle, 65535 = max_angle, 32767/32768 = centre_angle.
static uint16_t positionToRaw()
{
    float span = max_angle - min_angle;
    if (span == 0.0f)
        return 0;                              // no travel measured
    float t = (motor.shaft_angle - min_angle) / span;   // 0..1
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return (uint16_t)(t * 65535.0f);
}

// The token the handset sent in channel 3, echoed back so it can measure the
// round trip latency.
static uint16_t rx_token = 0;
static uint32_t last_channel_rx = 0;

static void sendPosition()
{
    crsf_sensor_battery_t crsfbatt = { 0 };
    // voltage : our own position, full 16 bit over the measured travel
    // current : the token we received, echoed back
    crsfbatt.voltage = htobe16(positionToRaw());
    crsfbatt.current = htobe16(rx_token);
    crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfbatt, sizeof(crsfbatt));
}

// The radio sends link statistics (0x14) on both of its CRSF ports, so
// receiving them proves the radio is talking to us.
static void linkStats(crsfLinkStatistics_t *ls)
{
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  Serial.print("  link: up_lq=");
  Serial.print(ls->uplink_Link_quality);
  Serial.print("%  down_lq=");
  Serial.print(ls->downlink_Link_quality);
  Serial.print("%  up_snr=");
  Serial.print(ls->uplink_SNR);
  Serial.print("  down_snr=");
  Serial.print(ls->downlink_SNR);
  Serial.print("  rf_mode=");
  Serial.println(ls->rf_Mode);
}

void packetChannels()
{
  int steering = crsf.getChannel(2);          // the handset's position, in us
  rx_token = (uint16_t)crsf.getChannelRaw(3); // the latency token, raw 11 bit
  last_channel_rx = millis();

  // (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  motor.target =   (steering - 1000.0f) * (max_angle - min_angle) / 1000.0f + min_angle;

  // Answer straight away so the handset gets the round trip measured with as
  // little turnaround as possible.
  sendPosition();

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 500)
  {
    lastPrint = millis();
    Serial.print("  rx_us=");
    Serial.print(steering);
    Serial.print("  target=");
    Serial.print(motor.target, 4);
    Serial.print("  pos=");
    Serial.print(motor.shaft_angle, 4);
    Serial.print("  token=");
    Serial.println(rx_token);
  }
}



// ---------------------------------------------------------------------------
// Travel calibration
//
// The servo bottoms out on a hard stop at each end of its travel, and the
// travel differs from unit to unit. Ramp the commanded angle slowly towards
// a stop and take the angle at which the rotor stops following the ramp as
// the stop: that is where the linkage bottoms out, whatever the controller
// would still like to command. The voltage limit is reduced for the sweep so
// a stalled motor cannot overheat or damage anything.
// ---------------------------------------------------------------------------
#define CAL_VOLTAGE_LIMIT  4.0f    // V, used only while finding the stops
#define CAL_RAMP_SPEED     2.0f    // rad/s
#define CAL_WINDOW_MS      100     // stall is judged over this window
#define CAL_STALL_MOVE     0.02f   // less than this in one window = not moving
#define CAL_TIMEOUT_MS     8000    // give up after this long

static float sweepToStop(float direction, float start)
{
  float ref = motor.shaft_angle;
  uint32_t refTime = millis();
  uint32_t t0 = millis();
  uint8_t  slow = 0;

  while ((uint32_t)(millis() - t0) < CAL_TIMEOUT_MS)
  {
    float elapsed = (millis() - t0) / 1000.0f;
    motor.target = start + direction * CAL_RAMP_SPEED * elapsed;
    motor.loopFOC();
    motor.move();

    // Judge movement over a fixed window: a slow ramp moves less than one
    // step per iteration, so per-iteration deltas would look like a stall.
    if ((uint32_t)(millis() - refTime) >= CAL_WINDOW_MS)
    {
      float moved = fabsf(motor.shaft_angle - ref);
      ref = motor.shaft_angle;
      refTime = millis();
      if (elapsed > 0.2f)
      {
        if (moved < CAL_STALL_MOVE)
        {
          if (++slow >= 2)
            break;              // two windows without movement: at the stop
        }
        else
          slow = 0;
      }
    }
  }
  return motor.shaft_angle;
}

// ---------------------------------------------------------------------------
// Serial console
//
// The servo is driven from the handset over CRSF, but its holding stiffness
// decides how the pair feels when either end is turned by hand: a stiff servo
// always snaps back to its commanded angle before the handset's lighter wheel
// can follow it. These commands let that be tuned live instead of reflashing.
// ---------------------------------------------------------------------------
static void printServoHelp()
{
  Serial.println();
  Serial.println("servo commands:");
  Serial.println("  s          show travel, target, position and tuning");
  Serial.println("  v <volts>  voltage limit (how hard it holds; 1..12)");
  Serial.println("  q <P>      position gain (how fast it returns; 0.5..50)");
  Serial.println("  h          this help");
  Serial.println();
}

static void printServoStatus()
{
  Serial.print("  travel ");
  Serial.print(min_angle, 4);
  Serial.print(" .. ");
  Serial.print(max_angle, 4);
  Serial.print("  centre ");
  Serial.print(center_angle, 4);
  Serial.print("  (");
  Serial.print((max_angle - min_angle) * 180.0f / PI, 1);
  Serial.println(" deg)");
  Serial.print("  target ");
  Serial.print(motor.target, 4);
  Serial.print("  pos ");
  Serial.print(motor.shaft_angle, 4);
  Serial.print("  err ");
  Serial.print(motor.target - motor.shaft_angle, 4);
  Serial.print("  voltage_limit ");
  Serial.print(motor.voltage_limit, 1);
  Serial.print(" V  P_angle ");
  Serial.println(motor.P_angle.P, 1);
}

static char consoleLine[40];
static uint8_t consoleLen = 0;

static void pollConsole()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n')
    {
      if (consoleLen == 0)
        continue;
      consoleLine[consoleLen] = 0;
      consoleLen = 0;

      if (strcmp(consoleLine, "s") == 0)
        printServoStatus();
      else if (strcmp(consoleLine, "h") == 0 || strcmp(consoleLine, "?") == 0)
        printServoHelp();
      else if (consoleLine[0] == 'v' && consoleLine[1] == ' ')
      {
        // strtod: this newlib build has no floating point scanf
        char *end = NULL;
        double volts = strtod(consoleLine + 2, &end);
        if (end != NULL && end != consoleLine + 2)
        {
          if (volts < 1.0) volts = 1.0;
          if (volts > 12.0) volts = 12.0;
          motor.voltage_limit = (float)volts;
          Serial.print("  voltage limit ");
          Serial.print(motor.voltage_limit, 1);
          Serial.println(" V");
        }
        else
          Serial.println("  usage: v <volts>   (1..12)");
      }
      else if (consoleLine[0] == 'q' && consoleLine[1] == ' ')
      {
        char *end = NULL;
        double p = strtod(consoleLine + 2, &end);
        if (end != NULL && end != consoleLine + 2)
        {
          if (p < 0.5) p = 0.5;
          if (p > 50.0) p = 50.0;
          motor.P_angle.P = (float)p;
          Serial.print("  position gain P_angle ");
          Serial.println(motor.P_angle.P, 1);
        }
        else
          Serial.println("  usage: q <P_angle>   (0.5..50)");
      }
      else
      {
        Serial.print("  unknown command: ");
        Serial.println(consoleLine);
        Serial.println("  h for help");
      }
    }
    else if (consoleLen < sizeof(consoleLine) - 1)
      consoleLine[consoleLen++] = c;
  }
}

void setup() {
  float sens;
  _delay(5000);
  spi2.begin();
  sensor1.init(&spi2);

  crsf.begin();
  crsf.onPacketChannels = &packetChannels;
  crsf.onPacketLinkStatistics = &linkStats;

  motor.linkSensor(&sensor1);

  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 24000;
  driver.init();
  // driver.voltage_limit = 3;

  motor.linkDriver(&driver);
  motor.controller = MotionControlType::torque;
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I =10;
  motor.PID_velocity.D = 0.0;
  motor.voltage_limit = 12;
  motor.LPF_velocity.Tf = 0.01;

  motor.init();
  if (!motor.initFOC())
  {
    // Alignment failed: the electrical zero is unknown, so the motor has to
    // stay disabled - driving it now would use a bogus commutation angle.
    // initFOC() has already disabled the driver.
    Serial.println("FOC alignment FAILED - motor left disabled.");
    Serial.println("check the supply, the driver wiring and the pole pair count");
    _delay(100);
    return;
  }

  Serial.println("calibrate: finding the end stops");
  // Sample in the frame the position controller compares against:
  //   motor.shaftAngle() = sensor_direction * getAngle() - sensor_offset
  // The raw encoder angle is NOT the same frame when the encoder counts the
  // other way round, and using it would mirror the whole travel.
  motor.loopFOC();
  sens = motor.shaftAngle();

  // Sweep with a reduced voltage limit so a stalled motor just holds gently
  // against the stop instead of fighting it at full torque.
  motor.controller = MotionControlType::angle;
  motor.voltage_limit = CAL_VOLTAGE_LIMIT;
  motor.enable();

  float stopA = sweepToStop(-1.0f, sens);
  Serial.print("  stop A : "); Serial.println(stopA, 4);
  float stopB = sweepToStop(+1.0f, motor.shaft_angle);
  Serial.print("  stop B : "); Serial.println(stopB, 4);

  min_angle = fminf(stopA, stopB);
  max_angle = fmaxf(stopA, stopB);
  center_angle = (min_angle + max_angle) / 2.0f;
  float span = max_angle - min_angle;

  Serial.print("  travel : "); Serial.print(span, 4);
  Serial.print(" rad ("); Serial.print(span * 180.0f / PI); Serial.println(" deg)");
  Serial.print("  min    : "); Serial.println(min_angle, 4);
  Serial.print("  max    : "); Serial.println(max_angle, 4);
  Serial.print("  center : "); Serial.println(center_angle, 4);

  if (span < 0.20f)
  {
    // A tiny span means the sweep never left the first stop; driving with it
    // would divide by almost nothing and slam the servo between the stops.
    Serial.println("  TRAVEL TOO SMALL - check the linkage, servo left disabled");
    motor.disable();
    _delay(100);
    return;
  }

  motor.voltage_limit = SERVO_VOLTAGE_LIMIT;
  motor.P_angle.P = SERVO_P_ANGLE;
  motor.target = center_angle;
  motor.enable();
  motor.loopFOC();
  motor.move();
  Serial.print("  at center  (voltage_limit ");
  Serial.print(motor.voltage_limit, 1);
  Serial.print(" V, P_angle ");
  Serial.print(motor.P_angle.P, 1);
  Serial.println(")");
  printServoHelp();
   _delay(100);
}

void loop() {
  static int loop_timer = millis();
  static uint32_t position_timer = 0;
  crsf.loop();
  pollConsole();
  motor.loopFOC();
  motor.move();
  static uint32_t stat_timer = 0;
  if ((uint32_t)(millis() - stat_timer) >= 2000)
  {
    stat_timer = millis();
    Serial.print("  crsf: frames=");
    Serial.print(crsf.framesReceived());
    Serial.print(" bytes=");
    Serial.print(crsf.bytesReceived());
    Serial.println();
  }
  // Fallback: keep the link alive with the last known position if the
  // handset stops sending. While channel frames are arriving we answer them
  // directly, so this stays quiet and does not pollute the timing.
  if ((uint32_t)(millis() - last_channel_rx) >= 100 &&
      (uint32_t)(millis() - position_timer) >= 20) {
    position_timer = millis();
    sendPosition();
  }
  if((millis() - loop_timer)>1000) {
    Serial.println(sensor1.getAngle());
    loop_timer = millis();
  }
    // Serial.println(sensor1.getAngle());
}

