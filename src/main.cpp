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


static void checkVbatt(uint16_t bat)
{
    crsf_sensor_battery_t crsfbatt = { 0 };
    uint16_t scaledVoltage = bat;
    // Values are MSB first (BigEndian)
    // crsfbatt.voltage = htobe32(scaledVoltage);
    crsfbatt.voltage = htobe16(scaledVoltage);
    crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfbatt, sizeof(crsfbatt));

}

void packetChannels()
{
  int steering = crsf.getChannel(2);  //CHECKME: expect 1000-2000
  // Serial.print(steering);

  // (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  motor.target =   (steering - 1000.0f) * (max_angle - min_angle) / 1000.0f + min_angle;
  // sensor1.getRawCount();
  // Serial.print('\t');
  // Serial.println(motor.target);
  // int target;
  checkVbatt(steering-989);
}



void setup() {
  float sens;
  int now;
  _delay(5000);
  spi2.begin();
  sensor1.init(&spi2);

  crsf.begin();
  crsf.onPacketChannels = &packetChannels;

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
  motor.initFOC();
  // motor.sensor_offset = sensor1.getAngle();

  Serial.println("calibrate");
  sens = sensor1.getAngle();

  max_angle = sens;
  min_angle = sens;
if(1){
  //force max angle
  now = millis();
  while((millis()-now) < 1000) {
      motor.target = -3;
      motor.loopFOC();
      motor.move();
  }
  //relax and measure angle
  motor.disable();
  now = millis();
  while((millis()-now) < 1000) {
    motor.loopFOC();
    motor.move();
  }
  min_angle = sensor1.getAngle();
  
  motor.enable();
//force max angle
  now = millis();
  while((millis()-now) < 1000) {
      motor.target = 3;
      motor.loopFOC();
      motor.move();
  }
  //relay and measure angle
  motor.disable();
  now = millis();
  while((millis()-now) < 1000) {
    motor.loopFOC();
    motor.move();
  }
  max_angle = sensor1.getAngle();

  now = millis();
  while((millis()-now) < 1000) {
   motor.loopFOC();
    motor.move();
  } 
 
  motor.controller = MotionControlType::angle;

  Serial.print("max angle : ");
  Serial.println(max_angle);
  Serial.print("min angle : ");
  Serial.println(min_angle);
  center_angle = (min_angle+((max_angle-min_angle)/2));
  motor.target = center_angle; 
  motor.enable();
  motor.loopFOC();
  motor.move();
  Serial.print("center : ");
  Serial.println(motor.target);
}
   _delay(100);
}

void loop() {
  static int loop_timer = millis();
  crsf.loop();
  motor.loopFOC();
  motor.move();
  if((millis() - loop_timer)>1000) {
    Serial.println(sensor1.getAngle());
    loop_timer = millis();
  }
    // Serial.println(sensor1.getAngle());
}

