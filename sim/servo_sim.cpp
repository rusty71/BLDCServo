// Host-side simulation of the BLDCServo position lock.
//
// Question: does the position lock actually work?
//
// The chain under test (BLDCServo/src/main.cpp):
//
//   setup()  calibration:
//       sens     = sensor1.getAngle();          <- RAW encoder angle
//       min_angle / max_angle                   <- RAW encoder angle
//       center_angle = min + (max-min)/2        <- RAW encoder angle
//
//   loop()   control:
//       motor.target = (steering-1000)*(max-min)/1000 + min;   <- RAW frame
//
//   SimpleFOC angle mode:
//       shaft_angle = shaftAngle()
//                  = sensor_direction * getAngle() - sensor_offset;
//       error      = target - shaft_angle;        <- SENSOR_DIRECTION frame
//
// The calibration and the target are in the RAW encoder frame, the feedback
// is in the SENSOR_DIRECTION frame. Those only agree when sensor_direction
// is +1.
//
// Build:  g++ -O2 -o servo_sim servo_sim.cpp && ./servo_sim

#include <cstdio>
#include <cmath>
#include <cstdint>
#include <string>

static const float PI     = 3.14159265358979323846f;
static const float TWO_PI = 2.0f * PI;
static const float NOT_SET = -12345.0f;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float normAngle(float a) { a = fmodf(a, TWO_PI); return a >= 0 ? a : a + TWO_PI; }

// ---------------------------------------------------------------------------
// SimpleFOC PIDController (common/pid.cpp)
// ---------------------------------------------------------------------------
struct PID {
    float P = 0, I = 0, D = 0, limit = 0;
    float integral_prev = 0, error_prev = 0;
    double ts_prev = 0;
    float operator()(float error, double ts) {
        float Ts = (float)((ts - ts_prev) * 1e-6);
        if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
        float proportional = P * error;
        float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);
        integral = clampf(integral, -limit, limit);
        float derivative = D * (error - error_prev) / Ts;
        float output = clampf(proportional + integral + derivative, -limit, limit);
        integral_prev = integral; error_prev = error; ts_prev = ts;
        return output;
    }
};

// ---------------------------------------------------------------------------
// SimpleFOC absolute encoder (common/base_classes/Sensor.cpp)
// ---------------------------------------------------------------------------
struct Encoder {
    float angle_prev = 0;                 // raw [0, 2pi)
    int   full_rotations = 0;
    float vel_angle_prev = 0;
    int   vel_full_rotations = 0;
    float velocity = 0;
    double angle_ts = 0, vel_ts = 0;   // microseconds, like _micros()

    void update(float raw, double ts_us) {
        angle_ts = ts_us;
        float d = raw - angle_prev;
        if (fabsf(d) > 0.8f * TWO_PI) full_rotations += (d > 0) ? -1 : 1;
        angle_prev = raw;
    }
    float getAngle()             { return (float)full_rotations * TWO_PI + angle_prev; }
    float getMechanicalAngle()   { return angle_prev; }
    float getVelocity() {
        float Ts = (float)((angle_ts - vel_ts) * 1e-6);
        if (Ts < 0.0f) { vel_angle_prev = angle_prev; vel_full_rotations = full_rotations; vel_ts = angle_ts; return velocity; }
        if (Ts < 5e-3f) return velocity;              // SimpleFOC min_elapsed_time
        velocity = ((float)(full_rotations - vel_full_rotations) * TWO_PI + (angle_prev - vel_angle_prev)) / Ts;
        vel_angle_prev = angle_prev; vel_full_rotations = full_rotations; vel_ts = angle_ts;
        return velocity;
    }
};

// ---------------------------------------------------------------------------
// Plant + controller
//
// psi is the encoder reading (the mechanical frame the calibration uses).
// The plant is a well behaved BLDC driven by the controller's voltage: after
// a correct FOC alignment the commutation is right, so the torque acts
// directly on psi.
// ---------------------------------------------------------------------------
struct Sim {
    // --- configuration taken from the project files ---
    int   pole_pairs    = 11;          // BLDCMotor motor = BLDCMotor(11);
    float voltage_limit = 12.0f;       // motor.voltage_limit = 12
    float velocity_limit= 20.0f;       // DEF_VEL_LIM
    float PIDv_P = 0.2f, PIDv_I = 10.0f;   // motor.PID_velocity.P / .I

    // --- plant ---
    float J = 5.0e-4f, B = 2.0e-2f, Kt = 2.0e-2f;
    float psi_min = -0.60f, psi_max = 0.60f;   // mechanical travel limits
    float psi = 0.0f;      // encoder reading
    float omega = 0.0f;
    float load = 0.0f;
    float dt = 1.0f / 20000.0f;
    double clock = 0;

    // --- sensor / SimpleFOC state ---
    Encoder enc;
    float encoder_zero = 0.0f;     // where the encoder zero sits mechanically
    int   sensor_direction = +1;   // set by alignSensor()
    float sensor_offset = 0.0f;    // never set by the sketch -> 0
    float zero_electric_angle = NOT_SET;

    PID P_angle{20.0f, 0, 0, 20.0f};        // DEF_P_ANGLE_P, limit = velocity_limit
    PID PID_velocity{0.2f, 10.0f, 0.0f, 12.0f};

    float shaft_angle = 0, shaft_velocity = 0, voltage_q = 0;
    // LowPassFilter with Tf = 0.01 s (motor.LPF_velocity.Tf = 0.01)
    float lpf_vel_y = 0, lpf_vel_prev = 0;
    double lpf_vel_ts = 0;
    bool  lpf_vel_init = false;
    float target = 0;

    // SimpleFOC FOCMotor::shaftAngle() / shaftVelocity()
    float shaftAngle()    { return sensor_direction * enc.getAngle() - sensor_offset; }
    float shaftVelocity() {
        float v = enc.getVelocity();
        // SimpleFOC LowPassFilter, Tf = 0.01 s
        float Tf = 0.01f;
        double ts = clock;
        if (!lpf_vel_init) { lpf_vel_init = true; lpf_vel_y = v; lpf_vel_prev = v; lpf_vel_ts = ts; }
        float dt = (float)((ts - lpf_vel_ts) * 1e-6);
        if (dt < 0.0f) dt = 1e-3f;
        else if (dt > 0.3f) { lpf_vel_y = v; lpf_vel_prev = v; lpf_vel_ts = ts; }
        else {
            float alpha = Tf / (Tf + dt);
            lpf_vel_y = alpha * lpf_vel_y + (1.0f - alpha) * v;
            lpf_vel_prev = lpf_vel_y;
            lpf_vel_ts = ts;
        }
        return sensor_direction * lpf_vel_y;
    }

    // SimpleFOC BLDCMotor::move(), angle mode
    void move() {
        shaft_angle    = shaftAngle();
        shaft_velocity = shaftVelocity();
        float shaft_angle_sp    = target;
        float shaft_velocity_sp = P_angle(shaft_angle_sp - shaft_angle, clock);
        shaft_velocity_sp = clampf(shaft_velocity_sp, -velocity_limit, velocity_limit);
        voltage_q = PID_velocity(shaft_velocity_sp - shaft_velocity, clock);
    }

    // plant step (commutation assumed correct after alignment)
    //
    // The encoder counts CW or CCW relative to the rotor, i.e. psi = +-theta.
    // FOC advances the electrical angle with +v_q, so in the *encoder* frame
    // the torque carries the sensor direction:
    void step() {
        float tau = (float)sensor_direction * Kt * voltage_q - B * omega - load;
        omega += (tau / J) * dt;
        psi += omega * dt;
        // mechanical stops
        if (psi > psi_max) { psi = psi_max; if (omega > 0) omega = 0; }
        if (psi < psi_min) { psi = psi_min; if (omega < 0) omega = 0; }
        clock += dt;
        enc.update(normAngle(psi + encoder_zero), clock * 1e6);
    }
};

// ---------------------------------------------------------------------------
// The sketch's calibration, reproduced exactly
// ---------------------------------------------------------------------------
struct Calib { float min_raw, max_raw, center_raw; };

// use_fixed_frame: sample motor.shaft_angle() (the frame the controller
// compares against) instead of the raw sensor angle.
static Calib calibrate(Sim &sim, bool use_fixed_frame)
{
    Calib c;
    // The sketch drives the motor to -3 V for 1 s, coasts 1 s, samples the
    // RAW angle, then repeats at +3 V.  (Torque mode, no phase resistance ->
    //  voltage.q = +/-3 V.)
    float lo = 1e9f, hi = -1e9f;

    for (int phase = 0; phase < 2; phase++) {
        float volts = (phase == 0) ? -3.0f : 3.0f;
        for (int i = 0; i < 20000; i++) {           // 1 s drive
            sim.voltage_q = volts;
            sim.step();
        }
        for (int i = 0; i < 20000; i++) {           // 1 s coast
            sim.voltage_q = 0.0f;
            sim.step();
        }
        float raw = use_fixed_frame ? sim.shaftAngle()   // the fix
                                     : sim.enc.getAngle();  // <- what the sketch samples
        if (phase == 0) { lo = hi = raw; }
        else { lo = std::min(lo, raw); hi = std::max(hi, raw); }
    }
    c.min_raw = lo; c.max_raw = hi;
    c.center_raw = lo + ((hi - lo) / 2.0f);
    return c;
}

static void report(const char *title, int sensor_dir, float enc_zero, float channel,
                   bool use_fixed_frame = false)
{
    Sim sim;
    sim.encoder_zero = enc_zero;
    sim.sensor_direction = sensor_dir;
    sim.PID_velocity.P = sim.PIDv_P;
    sim.PID_velocity.I = sim.PIDv_I;
    sim.PID_velocity.limit = sim.voltage_limit;

    printf("%s\n", title);
    printf("    sensor_direction = %+d, encoder zero = %.2f rad, channel = %.0f us\n",
           sensor_dir, enc_zero, channel);

    // The mechanical travel the servo actually has: +-0.6 rad around psi = 0.
    // We place the load so the +-3 V calibration sweep lands on it.
    sim.load = 0.0f;

    Calib c = calibrate(sim, use_fixed_frame);

    // The sketch's control law
    float steering = channel;
    sim.target = (steering - 1000.0f) * (c.max_raw - c.min_raw) / 1000.0f + c.min_raw;

    // run the loop
    float err = 0, worst = 0;
    for (int i = 0; i < 200000; i++) {      // 10 s
        sim.move();
        sim.step();
        float e = sim.target - sim.shaft_angle;
        if (fabsf(e) > PI) e = fmodf(e + PI, TWO_PI) - PI;
        err = e;
        if (fabsf(e) > worst) worst = fabsf(e);
        if (i < 6 || i % 40000 == 0)
            printf("      i=%6d psi=%+8.4f shaft_angle=%+8.4f target=%+8.4f err=%+8.4f vq=%+7.3f omega=%+8.3f\n",
                   i, sim.psi, sim.shaft_angle, sim.target, e, sim.voltage_q, sim.omega);
    }
    float achieved_psi = sim.enc.getAngle();
    printf("    calibrated (RAW frame)   min=%.4f max=%.4f centre=%.4f\n",
           c.min_raw, c.max_raw, c.center_raw);
    printf("    commanded motor.target    = %.4f (raw frame)\n", sim.target);
    printf("    final RAW encoder angle  = %.4f\n", achieved_psi);
    printf("    final controller error   = %+.4f rad  (%.1f deg)\n", err, err * 180.0f / PI);
    printf("    => %s\n\n", fabsf(err) < 0.10f ? "LOCKED on target" : "NOT LOCKED");
}

int main()
{
    printf("Simulating the BLDCServo position lock\n");
    printf("=======================================\n\n");

    printf("--- as the sketch is written today ---\n");
    report("A. CW, channel 1500 (centre)", +1, 0.0f, 1500.0f);
    report("B. CW, channel 1800 (80% of range)", +1, 0.0f, 1800.0f);
    report("C. CCW, channel 1500 (centre)", -1, 0.0f, 1500.0f);
    report("D. CCW, channel 1800 (80% of range)", -1, 0.0f, 1800.0f);
    printf("--- with the fix: calibrate in the controller frame (shaftAngle) ---\n");
    report("E. CW, channel 1800 (80% of range)  + fix", +1, 0.0f, 1800.0f, true);
    report("F. CCW, channel 1800 (80% of range) + fix", -1, 0.0f, 1800.0f, true);

    printf("D. What the controller actually compares:\n");
    printf("     error = motor.target - motor.shaft_angle\n");
    printf("            motor.target      = RAW encoder frame   (from the calibration)\n");
    printf("            motor.shaft_angle = sensor_direction * RAW - sensor_offset\n");
    printf("     -> the two only agree when sensor_direction == +1\n");
    return 0;
}
