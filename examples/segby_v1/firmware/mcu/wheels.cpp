/* Wheels hub — DEVICE HOOKS ONLY (§09, ADR-0003 Shape 1).
 *
 * The segby_v1 wheels board is a UART LEAF on an MKS Dual FOC v3.2 (ESP32
 * Lolin32-Lite): ONE node (0x05) driving BOTH wheels (M0 = left, M1 = right)
 * with SimpleFOC, each motor behind its OWN on-chip floor (§05). ALL the
 * mechanical wiring (the leaf route table, hub_on_body, the TWO floors'
 * init/on_command/ control_tick/status plumbing, the schedule,
 * hub_setup/hub_tasks) is GENERATED into firmware/gen/segby_v1/wheels.glue.h
 * from the IR. This file supplies only the device-specific hooks:
 *
 *   * wheels_device_setup()    — the dual-FOC bring-up (two BLDCMotor +
 *     BLDCDriver3PWM + MagneticSensorI2C (AS5600) channels, each on its own I²C
 *     bus; align ONLY the motors whose encoder answered, so a missing encoder
 *     never energises its motor — born-disarmed by default).
 *   * wheels_motor_left/right_drive() — run loopFOC()+move() with the floor's
 *     output torque as a q-axis voltage (torque-voltage mode).
 *   * wheels_post_control()    — sample the low-side current sense (telemetry
 *     only; control never reads it). This OVERRIDES the glue's weak no-op
 * default, and the glue calls it at the end of every control_tick.
 *
 * The real FOC drive is a SimpleFOC port. Every SimpleFOC call
 * is ARDUINO-guarded (this whole file is), so off-target the drive hooks are
 * absent and the floors + decode are exercised by the host C harnesses (which
 * never compile this). Including wheels.glue.h pulls in the generated
 * hub_setup/ hub_on_body/hub_tasks (C linkage). */
#if defined(ARDUINO)

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

/* This hub provides its own per-loop telemetry hook (current sense), so tell
 * the glue NOT to emit its no-op default — we define wheels_post_control()
 * below. */
#define WHEELS_POST_CONTROL_OVERRIDE
#include "wheels.glue.h"

/* --- Bench-verified electrical params (status "alignment ✓ · current sense ✓ ·
 * closed-loop velocity-mode working"). The MKS
 * Dual FOC v3.2 runs a 30-slot/20-pole outrunner (10 pole pairs,
 * cross-confirmed OLS slope 10.105 + A5@P11 motion) at 12 V. Leaving
 * phase_resistance UNSET keeps the PID/target in VOLTS, which matches our
 * torque→Uq mapping. --- */
static const int kPolePairs =
    10; /* confirmed: OLS slope 10.105, cross-checked */
static const float kVbusV = 12.0f;
static const float kDriverVLimit = 6.0f; /* half VBUS */
static const float kMotorVLimit = 4.0f;  /* caps the applied q-axis voltage */
static const float kMotorVAlign = 8.0f;  /* dominates this rotor's cogging */
static const uint32_t kI2cHz = 400000;

/* --- Low-side current sense (bench-verified):
 * INA181A2 ×50 V/V, 0.01 Ω shunt; M0 IA/IB = ADC 39/36, M1 IA/IB = ADC 35/34;
 * IC = NOT_SET (2-shunt, phase C reconstructed via KCL). Read OUT-OF-BAND for
 * telemetry ONLY — control stays torque-voltage, so a flaky sense never
 * destabilises the loop. The current LPF time constant smooths per-tick ADC
 * noise into a stable reported value. --- */
static const float kCsShuntOhms = 0.01f;
static const float kCsInaGain = 50.0f;
static const float kCurrLpfTf =
    0.02f; /* ≈8 Hz one-pole on the reported current */
#define M0_CS_IA 39
#define M0_CS_IB 36
#define M1_CS_IA 35
#define M1_CS_IB 34

/* --- MKS Dual FOC v3.2 pin map ---
 * Driver pins (the board's standard layout): M0 = pwm a/b/c 32/33/25,
 * M1 = pwm a/b/c 26/27/14, shared
 * enable 12. AS5600 encoders: each AS5600 shares
 * addr 0x36, so each rides its OWN I²C bus — M0 on Wire (SDA 19 / SCL 18), M1
 * on Wire1 (SDA 23 / SCL 5). */
#define M0_PWM_A 32
#define M0_PWM_B 33
#define M0_PWM_C 25
#define M1_PWM_A 26
#define M1_PWM_B 27
#define M1_PWM_C 14
#define M_ENABLE 12 /* shared driver enable */
#define M0_ENC_SDA 19
#define M0_ENC_SCL 18
#define M1_ENC_SDA 23
#define M1_ENC_SCL 5

/* M0 = left wheel, M1 = right wheel. */
static BLDCMotor m0_motor(kPolePairs);
static BLDCDriver3PWM m0_driver(M0_PWM_A, M0_PWM_B, M0_PWM_C, M_ENABLE);
static MagneticSensorI2C m0_sensor(AS5600_I2C);
static BLDCMotor m1_motor(kPolePairs);
static BLDCDriver3PWM m1_driver(M1_PWM_A, M1_PWM_B, M1_PWM_C, M_ENABLE);
static MagneticSensorI2C m1_sensor(AS5600_I2C);

/* Low-side current sense, two-shunt per motor (IC reconstructed via KCL). _NC
 * for the third pin. Constructed unconditionally; linked + init'd only for a
 * motor whose encoder answered (in setup). Read out-of-band, telemetry only. */
static LowsideCurrentSense m0_cs(kCsShuntOhms, kCsInaGain, M0_CS_IA, M0_CS_IB,
                                 _NC);
static LowsideCurrentSense m1_cs(kCsShuntOhms, kCsInaGain, M1_CS_IA, M1_CS_IB,
                                 _NC);
static LowPassFilter m0_ia_lpf(kCurrLpfTf), m0_ib_lpf(kCurrLpfTf);
static LowPassFilter m1_ia_lpf(kCurrLpfTf), m1_ib_lpf(kCurrLpfTf);

static bool m0_ready = false; /* M0 encoder present + initFOC ok */
static bool m1_ready = false;
static bool m0_cs_ready = false; /* M0 current sense linked + init ok */
static bool m1_cs_ready = false;
/* Latest filtered phase currents (amps) — telemetry only; control never reads
 * these. No :status field carries current in the v1 contract, so they are
 * SAMPLED
 * + AVAILABLE for a future telemetry port but not (yet) put on the wire
 * (changing the wire would be a contract change — out of scope). */
static float m0_ia_a = 0.0f, m0_ib_a = 0.0f;
static float m1_ia_a = 0.0f, m1_ib_a = 0.0f;
/* Latest closed-loop shaft velocity (rad/s) — the vel_left/vel_right SENSE
 * ports' telemetry (ADR-0009). Captured in wheels_post_control() AFTER both FOC
 * loops run, from SimpleFOC's BLDCMotor::shaft_velocity (the loop's velocity
 * estimate, updated by loopFOC()/move()). A non-ready motor holds at 0.0. */
static float m0_shaft_vel_rad_s = 0.0f;
static float m1_shaft_vel_rad_s = 0.0f;

/* AS5600 presence probe: a single, bounded I²C address-poll. GATES initFOC() —
 * calling initFOC on an absent encoder spins SimpleFOC's sensor-align on a
 * NACKing bus and hangs boot (no link, no telemetry). */
static const uint8_t kAs5600Addr = 0x36;
static bool as5600_present(TwoWire &w) {
  w.beginTransmission(kAs5600Addr);
  return w.endTransmission() == 0; /* 0 = ACK */
}

static void configure_motor(BLDCMotor &m) {
  m.voltage_limit = kMotorVLimit;
  m.voltage_sensor_align = kMotorVAlign;
  m.controller = MotionControlType::torque;
  m.torque_controller = TorqueControlType::voltage;
}

/* In torque-voltage mode SimpleFOC takes motor.target as Uq directly; we reuse
 * the floor's torque output as a q-axis voltage (an honest
 * first-cut), clamped to the motor voltage limit. */
static float torque_to_uq(float t) {
  if (t > kMotorVLimit)
    t = kMotorVLimit;
  if (t < -kMotorVLimit)
    t = -kMotorVLimit;
  return t;
}

/* --- device: the dual-FOC bring-up. born-disarmed floors are already inited by
 * the generated hub_setup BEFORE this runs (§05); here we bring up the
 * hardware. Align ONLY the motors whose encoder answered — a motor with an
 * absent AS5600 stays disabled, never energised, and never reaches drive_*().
 * --- */
extern "C" void wheels_device_setup(void) {
  /* arduino-esp32 3.x i2c-ng needs the HAL settled before Wire.begin (800 ms is
   * the empirical floor, bench-verified). Two AS5600s share addr 0x36, so
   * each rides its own bus. */
  delay(800);
  Wire.begin(M0_ENC_SDA, M0_ENC_SCL, kI2cHz);
  Wire1.begin(M1_ENC_SDA, M1_ENC_SCL, kI2cHz);

  bool m0_enc = as5600_present(Wire);
  bool m1_enc = as5600_present(Wire1);

  /* bring up + align ONLY the motors whose encoder answered (§05): a motor with
   * an absent AS5600 stays disabled, never energised — born-disarmed is the
   * default, and a skipped motor never reaches drive_*(). */
  if (m0_enc) {
    m0_sensor.init(&Wire);
    m0_motor.linkSensor(&m0_sensor);
    m0_driver.voltage_power_supply = kVbusV;
    m0_driver.voltage_limit = kDriverVLimit;
    m0_driver.init();
    m0_motor.linkDriver(&m0_driver);
    /* low-side sense: linkDriver BEFORE init (MCPWM ISR registration). Read
     * out-of-band for telemetry — we do NOT linkCurrentSense to the motor, so
     * control stays torque-voltage and a flaky sense can't destabilise it. */
    m0_cs.linkDriver(&m0_driver);
    m0_cs_ready = (m0_cs.init() == 1);
    configure_motor(m0_motor);
    m0_motor.init();
    m0_ready = m0_motor.initFOC();
    m0_motor.target = 0.0f;
  }
  if (m1_enc) {
    m1_sensor.init(&Wire1);
    m1_motor.linkSensor(&m1_sensor);
    m1_driver.voltage_power_supply = kVbusV;
    m1_driver.voltage_limit = kDriverVLimit;
    m1_driver.init();
    m1_motor.linkDriver(&m1_driver);
    m1_cs.linkDriver(&m1_driver);
    m1_cs_ready = (m1_cs.init() == 1);
    configure_motor(m1_motor);
    m1_motor.init();
    m1_ready = m1_motor.initFOC();
    m1_motor.target = 0.0f;
  }
}

/* --- device: apply effort to each motor and run its FOC loop. Touching a
 * skipped motor's loopFOC/move would read its dead sensor, so we drive ONLY the
 * motors that aligned. --- */
extern "C" void wheels_motor_left_drive(float effort) {
  if (!m0_ready)
    return;
  m0_motor.target = torque_to_uq(effort);
  m0_motor.loopFOC();
  m0_motor.move();
}

extern "C" void wheels_motor_right_drive(float effort) {
  if (!m1_ready)
    return;
  m1_motor.target = torque_to_uq(effort);
  m1_motor.loopFOC();
  m1_motor.move();
}

/* --- device: vel_left/vel_right SENSE reads (ADR-0009). Return the closed-loop
 * shaft velocity snapshotted in wheels_post_control() each tick. These are
 * non-blocking — a value is ALWAYS available (0.0 if the motor never aligned),
 * so they never time out and always return true. M0 = left, M1 = right. --- */
extern "C" bool wheels_vel_left_read(WheelSpeed *out) {
  out->rad_s = m0_shaft_vel_rad_s;
  return true;
}

extern "C" bool wheels_vel_right_read(WheelSpeed *out) {
  out->rad_s = m1_shaft_vel_rad_s;
  return true;
}

/* --- device: sample the low-side current sense for telemetry ONLY (the
 * current-sense read). One raw ADC read per linked phase, smoothed by
 * a one-pole LPF into a stable amps value. A motor with no linked sense
 * (encoder absent or init failed) holds at 0. This NEVER feeds control — a
 * flaky sense can't destabilise the torque-voltage loop. The v1 :status layout
 * carries no current field, so these are held in module state for a future
 * telemetry port, not put on the wire (that would be a contract change).
 *
 * This OVERRIDES the glue's weak wheels_post_control() default; the generated
 * control_tick calls it at the end of every loop, AFTER both FOC loops run.
 * --- */
extern "C" void wheels_post_control(void) {
  if (m0_cs_ready) {
    PhaseCurrent_s c = m0_cs.getPhaseCurrents();
    m0_ia_a = m0_ia_lpf(c.a);
    m0_ib_a = m0_ib_lpf(c.b);
  }
  if (m1_cs_ready) {
    PhaseCurrent_s c = m1_cs.getPhaseCurrents();
    m1_ia_a = m1_ia_lpf(c.a);
    m1_ib_a = m1_ib_lpf(c.b);
  }
  /* vel_left/vel_right telemetry (ADR-0009): snapshot SimpleFOC's closed-loop
   * velocity estimate AFTER both FOC loops ran this tick. Only for a ready
   * motor — a skipped motor's value stays 0.0. */
  if (m0_ready)
    m0_shaft_vel_rad_s = m0_motor.shaft_velocity;
  if (m1_ready)
    m1_shaft_vel_rad_s = m1_motor.shaft_velocity;
}

#endif /* ARDUINO */
