#include "stabilizer.h"
#include "stabilizer_types.h"

#include "pid.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_ae483.h"

#include "log.h"
#include "param.h"
#include "math3d.h"


// attitude_pid_controller.c BEGIN
#include <stdbool.h>
#include "FreeRTOS.h"

#define ATTITUDE_LPF_CUTOFF_FREQ      15.0f
#define ATTITUDE_LPF_ENABLE false
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ 30.0f
#define ATTITUDE_RATE_LPF_ENABLE false

static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}
// attitude_pid_controller.c END

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static bool tiltCompensationEnabled = false;
// static bool positionIsInit;

// attitude_pid_controller.c BEGIN
static bool attitudeIsInit;
PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;
// attitude_pid_controller.c END

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

void controllerAE483Init(void)
{
  // attitudeControllerInit(ATTITUDE_UPDATE_DT);
  if (!attitudeIsInit) {
    //TODO: get parameters from configuration manager instead
    pidInit(&pidRollRate,  0, PID_ROLL_RATE_KP,  PID_ROLL_RATE_KI,  PID_ROLL_RATE_KD,
        ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
    pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
        ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
    pidInit(&pidYawRate,   0, PID_YAW_RATE_KP,   PID_YAW_RATE_KI,   PID_YAW_RATE_KD,
        ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);

    pidSetIntegralLimit(&pidRollRate,  PID_ROLL_RATE_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidYawRate,   PID_YAW_RATE_INTEGRATION_LIMIT);

    pidInit(&pidRoll,  0, PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  ATTITUDE_UPDATE_DT,
        ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
    pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, ATTITUDE_UPDATE_DT,
        ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
    pidInit(&pidYaw,   0, PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   ATTITUDE_UPDATE_DT,
        ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);

    pidSetIntegralLimit(&pidRoll,  PID_ROLL_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidYaw,   PID_YAW_INTEGRATION_LIMIT);

    attitudeIsInit = true;
  }
  positionControllerInit();
}

bool controllerAE483Test(void)
{
  bool pass = true;

  pass &= attitudeIsInit;

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void controllerAE483(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
       attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
    } else {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    // attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
    //                             attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
    //                             &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);
    {
    pidSetDesired(&pidRoll, attitudeDesired.roll);
    rateDesired.roll = pidUpdate(&pidRoll, state->attitude.roll, true);

    // Update PID for pitch axis
    pidSetDesired(&pidPitch, attitudeDesired.pitch);
    rateDesired.pitch = pidUpdate(&pidPitch, state->attitude.pitch, true);

    // Update PID for yaw axis
    float yawError;
    yawError = attitudeDesired.yaw - state->attitude.yaw;
    if (yawError > 180.0f)
      yawError -= 360.0f;
    else if (yawError < -180.0f)
      yawError += 360.0f;
    pidSetError(&pidYaw, yawError);
    rateDesired.yaw = pidUpdate(&pidYaw, state->attitude.yaw, false);
    }

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      // attitudeControllerResetRollAttitudePID();
      pidReset(&pidRoll);
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      // attitudeControllerResetPitchAttitudePID();
      pidReset(&pidPitch);
    }

    // TODO: Investigate possibility to subtract gyro drift.
    // attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
    //                          rateDesired.roll, rateDesired.pitch, rateDesired.yaw);
    // attitudeControllerGetActuatorOutput(&control->roll,
    //                                     &control->pitch,
    //                                     &control->yaw);
    {
      pidSetDesired(&pidRollRate, rateDesired.roll);
      control->roll = saturateSignedInt16(pidUpdate(&pidRollRate, sensors->gyro.x, true));

      pidSetDesired(&pidPitchRate, rateDesired.pitch);
      control->pitch = saturateSignedInt16(pidUpdate(&pidPitchRate, -sensors->gyro.y, true));

      pidSetDesired(&pidYawRate, rateDesired.yaw);
      control->yaw = saturateSignedInt16(pidUpdate(&pidYawRate, sensors->gyro.z, true));
    }

    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }

  if (tiltCompensationEnabled)
  {
    control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();
  }
  else
  {
    control->thrust = actuatorThrust;
  }

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    // attitudeControllerResetAllPID();
    {
      pidReset(&pidRoll);
      pidReset(&pidPitch);
      pidReset(&pidYaw);
      pidReset(&pidRollRate);
      pidReset(&pidPitchRate);
      pidReset(&pidYawRate);
    }
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)


/**
 * Controller parameters
 */
PARAM_GROUP_START(controller)
/**
 * @brief Nonzero for tilt compensation enabled (default: 0)
 */
PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)


// Logging from attitude_pid_controller.c BEGIN
/**
 *  Log variables of attitude PID controller
 */
LOG_GROUP_START(pid_attitude)
/**
 * @brief Propertional output roll
 */
LOG_ADD(LOG_FLOAT, roll_outP, &pidRoll.outP)
/**
 * @brief Integral output roll
 */
LOG_ADD(LOG_FLOAT, roll_outI, &pidRoll.outI)
/**
 * @brief Derivative output roll
 */
LOG_ADD(LOG_FLOAT, roll_outD, &pidRoll.outD)
/**
 * @brief Propertional output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitch.outP)
/**
 * @brief Intergral output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitch.outI)
/**
 * @brief Derivative output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitch.outD)
/**
 * @brief Propertional output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYaw.outP)
/**
 * @brief Intergal output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYaw.outI)
/**
 * @brief Derivative output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYaw.outD)
LOG_GROUP_STOP(pid_attitude)

/**
 *  Log variables of attitude rate PID controller
 */
LOG_GROUP_START(pid_rate)
/**
 * @brief Propertional output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outP, &pidRollRate.outP)
/**
 * @brief Intergral output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outI, &pidRollRate.outI)
/**
 * @brief Derivative output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outD, &pidRollRate.outD)
/**
 * @brief Propertional output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitchRate.outP)
/**
 * @brief Intergral output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitchRate.outI)
/**
 * @brief Derivative output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitchRate.outD)
/**
 * @brief Propertional output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYawRate.outP)
/**
 * @brief Intergral output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYawRate.outI)
/**
 * @brief Derivative output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYawRate.outD)
LOG_GROUP_STOP(pid_rate)

/**
 * Tuning settings for the gains of the PID
 * controller for the attitude of the Crazyflie which consists
 * of the Yaw Pitch and Roll
 */
PARAM_GROUP_START(pid_attitude)
/**
 * @brief Propertional gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRoll.kp)
/**
 * @brief Intergral gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRoll.ki)
/**
 * @brief Derivative gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRoll.kd)
/**
 * @brief Propertional gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitch.kp)
/**
 * @brief Intergral gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitch.ki)
/**
 * @brief Derivative gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitch.kd)
/**
 * @brief Propertional gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYaw.kp)
/**
 * @brief Intergral gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYaw.ki)
/**
 * @brief Derivative gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYaw.kd)
PARAM_GROUP_STOP(pid_attitude)

/**
 * Tuning settings for the gains of the PID
 * controller for the rate angels of the Crazyflie which consists
 * of the Yaw Pitch and Roll rates
 */
PARAM_GROUP_START(pid_rate)
/**
 * @brief Propertional gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRollRate.kp)
/**
 * @brief Intergral gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRollRate.ki)
/**
 * @brief Derivative gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRollRate.kd)
/**
 * @brief Propertional gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitchRate.kp)
/**
 * @brief Intergral gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitchRate.ki)
/**
 * @brief Derivative gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitchRate.kd)
/**
 * @brief Propertional gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYawRate.kp)
/**
 * @brief Intergral gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYawRate.ki)
/**
 * @brief Derivative gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYawRate.kd)
PARAM_GROUP_STOP(pid_rate)
// attitude_pid_controller.c END
