#ifndef __CONTROLLER_AE483_H__
#define __CONTROLLER_AE483_H__

#include "stabilizer_types.h"
#include "commander.h"

// attitude_pid_controller.c
#include <stdbool.h>
#include "FreeRTOS.h"

// position_controller_pid.c
#include <math.h>
#include "num.h"

void controllerAE483Init(void);
bool controllerAE483Test(void);
void controllerAE483(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#endif //__CONTROLLER_AE483_H__
