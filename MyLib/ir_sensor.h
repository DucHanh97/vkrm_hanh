#ifndef IR_SENSOR_H
#define IR_SENSOR_H
#include "main.h"

/* ----- Define gains of PID controler ----- */
#define Kp			75
#define Ki			0
#define Kd			0
#define BASE_SPEED		25
/* ----------------------------------------- */

void car_following_line_handle(void);
uint8_t ir_sensor_read(void);
int8_t error_calculate(void);

#endif
