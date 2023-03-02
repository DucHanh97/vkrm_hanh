#ifndef ARM_ROBOT_H
#define ARM_ROBOT_H
#include "servo.h"
#define ORIGIN_Z	90
#define ORIGIN_X	50
#define ORIGIN_Y	150
#define ORIGIN_K	109
typedef struct
{
	Servo SV_Z;
	Servo SV_X;
	Servo SV_Y;
	Servo SV_K;
}Arm_Robot_Typedef;
void Arm_Robot_Init(Arm_Robot_Typedef *arm_rb,TIM_HandleTypeDef *sv_tim);
void Arm_Control_by_Step(Arm_Robot_Typedef *arm_rb, int8_t Z, int8_t X, int8_t Y, int8_t K);
void Arm_Go_to_Position(Arm_Robot_Typedef *arm_rb, uint8_t angle_z, uint8_t angle_x, uint8_t angle_y, uint8_t angle_k);
void Set_Default_State(Arm_Robot_Typedef *arm_rb);
#endif
