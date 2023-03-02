#include "ir_sensor.h"
#include "car_driver.h"
//#include "stm32f1xx_hal.h"

static int8_t		P = 0;
static int16_t		I = 0;
static int8_t		D = 0;
static int8_t		Pre_Error = 0;

typedef enum
{
	FOLLOW_LINE_STATE,
	LOST_LINE_STATE,
	AT_STATION_STATE
}Follow_Line_State;

Follow_Line_State follow_line_state = FOLLOW_LINE_STATE;

uint8_t ir_sensor_read(void)
{
	uint8_t ir_value = 0;
	ir_value |= HAL_GPIO_ReadPin(IR1_GPIO_Port, IR1_Pin) << 4;
	ir_value |= HAL_GPIO_ReadPin(IR2_GPIO_Port, IR2_Pin) << 3;
	ir_value |= HAL_GPIO_ReadPin(IR3_GPIO_Port, IR3_Pin) << 2;
	ir_value |= HAL_GPIO_ReadPin(IR4_GPIO_Port, IR4_Pin) << 1;
	ir_value |= HAL_GPIO_ReadPin(IR5_GPIO_Port, IR5_Pin) << 0;
	return ir_value;
}

/*
	11111 = 31	error = -5
	01111 = 15	error = -4
	00111 = 07	error = -3
	10111 = 23	error = -2
	00011 = 03	error = -2
	10011 = 19	error = -1
	11011 = 27	error = 0
	10001 = 17	error = 0
	11001 = 25	error = 1
	11000 = 24	error = 2
	11101 = 29	error = 2
	11100 = 28	error = 3
	11110 = 30	error = 4
	00000 = 0		error = 5
*/

int8_t error_calculate(void)
{
	int8_t error = 0;
	uint8_t ir_value = ir_sensor_read();
	switch(ir_value)
	{
		case 31:
			error = -5;
			break;
		case 15:
			error = -4;
			break;
		case 7:
			error = -3;
			break;
		case 23:
			error = -2;
			break;
		case 3:
			error = -2;
			break;
		case 19:
			error = -1;
			break;
		case 27:
			error = 0;
			break;
		case 17:
			error = 0;
			break;
		case 25:
			error = 1;
			break;
		case 24:
			error = 2;
			break;
		case 29:
			error = 2;
			break;
		case 28:
			error = 3;
			break;
		case 30:
			error = 4;
			break;
		case 0:
			error = 5;
			break;
		default:
			break;
	}
	return error;
}

int16_t PID_value_calculate(void)
{
	int8_t error = error_calculate();
	if (error == 5)
	{
		follow_line_state = AT_STATION_STATE;
	}
	else if (error == -5)
	{
		follow_line_state = LOST_LINE_STATE;
	}
	else
	{
		follow_line_state = FOLLOW_LINE_STATE;
		P = error;
		I += error;
		D = error - Pre_Error;
		Pre_Error = error;
		int PID_value = (int16_t)(Kp*P + Ki*I + Kd*D);
		return PID_value;
	}
	return 0;
}

void car_following_line_handle(void)
{
	switch(follow_line_state)
	{
		case FOLLOW_LINE_STATE:
		{
			int8_t delta_pid_speed = (int8_t)(PID_value_calculate() / 10);
			int8_t right_speed = BASE_SPEED - delta_pid_speed;
			int8_t left_speed = BASE_SPEED + delta_pid_speed;
			Car_Control_Wheels(right_speed, left_speed);
			break;
		}
		case LOST_LINE_STATE:
		{
			PID_value_calculate();
			Car_Control(CAR_STOP_STATE, 0);
			break;
		}
		case AT_STATION_STATE:
		{
			PID_value_calculate();
			Car_Control(CAR_STOP_STATE, 0);
			break;
		}
		default:
			break;
	}
	
}
