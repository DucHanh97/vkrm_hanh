#ifndef CAR_REMOTE_H
#define CAR_REMOTE_H
#include <stdint.h>

typedef enum
{
	REMOTE_STOP_STATE,
	CAR_REMOTE_STATE,
	ARM_REMOTE_STATE
}RobotRemoteState;

void set_car_params_remote(int8_t argv[6]);
void car_remote_handle(void);
#endif
