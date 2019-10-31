#include "PID.h"

float PID(float setpoint, float input){
	Kp = 16;
	Kd = 0.01;
	Ki =  10.5;
    dt = 0.01;

	err =setpoint - input;

//	if(err > 0)
//		dir = - 1;
//	else
//		dir = 1;
	P = Kp*(err );
	I += 0.5*Ki*dt*(err );
	D = Kd*(err - err_1)/dt;
	PID_value =  (P + I + D);
//	pre_Out1 = PID_value;
	err_1 = err;
//    err_2 = err_1;
//    PID_value_pre = PID_value;
//
	if (PID_value>500)
		PID_value=500;
	if (PID_value<(-500))
		PID_value=-500;

//    if(PID_value>0)
//    return PID_value  ;
//   else
    return PID_value  ;
}
