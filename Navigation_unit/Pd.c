

#include "PD.h"

void PDController_Init(PIDController *pid) {

	/* Clear controller internal memory */
	pd->prevError  = 0.0f;
	pd->out = 0.0f;

}

float PDcontroller_Update(Pdcontrller *pd, float targetPos, float currentPos){

    /*
    * e(t) = r(t) - u(t) Error signal
    */
    float error = targetPos - currentPos;

    /*
    * Kp*error proptional part of pd controller
    */
    float proptional = pd->Kp * error;

    /*
    * Kp*derivative error is the dervitave part of the pd controller
    */
    float derivative = pd->Kd * (error - prevError);

    /*
    * U(out) = propitonal part + derivitave part
    */
    float out = proptional + derivative;

    pd->out = out;
    pd->prevError = error;

    return out;

} 

