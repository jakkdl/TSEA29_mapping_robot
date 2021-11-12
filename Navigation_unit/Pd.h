
#define PD_CONTROLLER_H

typedef struct {

    /* Pd-constants */
    float Kp;
    float Kd;

    /* internal memory */
    float prevError;

    /* controller output */
    float out;

} PDcontroller;

void PDcontroller_Init(PDController *pd);

float PDcontroller_Update(Pdcontrller *pd, float targetPos, float currentPos);
