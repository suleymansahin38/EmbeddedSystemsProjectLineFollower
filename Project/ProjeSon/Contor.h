#ifndef Contor_H
#define Contor_H

#include "stm32f10x.h"

typedef struct {
		int infos[8];
    int pid_position;
    float Kp;
    float Ki;
    float Kd;
    int proportional, integrational, Derivative;
    int error_data[10];
    int total_error;
    int  maxspeedr;
    int maxspeedl;
    int right_motor_speedBase;
    int left_motor_speedBase;
    int Var_ARR;
    int last_error;
    int i;
    int k;
	  int motor_speed;
    int variable;
    int actv_cnt;
    int last_end;
    int counted_white;
} Contor;


extern Contor ContorInstance;
extern Contor *Controller; // Declare a pointer to My_PID

#endif
