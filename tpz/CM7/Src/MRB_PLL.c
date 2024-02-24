/*
 * MRB_PLL.c
 *
 *  Created on: 23 lut 2024
 *      Author: Maciek
 */

#include <stdio.h>
#include "MRB_PLL.h"
#include "MRB_TRIGONOMETRIC_LIB.h"
/******************************************
THREE PHASE FUNCTIONS
*******************************************/

struct Axis AlfaBeta(float signal_a, float signal_b, float signal_c)
{
	struct Axis AB;
	AB.x = 0.33333*(2*signal_a - signal_b - signal_c);
	AB.y = one_by_sqrt3*(signal_b - signal_c);
	return AB;
}

struct Axis DQ(float signal_alfa, float signal_beta, float angle)
{
	struct Axis DQ;
	DQ.x = 0;
	DQ.y = 0;
	//if(angle >= 0 && angle <= 2*PI)
	//{
	DQ.x = signal_alfa*cos(angle) + signal_beta*sin(angle);
	DQ.y = -signal_alfa*sin(angle) + signal_beta*cos(angle);
	//}
	return DQ;
}

float PID_Regulator(double error, PID_handler* pid)
{
	float deficiency;
	float pid_derivative;

	pid->pid_integral = pid->pid_integral + pid->KI*SAMPLE_PRERIOD*error;
	pid_derivative = (error - pid->last_error )*pid->KD*SAMPLE_FREQ;

	deficiency = error*pid->KP + pid->pid_integral + pid_derivative;
	pid->last_error = error;
	return deficiency;
}

float Integrator(double input_signal, double THRESHOLD_LOW, double THRESHOLD_HIGH, unsigned int handler)
{
	static float integrated_s[16];
	if( (integrated_s[handler] < THRESHOLD_HIGH) && (integrated_s[handler] > THRESHOLD_LOW) )
	{
	integrated_s[handler] = integrated_s[handler] + input_signal*SAMPLE_PRERIOD;
	}
	else integrated_s[handler] = 0;

	return integrated_s[handler];
}

struct ThreePhaseSystem PLL_3phase(float theta_main)
{
	struct ThreePhaseSystem PLL_three_phase;
	PLL_three_phase.phase_A = theta_main;
	PLL_three_phase.phase_B = theta_main + 4*PI_BY_3;
	PLL_three_phase.phase_C = theta_main + 2*PI_BY_3;
	if(PLL_three_phase.phase_B > 2*PI) PLL_three_phase.phase_B = PLL_three_phase.phase_B - 2*PI;
	if(PLL_three_phase.phase_C > 2*PI) PLL_three_phase.phase_C = PLL_three_phase.phase_C - 2*PI;

	return PLL_three_phase;
}
/******************************************
ONE PHASE FUNCTIONS
*******************************************/
double RMS(double signal_actual, PLL_1phase_Handler* PLL_1phase )
{
	double rms;
	double signal_x2;
	double signal_integral;

	signal_x2 = signal_actual*signal_actual;
	signal_x2 = signal_x2*DT;
	signal_integral = 0;
	for(int i=0; i<BUFFER_SIZE-1;i++)
	{
	PLL_1phase->signal_integral_RMS_buffer[i] = PLL_1phase->signal_integral_RMS_buffer[i+1];
	signal_integral = signal_integral + PLL_1phase->signal_integral_RMS_buffer[i];
	}
	PLL_1phase->signal_integral_RMS_buffer[BUFFER_SIZE-1] = signal_x2;
	signal_integral = signal_integral + signal_x2;
	rms = sqrt(BASE_FREQ*signal_integral);

	return rms;
}

double PLL_1phase(double input_signal, double rms, PLL_1phase_Handler* PLL_1phase )
{
	double actual_relUnit=0;
	double PLL_asin;
	//float buffer_sum = 0;
	float buffer_past = 0;
	float buffer_future = 0;
	//float moving_average_10;
	double didt;

//Scaling
	if(rms != 0) actual_relUnit = input_signal* ONE_BY_SQRT2 / rms;

	//Moving Average with 5 behind actual values and 5 before
	for(int i=0; i<10;i++)
	{
		PLL_1phase->moving_average_buffer[i] = PLL_1phase->moving_average_buffer[i+1];
		buffer_past = buffer_past + PLL_1phase->moving_average_buffer[i];
	}
	for(int i=10; i<20-1;i++)
	{
		PLL_1phase->moving_average_buffer[i] = PLL_1phase->moving_average_buffer[i+1];
		buffer_future = buffer_future + PLL_1phase->moving_average_buffer[i];
	}
	PLL_1phase->moving_average_buffer[20-1] = actual_relUnit;
	buffer_future = (buffer_future+actual_relUnit);
	//buffer_sum = buffer_past + buffer_future;
	//moving_average_10 = buffer_sum* 0.05;
	//******************************************//
	//didt = (actual_relUnit - PLL_1phase->signal_input_last) * SAMPLE_FREQ * DIDT_SCALING_CONSTANT ;
	didt = (buffer_future-buffer_past);
	if(actual_relUnit >= 1)
	{
	PLL_asin = PI_BY_2;
	}

	else if(actual_relUnit <= -1)
	{
	PLL_asin = PI_BY_2*3;
	}
	else if(actual_relUnit > 0 && didt > 0)
	{
	PLL_asin = asin(actual_relUnit);
	}
	else if(didt <= 0)
	{
	PLL_asin = -asin(actual_relUnit) + PI_BY_2 + PI_BY_2;
	}

	else if(actual_relUnit < 0 && didt > 0)
	{
	PLL_asin = asin(actual_relUnit) + PI_BY_2 * 3 + PI_BY_2;
	}
	else PLL_asin=0;

	//Protection from PLL falling
	if( (PLL_asin-PLL_1phase->PLL_asin_last) <0 && (PLL_asin-PLL_1phase->PLL_asin_last) >-PI )
	{
	PLL_asin = PLL_1phase->PLL_asin_last;
	}

	PLL_1phase->signal_input_last = actual_relUnit;
	PLL_1phase->PLL_asin_last = PLL_asin;
	//PLL_asin = PLL_asin + MOVING_AVERAGE_DRAFT;
	return PLL_asin;
}
