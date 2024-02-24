/*
 * MRB_PLL.h
 *
 *  Created on: 23 lut 2024
 *      Author: Maciek
 */

#ifndef INC_MRB_PLL_H_

#include <math.h>
#define one_by_sqrt3 0.5773502
#define SQRT2 1.4142
#define PI 3.1415926
#define PI_BY_2 1.570796
#define PI_BY_3 1.047197
#define ONE_BY_SQRT2 0.70711
#define PID_1_KP 50
#define PID_1_KI 500
#define PID_1_KD 0.0
#define SAMPLE_FREQ 10000
#define SAMPLE_PRERIOD 0.0001
#define DIDT_SCALING_CONSTANT 0.00303030303
#define BASE_FREQ 50
#define BASE_PERIOD 0.02
#define BUFFER_SIZE 200
#define DT 0.0001
/******************************************
THREE PHASE FUNCTIONS
*******************************************/
struct Axis
{
float x;
float y;
};


struct ThreePhaseSystem
{
double phase_A;
double phase_B;
double phase_C;
};
typedef struct ThreePhaseSystem Three_phase_handler;
struct PID_unit
{
float KP;
float KI;
float KD;
double pid_integral;
double last_error;
};
typedef struct PID_unit PID_handler;


struct Axis AlfaBeta(float signal_a, float signal_b, float signal_c);
struct Axis DQ(float signal_alfa, float signal_beta, float angle);
float PID_Regulator(double error, PID_handler* pid);
float Integrator(double input_signal, double THRESHOLD_LOW, double THRESHOLD_HIGH, unsigned int handler);
struct ThreePhaseSystem PLL_3phase(float theta_main);
/******************************************
ONE PHASE FUNCTIONS
*******************************************/


struct PLL_1_PHASE_INTEGRAL
{
float moving_average_buffer[20];
double signal_integral_RMS_buffer[BUFFER_SIZE];
double signal_input_last;
double PLL_asin_last;
};
typedef struct PLL_1_PHASE_INTEGRAL PLL_1phase_Handler;


double RMS(double signal_actual, PLL_1phase_Handler* PLL_1phase );
double PLL_1phase(double input_signal, double rms, PLL_1phase_Handler* PLL_1phase );

#define INC_MRB_PLL_H_



#endif /* INC_MRB_PLL_H_ */
