/*
 * Bokkmark.c
 *
 *  Created on: Feb 17, 2024
 *      Author: Maciek
 */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "Bookmark.h"

// Determination of the load type
enum Load LOAD_DET(float thetaU, float thetaI)
{
	enum Load Load_type_det;
	float angle_phi;

	angle_phi = thetaU - thetaI;

	if (((angle_phi > 0 + MARGIN) && (angle_phi < 90 - MARGIN)) || ((angle_phi < 0 - MARGIN) && (angle_phi < -90 - MARGIN)))  Load_type_det = Inductive;
	else if (((angle_phi < 0 - MARGIN) && (angle_phi > -90 + MARGIN)) || ((angle_phi > 0 + MARGIN) && (angle_phi > 90 + MARGIN)))  Load_type_det = Capacitive;
	else Load_type_det = Resistive;

	return Load_type_det;
}

// Determine the difference in tap change - 1 for gradual change, for a step change, it is the difference between the current and desired tap
short int Tap_diff(short int actual_tap_set, short int actual_tap, enum change_types change_type)
{
	short int e;
	if (change_type)
	{
		e = abs(actual_tap_set - actual_tap);
	}
	else e = 1;

	return e;
}

byte_frame_tap Tap_bit(TPPZ tppz, unsigned short int tap, unsigned short int phase )
{
byte_frame_tap _8bit_frame;
_8bit_frame.byte = 0x00;
switch(phase)
{
case 0:
	_8bit_frame.byte_8.b0 = 0;
	_8bit_frame.byte_8.b1 = 1;
	break;
case 1:
	_8bit_frame.byte_8.b0 = 1;
	_8bit_frame.byte_8.b1 = 0;
	break;
case 2:
	_8bit_frame.byte_8.b0 = 1;
	_8bit_frame.byte_8.b1 = 1;
	break;
default:
	_8bit_frame.byte_8.b0 = 0;
	_8bit_frame.byte_8.b1 = 0;
	break;
}
_8bit_frame.byte_8.tap_down = tppz.Tap_select[tap].Tap_down;
_8bit_frame.byte_8.tap_up = tppz.Tap_select[tap].Tap_up;
_8bit_frame.byte_8.actual_tap = tap;


return _8bit_frame;
}
// Software watchdog based on the tap switch structure and state machine
void program_Watchdog(TPPZ *tppz, struct Watchdog *watchdog, short int *state_machine, short int* actual_tap, short int actual_tap_set)
{
	if(*state_machine == 0)watchdog->counter = 0;
	else
	{
		if (watchdog->last_state_chage == *state_machine) watchdog->counter++;
		else
		{
			watchdog->counter = 0;
			watchdog->last_state_chage = *state_machine;
		}
		if (watchdog->counter >= SAMPLES_10ms)
		{
			*state_machine = 0;
			for (int i = 0; i < NUMBER_OF_TAPS; i++)
			{
				tppz->Tap_select[i].Tap_down = OFF;
				tppz->Tap_select[i].Tap_up = OFF;
			}

			tppz->Tap_select[actual_tap_set].Tap_down = ON;
			tppz->Tap_select[actual_tap_set].Tap_up = ON;
			*actual_tap = actual_tap_set;
			watchdog->counter = 0;
			watchdog->error_code = 0xFFFF;
		}
	}
}

// Determine the bit pattern for a specific tap


