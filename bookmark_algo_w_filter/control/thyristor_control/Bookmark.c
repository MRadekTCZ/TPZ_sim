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
int Tap_bit(TPPZ tppz, int tap, int enable)
{
	if (enable && tap >= 12)
	{
		// Bitmask for upper thyristors (odd bits) and lower thyristors (even bits)
		short int mask = (tppz.Tap_select[tap].Tap_up << 1) + tppz.Tap_select[tap].Tap_down;
		// The mask can be 00, 01 (only the lower of the pair is enabled), 11 (both enabled), or 10 (only the upper of the pair is enabled)
		// Bit shift depending on the tap number - Tap switch A (taps from 0 to 12)
		return ((0x3 & mask) << 2 * (tap - 12));
	}
	else if (enable && tap < 12)
	{
		short int mask = (tppz.Tap_select[tap].Tap_up << 1) + tppz.Tap_select[tap].Tap_down;
		return ((0x3 & mask) << (2 * (tap))); // Bit shift depending on the tap number, Tap switch B (taps from -12 to -1)
	}
	else return 0;
}
