#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "Bookmark.h"

//Wyznaczenie charakteru obciazenia
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

//Wyznaczenie roznicy zmiany zaczepu - 1 dla zmiany stopniowej, dla zmiany skokowej jest roznica pomiedzy aktualnym a zadanym zaczepem
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


// Watchdog programowy, bazuje na strukturze przelacznika zaczepow i maszynie stanow
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
int Tap_bit(TPPZ tppz, int tap, int enable)
{
	if (enable && tap >= 12)
	{
		//Maska bitowa  górne tyrystory (bity nieparzyste) dolne tyrystory (bity parzyste)
		short int mask = (tppz.Tap_select[tap].Tap_up << 1) + tppz.Tap_select[tap].Tap_down;
		// Maska moze byæ równa 00, 01 (za³¹czony tylko dolny z pary), 11 (za³¹czone oba) lub 10 (za³¹czony tylko górny z pary)
			//Przesuniecie bitowe w zale¿noœci od numeru zaczepu -  prze³¹cznik zaczepów A (zaczepy od 0 do 12)
		return ((0x3 & mask) << 2 * (tap - 12));
	}
	else if (enable && tap < 12)
	{
		short int mask = (tppz.Tap_select[tap].Tap_up << 1) + tppz.Tap_select[tap].Tap_down;
		return ((0x3 & mask) << (2 * (tap))); //Przesuniecie bitowe w zale¿noœci od numeru zaczepu, prze³¹cznik zaczepów B (zaczepy od -12 do -1)
	}
	else return 0;
}
