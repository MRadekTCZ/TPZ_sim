#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "DllHeader.h"
#include <stdbool.h>
#include "Bookmark.h"
/*********************************************************************************************************************************/
#define SYMULACJA
/*********************************************************************************************************************************/
#ifdef SYMULACJA
#define type_real double
#define keyword
#else
#define type_real type_real
#define keyword static __inline
#endif

double voltage_phase[3], current_phase[3]; //k¹ty fazowe theta i, theta u
double actual_current[3]; // pomiar pr¹du
static type_real temp[10];
double time_temp = 0.0;
TPPZ tppz[3];

struct Watchdog watchdog[3];



short int tap_set_value[5];// = { 0,2, 3, 4, 1 };// mona rcznie zada zaczepy
double tap_set_time[5];// = { 0, 0.024, 0.071, 0.112, 0.216 };


short int Global_set_actual_tap; //sygna³ przekazuj¹cy informacjê o zadanym zaczepie




//bool next_step;

static type_real TS, fs;


float q_error; //uchyb sk³adowej q uk³adu PLL



static short int state_machine[3]; //Maszyna stanów dla ka¿dej fazy
static short int set_actual_tap[3] = { 0,0,0 }; //Zadany numer zaczepu dla ka¿dej fazy (mogê one byæ przyjmowaæ z pewnym opóŸnieniem wzglêdem pozosta³ych faz
static short int actual_tap[3] = { 0,0,0 }; //Aktualny numer - w trakcie sekwencji prze³¹czanie mo¿e byæ inny dla ró¿nych faz. W stanie ustalony jest taki sam dla wszystkich faz
enum Load Loadtype[3] = { Inductive,Inductive,Inductive };
short int tap_delta; //ró¿nica zmiany zaczepu - dla zmiany stopniowej zawsze 1, dla zmiany skokowej zalezy od ró¿nicy zadanego i aktualnego zaczepu



void plecsSetSizes(struct SimulationSizes* aSizes)
{
	aSizes->numInputs = 14;
	aSizes->numOutputs = 11 + 5;
	aSizes->numStates = 0;
	aSizes->numParameters = 1; //number of user parameters passed in
}

//This function is automatically called at the beginning of the simulation
void plecsStart(struct SimulationState* aState)
{
	TS = aState->parameters[0];	//40e-6		okres impulsowania fs = 25kHz
	fs = 1.0 / TS;// //f prbkowania
	aState->outputs[0] = 0;
	aState->outputs[1] = 1;
	aState->outputs[2] = 1;
	aState->outputs[3] = 0;
	aState->outputs[4] = 0;
	aState->outputs[5] = 0;
	aState->outputs[6] = 0;
	aState->outputs[7] = 0;
	aState->outputs[8] = 0;
	aState->outputs[9] = 0;
	aState->outputs[10] = 0;
	


}

//This function is automatically called every sample time
//output is written to DLL output port after the output delay
void plecsOutput(struct SimulationState* aState)
{

	time_temp = aState->time;

	/*********************************************************************************************************************************/
	//POCZATEK ALGORYTMU STEROWANIA
	/*********************************************************************************************************************************/

	//Inicjalizacja
	if (time_temp < 0.001) {
		q_error = aState->inputs[12];
		tppz[0].phase = L1;
		
		for (int i = 0; i < 5; i++)
		{
			tap_set_time[i] = aState->inputs[2 + 2 * i];
			tap_set_value[i] = aState->inputs[3 + 2 * i];
		}
		actual_tap[0] = 0;

		tppz[0].Tap_select[0].Tap_down = ON;
		tppz[0].Tap_select[0].Tap_up = ON;
	}

	//Program ci¹g³y
	if (time_temp < 0.05) {
		Global_set_actual_tap = ACTUAL_TAP(tap_set_value, tap_set_time, time_temp); //Odczyt zadanych wartoœci zaczepu w chwilach czasowych na potrzeby symulacji
	}
	else Global_set_actual_tap = aState->inputs[2];
	
	q_error = aState->inputs[12];
	//Odczyt k¹tów fazowych
	voltage_phase[0] = aState->inputs[1]; // input 1 must be voltage
	current_phase[0] = aState->inputs[0]; // input 2 must be current
	actual_current[0] = aState->inputs[13];
	//Global_set_actual_tap = aState->inputs[3];
	

	//Debugowanie, podgl¹d niektórych dodatkowych zmiennych
	aState->outputs[13] = LOAD_DET(voltage_phase[0], current_phase[0]);
	aState->outputs[14] = fabs(1 - q_error);
	//aState->outputs[14] = actual_tap[phase];
	aState->outputs[14] = Global_set_actual_tap;;
	aState->outputs[15] = Global_set_actual_tap;




	

	if (time_temp > 0.03) 
	{		
		
		//Pêtla dla wszystkich trzech faz - w symulacji jest tylko jeden, lecz jest to rozwi¹zanie skalowalne
		for (int phase = 0; phase < 3; phase++) 
		{
			//program_Watchdog(&tppz[phase], &watchdog[phase], &state_machine[phase], &actual_tap[phase], set_actual_tap[phase]);
			tap_delta = Tap_diff(set_actual_tap[phase], actual_tap[phase], step);
			if (actual_current[phase] <= NO_LOAD_STATE)
			{

				set_actual_tap[phase] = Global_set_actual_tap;
				if (actual_tap[phase] != set_actual_tap[phase])
				{
					switch (state_machine[phase])
					{
					case 0:
						if (voltage_phase[phase] < (10 - MARGIN)) // Recyzja dla obci¹¿enia rezystancyjnego jest podejmnowana jedynie na bazie napiêcia - prze³¹czenia jest niesekwencyjne
						{
							tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
							tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
							state_machine[phase] = 1;
						}
						break;
					case 1:
						if (voltage_phase[phase] > (350 + MARGIN)) // Recyzja dla obci¹¿enia rezystancyjnego jest podejmnowana jedynie na bazie napiêcia - prze³¹czenia jest niesekwencyjne
						{
							tppz[phase].Tap_select[set_actual_tap[phase]].Tap_down = ON;
							tppz[phase].Tap_select[set_actual_tap[phase]].Tap_up = ON;
							actual_tap[phase] = set_actual_tap[phase];
							state_machine[phase] = 0;
						}
						break;
					}
				}

			}

			else
			{

				//Przygotowanie algorytmu na to, aby nie zmienia³ zaczepu, gdy w systemie trwa stan nieustalony
				if (state_machine[phase] == 0 && (q_error < 0.01))
				{

					Loadtype[phase] = LOAD_DET(voltage_phase[phase], current_phase[phase]);
					
					set_actual_tap[phase] = Global_set_actual_tap;

				}
				//Przygotowanie algorytmu na to, aby przerwa³ sekwencje prze³¹czania, gdy w sieci rozpocznie siê stan przejœciowy (powrót z stanu 1 do stanu 0)
				else if (state_machine[phase] == 1 && (q_error >= 0.01))
				{
					set_actual_tap[phase] = actual_tap[phase];

					tppz[phase].Tap_select[actual_tap[phase]].Tap_down = ON;
					tppz[phase].Tap_select[actual_tap[phase]].Tap_up = ON;
					state_machine[phase] = 0;

				}

				//Algorytm zmiany zaczepu
				//Tutaj zaczyna si ptla while docelowego algorytmu na mikrokontrolerze /FPGA
				if (actual_tap[phase] == set_actual_tap[phase])
				{
					watchdog[phase].counter = 0; //je¿eli zaczep zadany jest równy aktualnemu - reset watchdoga
				}
				else if (actual_tap[phase] != set_actual_tap[phase])
				{

					if (actual_tap[phase] < set_actual_tap[phase])
					{
						if (Loadtype[phase] == Resistive) // Warunki przeczenia przy obcieniu rezystancyjnym
						{
							switch (state_machine[phase])
							{
							case 0:
								if (voltage_phase[phase] > (180 + MARGIN)) // Recyzja dla obci¹¿enia rezystancyjnego jest podejmnowana jedynie na bazie napiêcia - prze³¹czenia jest niesekwencyjne
								{
									tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
									tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
									state_machine[phase] = 1;
								}
								break;

							case 1:
								if (voltage_phase[phase] > (360 - 5 * MARGIN))  //zezwolenie na zwarcie miedzy zaczepowe o niewielkiej wartoci
								{

									tppz[phase].Tap_select[actual_tap[phase]].Tap_up = ON;
									tppz[phase].Tap_select[actual_tap[phase]].Tap_down = ON;
									state_machine[phase] = 2;
								}
								break;
							case 2:
								if (voltage_phase[phase] > 2 && voltage_phase[phase] < (90 - MARGIN) && current_phase[phase] > 2 && current_phase[phase] < (90 - MARGIN))
								{

									actual_tap[phase]++;
									state_machine[phase] = 0;
									
								}
								break;
							}
						}

						if (Loadtype[phase] == Inductive) // Warunki przeczenia przy obcieniu indukcyjnym 
						{
							switch (state_machine[phase])
							{

							case 0:
								if (voltage_phase[phase] < (90 - MARGIN) && voltage_phase[phase] > MARGIN && current_phase[phase] < (90 - MARGIN) && current_phase[phase] > MARGIN) // gdy prad jest w fazie z napieciem
								{
									tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
									tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
									state_machine[phase] = 1;
								}
								break;
								// Warunki case 0 i case 1, s¹ podobne, ale zawsze pomiêdzy nimi jest jeden cykl programu, co daje czas na wy³¹czenie siê tyrystora
							case 1:
								if (voltage_phase[phase] > MARGIN && voltage_phase[phase] < (180 - MARGIN) && current_phase[phase] > MARGIN && current_phase[phase] < (180 - MARGIN))
								{

									tppz[phase].Tap_select[actual_tap[phase] + tap_delta].Tap_up = ON;
									state_machine[phase] = 2;

								}

								break;

							case 2:
								if (voltage_phase[phase] > (90 + MARGIN) && voltage_phase[phase] < (180 - MARGIN) && current_phase[phase] >(90 + MARGIN))  //Sekwencyjne przeczanie
								{

									tppz[phase].Tap_select[actual_tap[phase] + tap_delta].Tap_down = ON;
									state_machine[phase] = 3;

								}

								break;

							case 3:
								if (voltage_phase[phase] > MARGIN && voltage_phase[phase] < (90 - MARGIN) && current_phase[phase] > MARGIN && current_phase[phase] < (90 - MARGIN) && state_machine[phase] == 3)
								{
									//actual_tap[phase]++;
									actual_tap[phase] += tap_delta;
									state_machine[phase] = 0;
									

								}
								break;
							}



						}

						if (Loadtype[phase] == Capacitive) //warunki prze³¹czenia przy obci¹¿eniu pojemnoœciowym
						{
							switch (state_machine[phase])
							{

							case 0:
								if (voltage_phase[phase] < (120 - MARGIN) && voltage_phase[phase] > 2 && current_phase[phase] < (120 - MARGIN) && current_phase[phase] > 2)
								{
									tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
									tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
									state_machine[phase] = 1;
								}
								break;

							case 1:
								if (voltage_phase[phase] < (180 + MARGIN) && current_phase[phase] < (180 - MARGIN) && current_phase[phase] >(120 + MARGIN))
								{
									tppz[phase].Tap_select[actual_tap[phase] + tap_delta].Tap_down = ON;
									state_machine[phase] = 2;
								}
								break;

							case 2:
								if (voltage_phase[phase] > (180 + MARGIN) && current_phase[phase] > (180 + MARGIN))
								{
									tppz[phase].Tap_select[actual_tap[phase] + tap_delta].Tap_up = ON;
									state_machine[phase] = 3;
								}
								break;

							case 3:
								if (voltage_phase[phase] > 2 && voltage_phase[phase] < (90 - MARGIN) && current_phase[phase] > 2 && current_phase[phase] < (90 - MARGIN))
								{
									//actual_tap[phase]++;
									actual_tap[phase] += tap_delta;
									state_machine[phase] = 0;
									
								}
								break;
							}

						}
					}

					else if (actual_tap[phase] > set_actual_tap[phase])
					{
						if (Loadtype[phase] == Resistive) // Warunki przeczenia przy obcieniu rezystancyjnym
						{
							switch (state_machine[phase])
							{
							case 0:
								if (voltage_phase[phase] > (180 + MARGIN))
								{
									tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
									tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
									state_machine[phase] = 1;
								}
								break;
							case 1:
								if (voltage_phase[phase] > (360 - 5 * MARGIN))
								{
									tppz[phase].Tap_select[actual_tap[phase] - 1].Tap_up = ON;
									tppz[phase].Tap_select[actual_tap[phase] - 1].Tap_down = ON;
									state_machine[phase] = 2;
								}
								break;
							case 2:
								if (voltage_phase[phase] > 2 && voltage_phase[phase] < (90 - MARGIN) && current_phase[phase] > 2 && current_phase[phase] < (90 - MARGIN))
								{
									actual_tap[phase]--;
									state_machine[phase] = 0;
									

								}
								break;
							}
						}

						if (Loadtype[phase] == Inductive) //Warunki przeczenia przy obcieniu indukcyjnym
						{
							switch (state_machine[phase])
							{
							case 0:
								if (voltage_phase[phase] < (180 - MARGIN) && voltage_phase[phase] > 2 && current_phase[phase] < (180 - MARGIN) && current_phase[phase] > 2) //Wy³¹czenie tyrystorow przed zmian¹ polaryzacji
								{
									tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
									tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
									state_machine[phase] = 1;

								}
								break;
							case 1:
								if (voltage_phase[phase] > (180 + MARGIN) && current_phase[phase] < (180 - MARGIN)) //prze³¹czenie "na zak³adkê" - za³¹czenie przecienego tyrystora
								{
									tppz[phase].Tap_select[actual_tap[phase] - tap_delta].Tap_down = ON;
									state_machine[phase] = 2;

								}
								break;
							case 2:
								if (voltage_phase[phase] > (180 + MARGIN) && current_phase[phase] > (180 + MARGIN))
								{
									tppz[phase].Tap_select[actual_tap[phase] - tap_delta].Tap_up = ON;
									state_machine[phase] = 3;

								}
								break;
							case 3:
								if (voltage_phase[phase] > 2 && voltage_phase[phase] < (90 - MARGIN) && current_phase[phase] > 2 && current_phase[phase] < (90 - MARGIN))
								{

									//actual_tap[phase]--;
									actual_tap[phase] -= tap_delta;
									state_machine[phase] = 0;
									
								}
								break;
							}

						}

						if (Loadtype[phase] == Capacitive) // Warunki przeczenia przy obcieniu pojemnoœciowym
						{

							//Pierwszy wariant - bezzwarciowy
							switch (state_machine[phase])
							{

							case 0:
								if (voltage_phase[phase] > (0 + MARGIN) && current_phase[phase] > (0 + MARGIN) && current_phase[phase] < (180 - MARGIN)) // gdy prad jest w fazie z napieciem
								{
									tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;


									state_machine[phase] = 1;
								}
								break;
							case 1:
								if (voltage_phase[phase] > (90 + MARGIN) && current_phase[phase] > (180 + MARGIN) && voltage_phase[phase] < (180 - MARGIN))  //zezwolenie na zwarcie miedzy zaczepowe o niewielkiej wartoci

								{

									tppz[phase].Tap_select[actual_tap[phase] - tap_delta].Tap_down = ON;
									tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;


									state_machine[phase] = 2;
								}
								break;
							case 2:
								if (voltage_phase[phase] > (180 + MARGIN) && current_phase[phase] > (180 + MARGIN))  //zezwolenie na zwarcie miedzy zaczepowe o niewielkiej wartoci
								{
									tppz[phase].Tap_select[actual_tap[phase] - tap_delta].Tap_up = ON;
									state_machine[phase] = 3;
								}
								break;
							case 3:
								if (voltage_phase[phase] > MARGIN && current_phase[phase] > (MARGIN))
								{
									//actual_tap[phase]--;
									actual_tap[phase] -= tap_delta;
									state_machine[phase] = 0;
								}
								break;
							}

							/*
							//Drugi wariant - dla cos fi bliskiego 1
							switch (state_machine[phase])
							{
							case 0:
								if (voltage_phase[phase] > (180 + MARGIN) && current_phase[phase] > (180 + MARGIN) && state_machine[phase] == 0) // gdy prad jest w fazie z napieciem
								{
									tppz[phase].Tap_select[actual_tap[phase]].Tap_down = OFF;
									tppz[phase].Tap_select[actual_tap[phase]].Tap_up = OFF;
									state_machine[phase] = 1;
								}
								break;
							case 1:
								if (current_phase[phase] > (360 - 1 * MARGIN) && state_machine[phase] == 1)  //zezwolenie na zwarcie miedzy zaczepowe o niewielkiej wartoci
								{

									tppz[phase].Tap_select[actual_tap[phase]-1].Tap_down = ON;
									tppz[phase].Tap_select[actual_tap[phase]-1].Tap_up = ON;
									state_machine[phase] = 2;
								}
								break;
							case 2:
								if (state_machine[phase] == 2 && voltage_phase[phase] > 2 && voltage_phase[phase] < (90 - MARGIN) && current_phase[phase] > 2 && current_phase[phase] < (90 - MARGIN))
								{
									actual_tap[phase]--;
									state_machine[phase] = 0;
									
								}
								break;
							}
							*/
						}

					}

				}
			}
		}
		
		

	}
	/*********************************************************************************************************************************/
	//KONIEC ALGORYTMU STEROWANIA
	/*********************************************************************************************************************************/

	//IN: pomiary - wywolane na koncu funkcji w celu symulacji opoznienia pomiedzy sprzezeniami i sterowaniem




	//OUT: sterowanie tyrystorami

	//Przypisanie stanw poszczeglnym wyjciom

	aState->outputs[1] = tppz[0].Tap_select[0].Tap_up;
	aState->outputs[2] = tppz[0].Tap_select[0].Tap_down;
	aState->outputs[3] = tppz[0].Tap_select[1].Tap_up;
	aState->outputs[4] = tppz[0].Tap_select[1].Tap_down;
	aState->outputs[5] = tppz[0].Tap_select[2].Tap_up;
	aState->outputs[6] = tppz[0].Tap_select[2].Tap_down;
	aState->outputs[7] = tppz[0].Tap_select[3].Tap_up;
	aState->outputs[8] = tppz[0].Tap_select[3].Tap_down;
	aState->outputs[9] = tppz[0].Tap_select[4].Tap_up;
	aState->outputs[10] = tppz[0].Tap_select[4].Tap_down;




}

// Na potrzeby symulacji - dekodowanie zadanego zaczepu w danej chwili
int ACTUAL_TAP(short int tap_set_value[], double tap_set_time[], double sim_time)
{
	int ret;

	int length;
	length = sizeof(tap_set_value) / sizeof(short int);
	for (int i = 0; i <= length; i++)
	{
		if (time_temp > tap_set_time[i]) ret = tap_set_value[i];
	}


	return ret;
}




