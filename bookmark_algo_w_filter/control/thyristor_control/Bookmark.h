#pragma once

#define SAMPLES_10ms 3020 //liczba próbek do zliczania watchdoga
#define MARGIN 2.0 //margines k¹ta do prze³¹czania
#define NO_LOAD_STATE 0.2 //Próg wartoœci pr¹du RMS, od której uznawany jest brak obci¹zenia
#define NUMBER_OF_TAPS 5
//Stany tyrystorów




/*******************************/
//Definicja obiektu PPZ
enum ON_OFF { OFF = 0, ON = 1 };
typedef enum ON_OFF Thyri_Gate_State;
struct Tap_Thyristor_State //Struktura bitowa - stan tyrystora dolnego i gornego
{
	Thyri_Gate_State Tap_up;
	Thyri_Gate_State Tap_down;
};
enum PHASE { L1 = 1, L2 = 2, L3 = 3 };
struct Tap_State //Struktura przelacznika zaczepow
{
	struct Tap_Thyristor_State Tap_select[NUMBER_OF_TAPS]; //Dla ka¿dego zaczepu struktura bitowa z dwoma tyrystorami
	enum PHASE phase; //informacja o wybranej fazie przelacznika
};
typedef struct Tap_State TPPZ;
TPPZ tppz[3];
/************************************/


enum change_types { step, jump }; //Tryb zmiany zaczepów - skokowa lub schodkowa (po kolei, po jednym)

enum Load { Resistive, Inductive, Capacitive }; //Typy obci¹¿eñ


struct Watchdog {   // Struktura Watchdoga
	long int counter;
	int error_code;
	int cnt_reset_enable;
	short int last_state_chage;
};
enum Load LOAD_DET(float thetaU, float thetaI);
short int Tap_diff(short int actual_tap_set, short int actual_tap, enum change_types change_type);

//Finalnie stwierdzi³em, ¿e algorytm g³ówny lepiej, aby nie by³ funkcj¹. £atwiej podgl¹daæ zmienne itp
//short int BOOKMARK(TPPZ* tppz, short int actual_tap_set_global, double voltage_phase, double current_phase, double actual_current, double q_error, struct Watchdog* watchdog);
void program_Watchdog(TPPZ* tppz, struct Watchdog* watchdog, short int* state_machine, short int* actual_tap, short int actual_tap_set);
int Tap_bit(TPPZ tppz, int tap, int enable);