/*
 * Bookmark.h
 *
 *  Created on: Feb 17, 2024
 *      Author: Maciek
 */

#ifndef INC_BOOKMARK_H_
#define INC_BOOKMARK_H_

#define SAMPLES_10ms 3020 // Number of samples for watchdog counting
#define MARGIN 2.0 // Angle margin for switching
#define NO_LOAD_STATE 0.2 // Threshold value of RMS current below which the load is considered absent
#define NUMBER_OF_TAPS 16
// Thyristor states

/*******************************/
// Definition of the TPPZ object
enum ON_OFF { OFF = 0, ON = 1 };
typedef enum ON_OFF Thyri_Gate_State;
struct Tap_Thyristor_State // Bit structure - state of upper and lower thyristors
{
    Thyri_Gate_State Tap_up;
    Thyri_Gate_State Tap_down;
};
enum PHASE { L1 = 1, L2 = 2, L3 = 3 };
struct Tap_State // Switch object structure
{
    struct Tap_Thyristor_State Tap_select[NUMBER_OF_TAPS]; // For each tap, a bit structure with two thyristors
    enum PHASE phase; // Information about the selected switch phase
};
typedef struct Tap_State TPPZ;

/************************************/

enum change_types { step, jump }; // Tap change mode - step or jump (one by one)
enum Load { Resistive, Inductive, Capacitive }; // Types of loads

struct Watchdog {   // Watchdog structure
    long int counter;
    int error_code;
    int cnt_reset_enable;
    short int last_state_chage;
};
enum Load LOAD_DET(float thetaU, float thetaI);
short int Tap_diff(short int actual_tap_set, short int actual_tap, enum change_types change_type);

// Ultimately, I decided that the main algorithm is better not to be a function. Easier to monitor variables, etc.
// short int BOOKMARK(TPPZ* tppz, short int actual_tap_set_global, double voltage_phase, double current_phase, double actual_current, double q_error, struct Watchdog* watchdog);
void program_Watchdog(TPPZ* tppz, struct Watchdog* watchdog, short int* state_machine, short int* actual_tap, short int actual_tap_set);
int Tap_bit(TPPZ tppz, int tap, int enable);

#endif /* INC_BOOKMARK_H_ */
