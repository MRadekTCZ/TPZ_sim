/*
 * Bookmark.h
 *
 *  Created on: Feb 17, 2024
 *      Author: Maciek
 */

#ifndef INC_BOOKMARK_H_
#define INC_BOOKMARK_H_

#define SAMPLES_10ms 3020 // Number of samples for watchdog counting
#define MARGIN 5.0 // Angle margin for switching
#define NO_LOAD_STATE 0.2 // Threshold value of RMS current below which the load is considered absent
#define OVERCURRENT 205.0 // Maximum constant switching current
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
typedef struct TAP_INFO_FRAME
{
	unsigned char actual_tap : 4;
	unsigned char tap_up : 1;
	unsigned char tap_down : 1;
	unsigned char b1 : 1;
	unsigned char b0 : 1;
}tap_info_frame;

typedef union BYTE_u
{
	tap_info_frame byte_8;
	unsigned char byte;
}byte_frame_tap;
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


byte_frame_tap Tap_bit(TPPZ tppz, unsigned short int tap, unsigned short int phase );

//DATA EXCHANGE AND DIAGNOSTICS**************************

typedef struct Error_status_bit
{
	uint16_t UART_error : 1;
	uint16_t failed_thyristor_switching : 1;
	uint16_t grid_transient_state : 1;
	uint16_t taps_in_phases_not_equal : 1;
	uint16_t switchover_process_ongoing : 1;
	uint16_t current_idle_state : 1;
	uint16_t diagnosis_task_done: 1;
	uint16_t step_or_jump_mode: 1;
	uint16_t local_or_remote: 1;
	uint16_t automatic_or_manual: 1;
	uint16_t base_task_done: 1;
	uint16_t overcurrent: 1;
	uint16_t measurement_mode: 1;
	uint16_t tap_max: 1;
	uint16_t tap_min: 1;
	uint16_t resistive_load_warning: 1;
}Error_status;
typedef union Status_16t
{
	Error_status b_bit;
	uint16_t byte_16t;
}status_16t;

typedef struct Status_bit_8t
{
	uint16_t b7 : 1;
	uint16_t b6 : 1;
	uint16_t b5 : 1;
	uint16_t b4 : 1;
	uint16_t b3 : 1;
	uint16_t b2 : 1;
	uint16_t b1: 1;
	uint16_t b0: 1;

}status_bit_8t;
typedef union Status_8t
{
	status_bit_8t b_bit;
	uint8_t byte_8t;
}status_8t;
#endif /* INC_BOOKMARK_H_ */
