#include "include.h"
#include "gpio_interrupt.h"
#include "Config.h"
#include "Bluetooth/uartstdio.h"

#define MAX_CAN_NODE			30		// CAN NETWORK
#define ACTIVE_CAN_NODE			15
#define MAX_TIME_SEND_DATA 		45 		// 20Hz, 50ms

static TIMER_ID timer = INVALID_TIMER_ID;
extern uint16_t CAN_MSG[MAX_CAN_NODE][4];		// ID - DATA - XXX - XXX
uint16_t i;

void TranferData_Isr(void){
//	UARTprintf("s%03d,%03d,%03d,%03d,%03d,s%03d,%03d,%03d,%03d,%03d\r\n",CAN_MSG[0][1],CAN_MSG[1][1],CAN_MSG[2][1],CAN_MSG[3][1],CAN_MSG[4][1],CAN_MSG[5][1],CAN_MSG[6][1],CAN_MSG[7][1],CAN_MSG[8][1],CAN_MSG[9][1]);
	UARTprintf("s");
	for(i=0; i<ACTIVE_CAN_NODE-1; i++){
		UARTprintf("%03d,",CAN_MSG[i][1]);
	}
	UARTprintf("%03d\r\n",CAN_MSG[ACTIVE_CAN_NODE-1][1]);

	if(timer != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(timer);
	timer = INVALID_TIMER_ID;
}

void start_counter(void){
	clear_all();
	timer = TIMER_RegisterEvent(&TranferData_Isr, MAX_TIME_SEND_DATA);
}

void clear_all(void){
	for(i = 0; i < MAX_CAN_NODE; i++){
		CAN_MSG[i][0] = 0;
		CAN_MSG[i][1] = 0;
	}
}
