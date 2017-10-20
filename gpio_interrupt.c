/*
 * gpio_interrupt.c
 *
 *  Created on: 22-09-2015
 *      Author: Sang
 */


/* Update 23/9
 * Remove interrupt port B - conflict CC1101
 * Remove Port E.0 - ..
 * Remove trigger_isr
 * ......
 */
/* Update 10/12
 *
 */

#include "include.h"
#include "gpio_interrupt.h"
#include "Config.h"
#include "Bluetooth/uartstdio.h"

#define CAN_MAX_NODE			12
#define MAX_RECEIVER 12  // maximum 12 module
#define MAX_TIME_SEND_DATA 25 //20ms --> max distance = 7m

static bool FIRST_ISR_FLAG = false;

#define 	FIRST_INTERRUPT_PORT 		GPIO_PORTE_BASE
#define 	FIRST_INTERRUPT_PORT_NUM 	GPIO_PIN_1


static uint32_t MODULE[][4] = {
		{GPIO_PORTA_BASE, GPIO_PIN_5, 0, 0},   //PORT - PIN - FLAG - VALUE
		{GPIO_PORTA_BASE, GPIO_PIN_6, 0, 0},
		{GPIO_PORTA_BASE, GPIO_PIN_7, 0, 0},
		{GPIO_PORTD_BASE, GPIO_PIN_0, 0, 0},
		{GPIO_PORTD_BASE, GPIO_PIN_1, 0, 0},
		{GPIO_PORTD_BASE, GPIO_PIN_2, 0, 0},
		{GPIO_PORTD_BASE, GPIO_PIN_3, 0, 0},
		{GPIO_PORTE_BASE, GPIO_PIN_1, 0, 0},
		{GPIO_PORTE_BASE, GPIO_PIN_2, 0, 0},
		{GPIO_PORTE_BASE, GPIO_PIN_3, 0, 0},
		{GPIO_PORTD_BASE, GPIO_PIN_7, 0, 0},
		{GPIO_PORTD_BASE, GPIO_PIN_6, 0, 0},
};

uint8_t data[50];
uint16_t len;
static TIMER_ID timer = INVALID_TIMER_ID;

extern uint16_t CAN_MSG[12][4];		// ID - DATA - XXX - XXX

void TriggerIsr(void);
void PortA_Isr(void);
void PortD_Isr(void);
void PortE_Isr(void);
void TranferData_Isr(void);

void TranferData_Isr(void)
{
//	UARTprintf("%d\r\n",CAN_MSG[4][1]);

	UARTprintf("%d,%d,%d,%d,%d\r\n",CAN_MSG[0][1],CAN_MSG[1][1],CAN_MSG[2][1],CAN_MSG[3][1],CAN_MSG[4][1]);

	if(timer != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(timer);
	timer = INVALID_TIMER_ID;
}
void PortA_Isr(void)
{
	if ((GPIOIntStatus(GPIO_PORTA_BASE, false) & GPIO_PIN_5) && (MODULE[0][2] == 1)) {
		GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_5);
		GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);
		MODULE[0][2] = 2;
		MODULE[0][3] = TimerValueGet(WTIMER0_BASE, TIMER_A);

	}else if((GPIOIntStatus(GPIO_PORTA_BASE, false) & GPIO_PIN_6) && (MODULE[1][2] == 1)) {
		GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_6);
		GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
		MODULE[1][2] = 2;
		MODULE[1][3] = TimerValueGet(WTIMER0_BASE, TIMER_A);
	}
	else if((GPIOIntStatus(GPIO_PORTA_BASE, false) & GPIO_PIN_7) && (MODULE[2][2] == 1)) {
		GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_7);
		GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);
		MODULE[2][2] = 2;
		MODULE[2][3] = TimerValueGet(WTIMER0_BASE, TIMER_A);
	}
}

void PortD_Isr(void)
{
	if ((GPIOIntStatus(GPIO_PORTD_BASE, false) & GPIO_PIN_0) && (MODULE[3][2] == 1)) {
		GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_0);
		GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_0);
		MODULE[3][2] = 2;
		MODULE[3][3] = TimerValueGet(WTIMER0_BASE, TIMER_A);

	}else if((GPIOIntStatus(GPIO_PORTD_BASE, false) & GPIO_PIN_1) && (MODULE[4][2] == 1)) {
		GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_1);
		GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_1);
		MODULE[4][2] = 2;
		MODULE[4][3] = TimerValueGet(WTIMER0_BASE, TIMER_A);

	}else if((GPIOIntStatus(GPIO_PORTD_BASE, false) & GPIO_PIN_2) && (MODULE[5][2] == 1)) {
		GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_2);
		GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_2);
		MODULE[5][2] = 2;
		MODULE[5][3] = TimerValueGet(WTIMER0_BASE, TIMER_A);

	}else if((GPIOIntStatus(GPIO_PORTD_BASE, false) & GPIO_PIN_3) && (MODULE[6][2] == 1)) {
		GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_3);
		GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_3);
		MODULE[6][2] = 2;
		MODULE[6][3] = TimerValueGet(WTIMER0_BASE, TIMER_A);

	}else if((GPIOIntStatus(GPIO_PORTD_BASE, false) & GPIO_PIN_7) && (MODULE[10][2] == 1)) {
		GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_7);
		GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_7);
		MODULE[10][2] = 2;
		MODULE[10][3] = TimerValueGet(WTIMER0_BASE, TIMER_A);

	}else if((GPIOIntStatus(GPIO_PORTD_BASE, false) & GPIO_PIN_6) && (MODULE[11][2] == 1)) {
		GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_6);
		GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_6);
		MODULE[11][2] = 2;
		MODULE[11][3] = TimerValueGet(WTIMER0_BASE, TIMER_A);
	}
}

void PortE_Isr(void)
{
	if ((GPIOIntStatus(GPIO_PORTE_BASE, false) & GPIO_PIN_1) && (MODULE[7][2] == 1)) {
		GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_1);
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);
		MODULE[7][2] = 2;
		MODULE[7][3] = TimerValueGet(WTIMER0_BASE, TIMER_A);

	}else if((GPIOIntStatus(GPIO_PORTE_BASE, false) & GPIO_PIN_2) && (MODULE[8][2] == 1)) {
		GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_2);
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);
		MODULE[8][2] = 2;
		MODULE[8][3] = TimerValueGet(WTIMER0_BASE, TIMER_A);
	}
	else if((GPIOIntStatus(GPIO_PORTE_BASE, false) & GPIO_PIN_3) && (MODULE[9][2] == 1)) {
		GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_3);
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_3);
		MODULE[9][2] = 2;
		MODULE[9][3] = TimerValueGet(WTIMER0_BASE, TIMER_A);
	}
}

void TriggerIsr(void)
{
	 if ((GPIOIntStatus(GPIO_PORTE_BASE, false) & GPIO_PIN_1) && (!FIRST_ISR_FLAG)) {
		 	 	FIRST_ISR_FLAG = true;
	    	    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);
	    	    GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_1);
	    	    main_receiver_config();
	    	    timer = TIMER_RegisterEvent(&TranferData_Isr, MAX_TIME_SEND_DATA);
	    	    WTimer_counter_config();
	    	    WTimer_counter_clear();
	    }
}


void start_counter(void)
{
	clear_all();
//	main_receiver_config();

	timer = TIMER_RegisterEvent(&TranferData_Isr, MAX_TIME_SEND_DATA);

	//	WTimer_counter_config();
//	WTimer_counter_clear();
}

void config_gpio(void)
{
	//for Port A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);        // Enable port A
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE,  GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);  // Init as input
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 ,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor

	//for Port D
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);        // Enable port D
	//*** Unlock GPIO commit control register to use PD7 (SW1)
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
		HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
		HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
		HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);  // Init as input
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7 ,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor

	//for Port E
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);        // Enable port A
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE,  GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);  // Init as input
	GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 ,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor
}

void config_first_interrupt(void)
{
	clear_all();
	FIRST_ISR_FLAG = false;
    GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_1);        // Disable interrupt for PF4 (in case it was enabled)
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);      // Clear pending interrupts for PF4
    GPIOIntRegister(GPIO_PORTE_BASE, &TriggerIsr);     // Register our handler function for port F
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_1,GPIO_FALLING_EDGE);             // Configure PF4 for falling edge trigger
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_1);     // Enable interrupt for PF4
}

void main_receiver_config()
{
	int i;
	for(i = 0; i < MAX_RECEIVER; i++)
	{
		MODULE[i][2] = 1; //enable this port
		MODULE[i][3] = 0;
		config_receiver(i);
	}
	GPIOIntRegister(GPIO_PORTA_BASE, &PortA_Isr);
	GPIOIntRegister(GPIO_PORTD_BASE, &PortD_Isr);
	GPIOIntRegister(GPIO_PORTE_BASE, &PortE_Isr);
	for(i = 0; i < MAX_RECEIVER; i++)
	{
		enable_receiver(i);
	}
}

void config_receiver(int num)
{
	GPIOIntDisable(MODULE[num][0], MODULE[num][1]);        // Disable interrupt for PF4 (in case it was enabled)
	GPIOIntClear(MODULE[num][0], MODULE[num][1]);      // Clear pending interrupts for PF4
	GPIOIntTypeSet(MODULE[num][0], MODULE[num][1],GPIO_FALLING_EDGE);             // Configure PF4 for falling edge trigger
}

void enable_receiver(int num)
{
	GPIOIntEnable(MODULE[num][0], MODULE[num][1]);
}

void clear_all(void)
{

	int i;
//	for(i = 0; i < MAX_RECEIVER; i++)
//	{
//		MODULE[i][2] = 0; //enable this port
//		MODULE[i][3] = 0;
//	}

	for(i = 0; i < CAN_MAX_NODE; i++)
	{
		CAN_MSG[i][0] = 0;
		CAN_MSG[i][1] = 0;
	}

}
