#include "bsp.h"
#include "bsp_leds.h"
#include "mrfi.h"
#include "radios/family1/mrfi_spi.h"
#include "driverlib/sysctl.h"
//#include "../include.h"  //Conflict
#include "../Config.h"
#include "../gpio_interrupt.h"
#include "../Timer/Timer.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

#include "utils/uartstdio.h"


volatile uint16_t x;
mrfiPacket_t packet;


#define CAN_ID_OFFSET		(10+1)			// CAN_NODE_ID: 11,12,13,...
#define CAN_MAIN_ID			20

uint16_t CAN_MSG[12][4];		// ID - DATA - XXX - XXX
uint16_t counter = 0;
volatile bool rxFlag = 0; // msg recieved flag
volatile bool errFlag = 0; // error flag
unsigned int msgData; // the message data is four bytes long which we can allocate as an int32
unsigned char *msgDataPtr = (unsigned char *)&msgData; // make a pointer to msgData so we can access individual bytes
tCANMsgObject send_msg;
tCANMsgObject msg; // the CAN msg object
unsigned char r_msgData[8]; // 8 byte buffer for rx message data

double R_OP[3];
double i_count;
double RCM[15] = {105, 5.8, 206.4, 204.3, 105.5, 206.4, 105.5, 204.5, 206.4, 5.9, 105, 206.4, 105, 105, 190};
//orgRCM = [  105;5.8;206.4;
//;;  204.3   105.5   206.4;
//;;  105.5   204.5   206.4;
//;;  5.9;105;206.4;
//;;  105;105;190 ];

// CAN interrupt handler
void CANIntHandler(void) {
	unsigned long status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE); // read interrupt status
	if(status == CAN_INT_INTID_STATUS) { // controller status interrupt
		status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
		errFlag = 1;
	} else if(status == 1) { // msg object 1
		CANIntClear(CAN0_BASE, 1); // clear interrupt
		rxFlag = 1; // set rx flag
		errFlag = 0; // clear any error flags
	}
}

void CAN_INIT(){
	// Set up CAN0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinConfigure(GPIO_PE4_CAN0RX);
	GPIOPinConfigure(GPIO_PE5_CAN0TX);
	GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
	CANInit(CAN0_BASE);
	CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);
	CANIntRegister(CAN0_BASE, CANIntHandler); // use dynamic vector table allocation
	CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
	IntEnable(INT_CAN0);
	CANEnable(CAN0_BASE);
}

int main()
{

	// Set the system clock to run at 50Mhz off PLL with external crystal as
	// reference.
	System_config();
	Timer_Init();
	config_gpio();
	WTimer_counter_config();
	Uart_RF_config();
	CAN_INIT();

	// Use ID and mask 0 to recieved messages with any CAN ID
	msg.ui32MsgID = 0;
	msg.ui32MsgIDMask = 0;
	msg.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
	msg.ui32MsgLen = 8; // allow up to 8 bytes

	// Set up send_msg object
	msgData = 0;
	send_msg.ui32MsgID = 20<<8;
	send_msg.ui32MsgIDMask = 0;
	send_msg.ui32Flags = MSG_OBJ_NO_FLAGS;
	send_msg.ui32MsgLen = sizeof(msgDataPtr);
	send_msg.pui8MsgData = msgDataPtr;

	// Load msg into CAN peripheral message object 1 so it can trigger interrupts on any matched rx messages
	CANMessageSet(CAN0_BASE, 1, &msg, MSG_OBJ_TYPE_RX);

	//start
	//config_first_interrupt();

	BSP_Init();
	MRFI_Init();
	mrfiSpiWriteReg(PA_TABLE0,0xC0);
	MRFI_WakeUp();
	MRFI_RxOn();
	packet.frame[0]=8+5;
	UARTprintf("test UART\r\n");

//	LeastSquare_initialize();
	while (1)
	{
		if(rxFlag) { // rx interrupt has occured
//			counter++;

			msg.pui8MsgData = r_msgData; // set pointer to rx buffer
			CANMessageGet(CAN0_BASE, 1, &msg, 0); // read CAN message object 1 from CAN peripheral

			rxFlag = 0;
//			if(msg.ui32Flags & MSG_OBJ_DATA_LOST) { // check msg flags for any lost messages
//				UARTprintf("CAN message loss detected\n");
//			}

//			UARTprintf("%d ", r_msgData[0]);
//			UARTprintf("%d", r_msgData[1]*256 + r_msgData[2]);
//			UARTprintf("%d", r_msgData[2]);
//			UARTprintf("%c", r_msgData[3]);
//			UARTprintf("%d", r_msgData[4]);
//			UARTprintf("\r\n");

			//Code .......
			//  Store ID - DATA


			CAN_MSG[r_msgData[0]-CAN_ID_OFFSET][0] = r_msgData[0];
			CAN_MSG[r_msgData[0]-CAN_ID_OFFSET][1] = r_msgData[1]*256 + r_msgData[2];

		}
	}
}
void MRFI_RxCompleteISR()
{
	//MRFI_Transmit(&packet, MRFI_TX_TYPE_FORCED);
	start_counter();


	 msgDataPtr[0] = 20;
	 CANMessageSet(CAN0_BASE, 2, &send_msg, MSG_OBJ_TYPE_TX); // send as msg object 1
	// UARTprintf("TX sent\r\n");
	BSP_TOGGLE_LED2();
	//Receive package start to read
}
