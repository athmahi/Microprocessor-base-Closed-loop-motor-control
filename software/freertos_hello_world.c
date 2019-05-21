/*
* @file software.c
* @author Atharva Mahindrakar
* @author Prasanna Kulkarni
* @copyright Portland State University 2019
*
* This C file implments the PID control loop on Nexys4DDR4 board using FreeRTOS
*
* The flow of execution is as follows:
* First the threads and messaging queues are created for input, display & PID functionalities. hardware peripherals are
* are initialised and WDT timer interrupt is enabled. The schedular gets started when all initialisation processes are done.
* The input thread is used to get PID inputs and target RPM from user with the help of rotary encoder, switches and push 
* buttons. Message queues are sent from input thread to display thread to display PID parameters and the detected encoder
* result of the motor on Oled display. In PID thread, pid handler function is used to perform PID calculations. The reference
* of the declared struct is parsed to this function to get the PID parameters data. The returned PWM value is then assigned
* to motors.
*/


/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
/* Std includes. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/***************************** Include Files *********************************/
/* Project Two includes. */
#include "microblaze_sleep.h"
#include "nexys4IO.h"
#include "PmodOLEDrgb.h"
#include "PmodENC.h"
#include "xintc.h"
#include "stdbool.h"
#include "math.h"
#include "xil_types.h"
#include "xstatus.h"
#include "xparameters.h"
#include "xtmrctr.h"
#include "Pmod_HB3.h"
#include "xwdttb.h"

#define mainQUEUE_LENGTH					( 1 )

/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK						( portTickType ) 0


// AXI timer parameters 1 : Nexys4IO
#define AXI_TIMER1_DEVICE_ID	XPAR_AXI_TIMER_1_DEVICE_ID
#define AXI_TIMER1_BASEADDR		XPAR_AXI_TIMER_1_BASEADDR
#define AXI_TIMER1_HIGHADDR		XPAR_AXI_TIMER_1_HIGHADDR
#define TmrCtrNumber1			0

// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_HIGHADDR
#define PMODHB3_BASEADDR		0x44A10000

#define INTC_DEVICE_ID				XPAR_INTC_0_DEVICE_ID



/* The threads/tasks as described at the top of this file. */
//static void master_thread( void *pvParameters );

static void input_thread( void *pvParameters );
static void display_thread( void *pvParameters );
static void pid_thread( void *pvParameters );

static void watch_handler (void *pvUnused);

/* The queue used by the queue send and queue receive tasks. */
//static xQueueHandle Display_RPM_Setpoint = 0;
static xQueueHandle Display_RPM = 0;
static xQueueHandle Display_Kp  = 0;
static xQueueHandle Display_Ki  = 0;
static xQueueHandle Display_Kd  = 0;
//static xQueueHandle xQueueInput_PID	 = 0;


/**
	Structure defination to store PID parameters so that they could be accessed in multiple 
	threads
*/

typedef struct {
	u8 Kp;
	u8 Ki;
	u8 Kd;
	u16 current_rpm;
	u16 target_rpm;
	double error;
	double prev_error;
	double integral;
	double derivative;

}PID_control;




PmodOLEDrgb	pmodOLEDrgb_inst;			// PmodOLED instance
PmodENC 	pmodENC_inst;				// PMOD ENCinstance

XTmrCtr		AXITimerInst;				// AXI timer instance
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance


xSemaphoreHandle bin_sem = 0;				// semaphore declaration

PID_control PID, Input;					// PID instance

XWdtTb watch_dog;						//Watch dog timer instance

int main(void)
{

	uint32_t sts;


	sts = do_init();					// hardware initialisation
	if (XST_SUCCESS != sts)
	{
		exit(1);
	}

	xil_printf("Hello from FreeRTOS Example\r\n");
	init_display();

	
	Display_RPM	= xQueueCreate( mainQUEUE_LENGTH, sizeof( int ) );		// Queue declaration
	Display_Kp  = xQueueCreate( mainQUEUE_LENGTH, sizeof( u8 ) );
	Display_Ki  = xQueueCreate( mainQUEUE_LENGTH, sizeof( u8 ) );
	Display_Kd  = xQueueCreate( mainQUEUE_LENGTH, sizeof( u8 ) );
	
	
	configASSERT(Display_RPM);			// config assert for queues
	configASSERT( Display_Kp );
	configASSERT( Display_Ki );
	configASSERT( Display_Kd );

										//Create Semaphore
	vSemaphoreCreateBinary( bin_sem );

	xTaskCreate( input_thread,				// initialise thread input_thread
		   ( const char * ) "IT",
							2048,
							NULL,
							1,				// priority 1
							NULL );

	xTaskCreate( display_thread,			// initialise display input_thread
		   ( const char * ) "DT",
							2048,
							NULL,
							2,				// priority 2
							NULL );

	xTaskCreate( pid_thread,				// initialise thread pid_thread
		   ( const char * ) "PT",
							2048,
							NULL,
							1,				// priority1
							NULL );
	NX4IO_setLEDs(0x0);						// clear LEDs

	xil_printf("starting scheduler **********\n");
	vTaskStartScheduler();
	xil_printf("Ending scheduler **********\n");
	return 0;
}



/**
*	In this thread inputs from switches, PmodENC & push buttons
*	are read and accordingly the increment / decrement operation
*	on input PID parameters is perfomed
*	Also from our custom PmodHB3 driver,  encoder input is read
*	and used forclose  loop control.
*/


void input_thread( void *pvParameters ) {

	u32 state,laststate;
	int ticks = 0, lastticks = 0;
	u8 tempKp = 0;
	u8 tempKi = 0;
	u8 tempKd = 0;
	u16 SWITCH_IN = 0;

	u16 pid_incr = 0;

	u16 rpm_incr = 0;

	u16 pid_param = 0;
	u32 tempTargetRPM = 0;
	u32 RPM_detect = 0;

	int error = 0;
	int prev_error = 0;
	int differential = 0;
	int integral = 0;

	int pid_out = 0;

	u16 enc_switch = 0;
	u16 enc_switch_state = 0;
	u16 enc_switch_laststate = 0;


	while(1)
	{

		
		SWITCH_IN = NX4IO_getSwitches();	// read switch input

		NX4IO_setLEDs(SWITCH_IN);			// set LEDs depending upon input Switch data



		if((SWITCH_IN & 0X0030) == 0X00)		// if SW[5:4] == 0
		{
			pid_incr = 1;						// sclale is 1
		}

		else if((SWITCH_IN & 0X0030) == 0x0010)	// if SW[5:4] == 0x01
		{
			pid_incr = 5;						// scale is 5
		}

		else if ((SWITCH_IN & 0X0030) >= 0x0020)	// if SW[5:4] == 0x1x
		{
			pid_incr = 10;							// scale is 5
		}



		if((SWITCH_IN & 0x000C)==(0x0008|0x000C))
		{      											//SW[3:2] = 0x1x
			pid_param = 1;        						//update Kp
		}
		else if((SWITCH_IN & 0x000C)==0x0004)			//SW[3:2] = 0x01
		{         										
			pid_param = 2;           					//Update  ki
		}
		else if((SWITCH_IN & 0x000C)==0x0000)			//SW[3:2] = 0x1x
		{          										
			pid_param = 3;        						//Update kd
		}



		if((SWITCH_IN & 0X0003) == 0X00)				// SW[3:2] = 0
		{
			rpm_incr = 1;								// incr = 1
		}

		else if((SWITCH_IN & 0X0003) == 0x0001)			// SW[3:2] = 0x01
		{
			rpm_incr = 5;								// incr = 5
		}

		else if ((SWITCH_IN & 0X0003) >= 0x0002)		// SW[3:2] = 0x1x
		{
			rpm_incr = 10;								// incr = 10
		}

		state = ENC_getState(&pmodENC_inst);			// ENC encoder input

		ticks = ENC_getRotation(state, laststate);

		enc_switch_state = ENC_switchOn(state);


		if(ticks == 1)								
		{
			PID.target_rpm = PID.target_rpm + rpm_incr;						// increase the speed

			if(PID.target_rpm < 0)
				PID.target_rpm = -(PID.target_rpm);

			if(PID.target_rpm > 999)
				PID.target_rpm = 0;
		}
		else if(ticks == -1)							//If new ticks is less than old ticks
		{
			PID.target_rpm = PID.target_rpm - rpm_incr;						//Decrease the speed

			if(PID.target_rpm < 0)
				PID.target_rpm = -(PID.target_rpm);

			if(PID.target_rpm > 999)
				PID.target_rpm = 0;
		}


		if (NX4IO_isPressed(BTNU))						// increment PID parameters
		{
			if(pid_param==1)
				PID.Kp = PID.Kp + pid_incr;

			if(pid_param==2)
				PID.Ki = PID.Ki + pid_incr;

			if(pid_param==3)
				PID.Kd = PID.Kd + pid_incr;

		}

		if (NX4IO_isPressed(BTND))						// Decrement PID parameters
		{
			if(pid_param==1)
				PID.Kp = PID.Kp - pid_incr;

			if(pid_param==2)
				PID.Ki = PID.Ki - pid_incr;

			if(pid_param==3)
				PID.Kd = PID.Kd - pid_incr;

		}

		if (NX4IO_isPressed(BTNC))						// resets PID parameters
		{
			PID.Kp = 0;								
			PID.Ki = 0;
			PID.Kd = 0;
			PID.target_rpm = 0;
			PMOD_HB3_mWriteReg(PMODHB3_BASEADDR, 12, 0);
		}

		NX4IO_SSEG_putU32Dec(PID.target_rpm, 1);

		tempTargetRPM = PID.target_rpm * 2047/999;						//scale the RPM to 10 bit value 

		//PMOD_HB3_mWriteReg(PMODHB3_BASEADDR, 4, 0);					// give 
		//PMOD_HB3_mWriteReg(PMODHB3_BASEADDR, 12, tempTargetRPM);
		RPM_detect = PMOD_HB3_mReadReg(PMODHB3_BASEADDR, 8);			// read encoder input from	


		PID.current_rpm = RPM_detect;

		if(!xQueueSend( Display_Kp, &PID.Kp, portMAX_DELAY ))			// send PID data to display thread
		{
			xil_printf("Failed to send message to DISPLAY THREAD \n");
		}

		if(!xQueueSend( Display_Ki, &PID.Ki, portMAX_DELAY ))			// send PID data to display thread
		{
			xil_printf("Failed to send message to DISPLAY THREAD \n");
		}

		if(!xQueueSend( Display_Kd, &PID.Kd, portMAX_DELAY ))			// send PID data to display thread
		{
			xil_printf("Failed to send message to DISPLAY THREAD \n");
		}

		if(!xQueueSend( Display_RPM, &RPM_detect, portMAX_DELAY ))		// send PID data to display thread
		{
			xil_printf("Failed to send message to DISPLAY THREAD \n");
		}



		if(enc_switch_state != enc_switch_laststate)					// if enc switch is flipped
		{
			PMOD_HB3_mWriteReg(PMODHB3_BASEADDR, 12, 0);				// switch off the motor
			//xil_printf("\n\n ENC switch flipped\n\n");
		}

		if(enc_switch_state == 1 )								// if enc switchis on
			PMOD_HB3_mWriteReg(PMODHB3_BASEADDR, 4, 0);			// rotate motor clockwise
		else if(enc_switch_state == 0 )							// if enc switch if off
			PMOD_HB3_mWriteReg(PMODHB3_BASEADDR, 4, 1);			// rotate motor anti clockwise

		laststate = state;
		lastticks = ticks;

		enc_switch_laststate = enc_switch_state;

		vTaskDelay(3);

	}
}

/**
	In display thread, simply data sent from input thread is displayed over PmodOled display
*/

void display_thread( void *pvParameters )
{
	u8 tempKp = 0;
	u8 tempKi = 0;
	u8 tempKd = 0;
	u32 tempRPM_detect = 0;
	u32 temprprp = 0;

	while(1)
	{

		//xil_printf(" In thread display_thread \n");
		if( xQueueReceive( Display_Kp, &tempKp, mainDONT_BLOCK ) )
		{
			//xil_printf("data recetved of Kp - %d \n", tempKp);
		}

		if( xQueueReceive( Display_Ki, &tempKi, mainDONT_BLOCK ) )
		{
			//xil_printf("data recetved of Ki - %d \n", tempKi);
		}

		if( xQueueReceive( Display_Kd, &tempKd, mainDONT_BLOCK ) )
		{
			//xil_printf("data recetved of Kd - %d \n", tempKd);
		}

		if( xQueueReceive( Display_RPM, &tempRPM_detect, mainDONT_BLOCK ) )
		{
			//xil_printf("         data recetved of RPM - %d \n", tempRPM_detect);
		}

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 1);				// Display the kp data
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 1);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempKp, 10);

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 3);				// Display the Ki data
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 3);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempKi, 10);

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 5);				// Display the Kd data
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 5);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempKd, 10);


		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 7);				// Display detected RPM
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 7);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempRPM_detect, 10);

		/*if(tempRPM_detect == 0)
			temprprp = tempRPM_detect;
		else
			temprprp = tempRPM_detect + 30;

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 7);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 7);
		PMDIO_putnum(&pmodOLEDrgb_inst, temprprp, 10);*/

		vTaskDelay(3);

	}
}

/**
    In this thread waits untill all the PID parameters are zero. if one of the 
    parameter changes then it calls PID handler function to which an reference
    of the PID struct is sent. That function returns the target pwm value at which motor
    shou;d run. That value is then writen on the PmodHB3's register which generates 
    PWM

*/

void pid_thread( void *pvParameters )
{

	int target_speed = 0;
	int pid_speed = 0;
	while(1)
	{
		target_speed = PID.target_rpm;

		if(PID.Kd == 0 && PID.Ki == 0 && PID.Kp == 0)
		{
			// Do nothing yet
		}

		else
		{
			pid_speed = pid_handler(&PID, target_speed);                // get the target PWM value
			PMOD_HB3_mWriteReg(PMODHB3_BASEADDR, 12, pid_speed);        // write the value to Pmod IP
		}

		vTaskDelay(5);

	}
}

int pid_handler (PIDControl *pid, int target_speed)
{
	int i =0;
	int out_pwm = 0;

	int offset = 0;
	double error = 0;
	double tempKp = 0;
	double tempKi = 0;
	double tempKd = 0;

	int current_rpm_dummy = 0;

	//pid->target_rpm = target_speed;

	error = target_speed - pid->current_rpm ;           // calculate error

	pid->derivative = error - pid->prev_error;          // calculate derivative




	pid->integral = pid->integral + error;              // calculate the integral

	pid->error = error;                                 // assign current error to previous error


	if(pid->integral > 1075)                            // bound the integral error to -1075 to 1075
		pid->integral = 1075;

	if(pid->integral <-1075)
		pid->integral = -1075;

	tempKp = (double)pid->Kp / (double)255;             // scale down the PID parameters from 8 bits to
	tempKi = (double)pid->Ki / (double)255;             // floating 0-1 scale
	tempKd = (double)pid->Kd / (double)255;

    // calculate target PWM
	out_pwm = offset + (int) (( tempKp *1.4*pid->error ) + (tempKi * pid->integral ) + ( tempKd  * pid->derivative ));




	if (out_pwm < 0)
		out_pwm = -(out_pwm);                          // handle -ve results

	current_rpm_dummy = pid->current_rpm;

	xil_printf("%d,%d\n",target_speed, current_rpm_dummy);

	pid->prev_error = pid->error;                       // assign the current error to previous error

	return out_pwm;                                        // return the calculated PWM

}




/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion, 
*
* @return  *NONE*
*
* @note
* No size check is done on the return string size.  Make sure you leave room
* for the full string plus the terminating null in string
*****************************************************************************/

void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}

  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;

	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;

  	return;
}


void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];

  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}


/****************************************************************************/
/**
* initialize the system
*
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/


int	 do_init(void)
{
	uint32_t status, status_1, status_2, status_3 ;				// status from Xilinx Lib calls

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);       // initialise NX4IO
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);    // initialise pmodOled display

	// initialize the pmodENC and hardware
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);                 // initialise pmod ENC module

	status = AXI_Timer1_initialize();                           // initialise AXI timer 1
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	xil_printf("\n\n****** AXI timer 1 initialised\n\n****");


	XWdtTb_Config *config;                                      // instantiate watchdog timer

	config = XWdtTb_LookupConfig(XPAR_WDTTB_0_DEVICE_ID);       // configure WDT

	XWdtTb_CfgInitialize(&watch_dog, config, config->BaseAddr);     // configure WDT

	XWdtTb_ProgramWDTWidth(&watch_dog, 100);                    // set 32 bit WDT count

    // intialise the WDT handler
	xPortInstallInterruptHandler(XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR,watch_handler, NULL);

	vPortEnableInterrupt(XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR); // enable WDt interruot

	XWdtTb_Start(&watch_dog);                                   // start WDT


	return XST_SUCCESS;
}

/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */

int AXI_Timer1_initialize(void){

	uint32_t status;    // status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER1_DEVICE_ID);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber1);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER1_BASEADDR, TmrCtrNumber1,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER1_BASEADDR, TmrCtrNumber1, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER1_BASEADDR, TmrCtrNumber1);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER1_BASEADDR, TmrCtrNumber1);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER1_BASEADDR, TmrCtrNumber1, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER1_BASEADDR, TmrCtrNumber1);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER1_BASEADDR, TmrCtrNumber1, ctlsts);

	XTmrCtr_Enable(AXI_TIMER1_BASEADDR, TmrCtrNumber1);
	return XST_SUCCESS;
}

/**
    Sets the PID parameter variable names on left edge of the display
*/

void init_display()
{
	// Set up the display output
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(200, 12, 44));
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Kp:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Ki:");
	// To display 0 initially : KA
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Kd:");
	// To display 0 initially : KA
	// To show the Prof/TA whether HW/SW detection is selected
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 7);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"RPM:");

	// blank the display digits and turn off the decimal points
	//NX410_SSEG_setAllDigits(SSEGLO, CC_0, CC_0, CC_0, CC_0, DP_NONE);
	//NX410_SSEG_setAllDigits(SSEGHI, CC_0, CC_0, CC_0, CC_0, DP_NONE);
}

/**
    This handler gets called every time WDt interrupt occurs. 
*/
void watch_handler (void *pvUnused)
{
	int sw_in = 0;

	sw_in = NX4IO_getSwitches();        // reads switch inputs

	
	if(sw_in == 32768)                  // checks the condition of whether sw[15] is on or off
	{
		OLEDrgb_Clear(&pmodOLEDrgb_inst);
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
		xil_printf(" \n\n CRRASHED !!!!!!!!!!!!!\n");
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"CRASHED ! ! !");       // display crash message on Oled display
		NX4IO_setLEDs(0xff0f);                                      // sets LEDs
		vTaskEndScheduler();                                        // ends schedular
	}

	else
	{
		XWdtTb_RestartWdt(&watch_dog);              // restarts WDT
	}

	sw_in = 0;
}


