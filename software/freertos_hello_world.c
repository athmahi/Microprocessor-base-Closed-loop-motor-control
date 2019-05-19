//@author Sai Bodanki


//The application Demonstrates usage of Queues, Semaphores, Tasking model.

//Application - Glow LED[7:0] for first interrupt and Glow LED[15:8] for next interrupt --> Repeat

//Flow diagram
// GPIO Interrupt (DIP Switch) --> ( ISR )Send a Semaphore --> Task 1 (Catch the Semaphore) -->
// -->Task 1 - Send a Queue to Task -2 --> Task 2 Receive the queue --> Write to GPIO (LED)


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

/* The queue used by the queue send and queue receive tasks. */
//static xQueueHandle xQueueDisplayRPM_Setpoint = 0;
static xQueueHandle xQueueDisplayRPM = 0;
static xQueueHandle xQueueDisplayKp  = 0;
static xQueueHandle xQueueDisplayKi  = 0;
static xQueueHandle xQueueDisplayKd  = 0;
static xQueueHandle xQueueInput_PID	 = 0;


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

	u32 rpm_counter;
	//u32 led_value;			// Based on PID lit up LEDs

}PIDControl;





//Create Instances
//Function Declarations

//Declare a Sempahore
xSemaphoreHandle binary_sem;

PmodOLEDrgb	pmodOLEDrgb_inst;
PmodENC 	pmodENC_inst;

XTmrCtr		AXITimerInst;
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance


xSemaphoreHandle xPID = 0;

PIDControl PID, Input;

int main(void)
{

	uint32_t sts;


	sts = do_init();
	if (XST_SUCCESS != sts)
	{
		exit(1);
	}

	xil_printf("Hello from FreeRTOS Example\r\n");
	init_display();




	/* Create the queue */
	//xQueueDisplayRPM_Setpoint = xQueueCreate( mainQUEUE_LENGTH, sizeof( int ) );
	xQueueDisplayRPM	  = xQueueCreate( mainQUEUE_LENGTH, sizeof( int ) );
	xQueueDisplayKp  = xQueueCreate( mainQUEUE_LENGTH, sizeof( u8 ) );
	xQueueDisplayKi  = xQueueCreate( mainQUEUE_LENGTH, sizeof( u8 ) );
	xQueueDisplayKd  = xQueueCreate( mainQUEUE_LENGTH, sizeof( u8 ) );
	//xQueueInput_PID  =

	/* Sanity check that the queue was created. */
	//configASSERT( xQueueDisplayRPM_Setpoint );
	configASSERT(xQueueDisplayRPM);
	configASSERT( xQueueDisplayKp );
	configASSERT( xQueueDisplayKi );
	configASSERT( xQueueDisplayKd );

	//Create Semaphore
	vSemaphoreCreateBinary( xPID );

	xTaskCreate( input_thread,
		   ( const char * ) "IT",
							2048,
							NULL,
							1,
							NULL );

	xTaskCreate( display_thread,
		   ( const char * ) "DT",
							2048,
							NULL,
							2,
							NULL );

	xTaskCreate( pid_thread,
		   ( const char * ) "PT",
							2048,
							NULL,
							1,
							NULL );

	xil_printf("starting scheduler **********\n");
	vTaskStartScheduler();
	xil_printf("Ending scheduler **********\n");
	return 0;
}



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





	// get the previous state
	while(1)
	{

		/*xil_printf(" In thread input_thread \n");
		PID.Kp = tempKp;
		PID.Ki = tempKi;
		PID.Kd = tempKd;

		tempKp++;
		tempKi++;
		tempKd++;

		if(!xQueueSend( xQueueDisplayKp, &PID.Kp, portMAX_DELAY ))
		{
			xil_printf("Failed to send message to DISPLAY THREAD \n");
		}*/

		SWITCH_IN = NX4IO_getSwitches();	// read switch input



		if((SWITCH_IN & 0X0030) == 0X00)
		{
			pid_incr = 1;
		}

		else if((SWITCH_IN & 0X0030) == 0x0010)
		{
			pid_incr = 5;
		}

		else if ((SWITCH_IN & 0X0030) >= 0x0020)
		{
			pid_incr = 10;
		}



		if((SWITCH_IN & 0x000C)==(0x0008|0x000C))
		{      													//Check if switches[3:2] are 10 or 11
			pid_param = 1;        								//Update the value of proportional constant kp
		}
		else if((SWITCH_IN & 0x000C)==0x0004)
		{         												//Check if switches[3:2] are 01
			pid_param = 2;           							//Update the value of integral constant ki
		}
		else if((SWITCH_IN & 0x000C)==0x0000)
		{          												//Check if switches[3:2] are 00
			pid_param = 3;        								//Update the value of derivative constant kd
		}



		if((SWITCH_IN & 0X0003) == 0X00)
		{
			rpm_incr = 1;
		}

		else if((SWITCH_IN & 0X0003) == 0x0001)
		{
			rpm_incr = 5;
		}

		else if ((SWITCH_IN & 0X0003) >= 0x0002)
		{
			rpm_incr = 100;
		}

		state = ENC_getState(&pmodENC_inst);

		ticks = ENC_getRotation(state, laststate);

		if(ticks == 1)								//If new ticks is greater than old ticks
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


		if (NX4IO_isPressed(BTNU))
		{
			if(pid_param==1)
				PID.Kp = PID.Kp + pid_incr;

			if(pid_param==2)
				PID.Ki = PID.Ki + pid_incr;

			if(pid_param==3)
				PID.Kd = PID.Kd + pid_incr;

		}

		if (NX4IO_isPressed(BTND))
		{
			if(pid_param==1)
				PID.Kp = PID.Kp - pid_incr;

			if(pid_param==2)
				PID.Ki = PID.Ki - pid_incr;

			if(pid_param==3)
				PID.Kd = PID.Kd - pid_incr;

		}

		if (NX4IO_isPressed(BTNC))
		{
			PID.Kp = 0;
			PID.Ki = 0;
			PID.Kd = 0;
			PID.target_rpm = 0;
		}

		NX4IO_SSEG_putU32Dec(PID.target_rpm, 1);

		tempTargetRPM = PID.target_rpm * 2047/999;

		PMOD_HB3_mWriteReg(PMODHB3_BASEADDR, 4, 1);
		//PMOD_HB3_mWriteReg(PMODHB3_BASEADDR, 12, tempTargetRPM);
		RPM_detect = PMOD_HB3_mReadReg(PMODHB3_BASEADDR, 8);

		PID.current_rpm = RPM_detect;

		if(!xQueueSend( xQueueDisplayKp, &PID.Kp, portMAX_DELAY ))
		{
			xil_printf("Failed to send message to DISPLAY THREAD \n");
		}

		if(!xQueueSend( xQueueDisplayKi, &PID.Ki, portMAX_DELAY ))
		{
			xil_printf("Failed to send message to DISPLAY THREAD \n");
		}

		if(!xQueueSend( xQueueDisplayKd, &PID.Kd, portMAX_DELAY ))
		{
			xil_printf("Failed to send message to DISPLAY THREAD \n");
		}

		if(!xQueueSend( xQueueDisplayRPM, &RPM_detect, portMAX_DELAY ))
		{
			xil_printf("Failed to send message to DISPLAY THREAD \n");
		}


		/*error = PID.target_rpm - RPM_detect;

		differential = error - prev_error;



		if(error < (PID.target_rpm / 10))
			integral += error;
		else
			integral = 0;



		pid_out = (PID.Kp * error) + (PID.Ki * integral) + (PID.Kd * differential);

		pid_out = 200 + pid_out/100;

		if(differential != 0)
			xil_printf("\n**********************\n");


		xil_printf("\n\n\t\t PID - %d error - %d	%d \n\n", pid_out, error, integral );


		//OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 7);
		//PMDIO_putnum(&pmodOLEDrgb_inst, pid_out, 10);*/




		laststate = state;
		lastticks = ticks;
		prev_error = error;

		vTaskDelay(3);

	}
}



void display_thread( void *pvParameters )
{
	u8 tempKp = 0;
	u8 tempKi = 0;
	u8 tempKd = 0;
	u32 tempRPM_detect = 0;

	while(1)
	{

		//xil_printf(" In thread display_thread \n");
		if( xQueueReceive( xQueueDisplayKp, &tempKp, mainDONT_BLOCK ) )
		{
			//xil_printf("data recetved of Kp - %d \n", tempKp);
		}

		if( xQueueReceive( xQueueDisplayKi, &tempKi, mainDONT_BLOCK ) )
		{
			//xil_printf("data recetved of Ki - %d \n", tempKi);
		}

		if( xQueueReceive( xQueueDisplayKd, &tempKd, mainDONT_BLOCK ) )
		{
			//xil_printf("data recetved of Kd - %d \n", tempKd);
		}

		if( xQueueReceive( xQueueDisplayRPM, &tempRPM_detect, mainDONT_BLOCK ) )
		{
			//xil_printf("         data recetved of RPM - %d \n", tempRPM_detect);
		}

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 1);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 1);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempKp, 10);

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 3);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 3);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempKi, 10);

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 5);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 5);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempKd, 10);


		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 7);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 7);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempRPM_detect, 10);



		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 7);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 7);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempRPM_detect, 10);

		vTaskDelay(3);

	}
}


void pid_thread( void *pvParameters )
{

	int target_speed = 0;
	int pid_speed = 0;
	while(1)
	{
		target_speed = PID.target_rpm;
		//xil_printf(" target_rpm - %d \n\n", target_speed);

		if(PID.Kd == 0 && PID.Ki == 0 && PID.Kp == 0)
		{
			// Do nothing yet
			//pid_speed = pid_handler(&PID, target_speed);
		}

		else
		{
			//if(PID.error>= 10 || PID.error<= -10 )
				pid_speed = pid_handler(&PID, target_speed);

			//pid_speed =(int) ((double) pid_speed * (double) 1.5);

			//xil_printf("\n\n PID PWM is %d \n\n", pid_speed);

			/*if(pid_speed > 2047)
				pid_speed = 2047;*/



			PMOD_HB3_mWriteReg(PMODHB3_BASEADDR, 12, pid_speed);

			// update motor speed
			//pid_handler();
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

	//pid->target_rpm = target_speed;

	error = target_speed - pid->current_rpm ;

	pid->derivative = error - pid->prev_error;




	pid->integral = pid->integral + error;

	pid->error = error;

	//else
	//	pid->integral = 0;

	if(pid->integral > 1000)
		pid->integral = 1000;

	if(pid->integral <-1000)
		pid->integral = -1000;

	tempKp = (double)pid->Kp / (double)255;
	tempKi = (double)pid->Ki / (double)255;
	tempKd = (double)pid->Kd / (double)255;

	out_pwm = offset + (int) (( tempKp *1.4*pid->error ) + (tempKi * pid->integral ) + ( tempKd  * pid->derivative ));




	if (out_pwm < 0)
		out_pwm = -(out_pwm);
	xil_printf("\n\n PID target	%d	Current %d	Errors  %d Integra %d PID PWM %d \n\n",target_speed,pid->current_rpm, (int)pid->error,(int) pid->integral, out_pwm);

	pid->prev_error = pid->error;

	return out_pwm;

}



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
int	 do_init(void)
{
	uint32_t status, status_1, status_2, status_3 ;				// status from Xilinx Lib calls

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the pmodENC and hardware
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

	status = AXI_Timer1_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	xil_printf("\n\n****** AXI timer 1 initialised\n\n****");

	/*status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}

	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}*/


	return XST_SUCCESS;
}


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



/*void pid_thread( void  )
{
	int i = 0;
	u8 tempKp = 0;
	u8 tempKi = 0;
	u8 tempKd = 0;
	u32 temp_current_rpm = 0;
	u32 temp_target_rpm = 0;

	double temp_error;
	double temp_prev_error;
	double temp_integral;
	double temp_derivative;



		PID.error = PID.target_rpm - PID.current_rpm;

		temp_derivative = PID.error - PID.prev_error;
		PID.derivative = temp_derivative;
		PID.prev_error = PID.error;

		if(PID.error < (PID.target_rpm/10))
		{
			PID.integral = PID.integral + PID.error;
		}
		else
			PID.integral = 0;

		PID.target_rpm = (int) ((PID.Kp * PID.error) + (PID.integral * PID.integral) + (PID.Kd * PID.derivative));

		temp_target_rpm = PID.target_rpm * 2047/999;
		PMOD_HB3_mWriteReg(PMODHB3_BASEADDR, 4, 1);
		PMOD_HB3_mWriteReg(PMODHB3_BASEADDR, 12, PID.target_rpm);
		xil_printf("set PWM is - %d\n", PID.target_rpm);


}

OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 1);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 1);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempKp, 10);

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 3);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 3);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempKi, 10);

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 5);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 5);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempKd, 10);


		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 7);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"      ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 7);
		PMDIO_putnum(&pmodOLEDrgb_inst, tempRPM_detect, 10);*/
