//@author Sai Bodanki


//The application Demonstrates usage of Queues, Semaphores, Tasking model.

//Application - Glow LED[7:0] for first interrupt and Glow LED[15:8] for next interrupt --> Repeat

//Flow diagram
// GPIO Interrupt (DIP Switch) --> ( ISR )Send a Semaphore --> Task 1 (Catch the Semaphore) -->
// -->Task 1 - Send a Queue to Task -2 --> Task 2 Receive the queue --> Write to GPIO (LED)

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* BSP includes. */
#include "xtmrctr.h"
#include "xgpio.h"
#include "sleep.h"

#define mainQUEUE_LENGTH					( 1 )

/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK						( portTickType ) 0

//Create Instances
static XGpio xOutputGPIOInstance, xInputGPIOInstance;

//Function Declarations
static void prvSetupHardware( void );

//Declare a Sempahore
xSemaphoreHandle binary_sem;

/* The queue used by the queue send and queue receive tasks. */
static xQueueHandle xQueue = NULL;

//Value of LED, (1010 1010 1010 1010)
unsigned int led = 43690;

//ISR, to handle interrupt og GPIO dip switch
//Can also use Push buttons.
//Give a Semaphore
static void gpio_intr (void *pvUnused)
{
	xSemaphoreGiveFromISR(binary_sem,NULL);

	XGpio_InterruptClear( &xInputGPIOInstance, 1 );

}

//A task which takes the Interrupt Semaphore and sends a queue to task 2.
void sem_taken_que_tx (void *p)
{
	unsigned long ulValueToSend = 255UL;

	while(1)
	if(xSemaphoreTake(binary_sem,500)){
		xil_printf("Queue Sent: %d\n",ulValueToSend);
		xQueueSend( xQueue, &ulValueToSend, mainDONT_BLOCK );
		ulValueToSend = ~ulValueToSend;//Toggle for next time.
	}
	else
		xil_printf("Semaphore time out\n");
	//Could use to take rotary encoder values (A trick, coz pmodENC dosen't support interrupt.)
}

void que_rx (void *p)
{
	unsigned long ulReceivedValue;
	while(1){
		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );
		//Write to LED.
		XGpio_DiscreteWrite( &xOutputGPIOInstance, 1, ulReceivedValue );
		xil_printf("Queue Received: %d\n",ulReceivedValue);
	}
}

int main(void)
{
	//Initialize the HW
	prvSetupHardware();

	//Creatye Semaphore
	vSemaphoreCreateBinary(binary_sem);

	/* Create the queue */
	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );

	/* Sanity check that the queue was created. */
	configASSERT( xQueue );

	//Create Task1
	xTaskCreate( sem_taken_que_tx,
				 ( const char * ) "TX",
				 2048,
				 NULL,
				 1,
				 NULL );

	//Create Task2
	xTaskCreate( que_rx,
				"RX",
				2048,
				NULL,
				2,
				NULL );



	//Start the Secheduler
	vTaskStartScheduler();

	return -1;
}


static void prvSetupHardware( void )
{
portBASE_TYPE xStatus;
const unsigned char ucSetToOutput = 0U;

	/* Initialize the GPIO for the LEDs. */
	xStatus = XGpio_Initialize( &xOutputGPIOInstance, XPAR_AXI_GPIO_0_DEVICE_ID );

	if( xStatus == XST_SUCCESS )
	{
		/* All bits on this channel are going to be outputs (LEDs). */
		XGpio_SetDataDirection( &xOutputGPIOInstance, 1, ucSetToOutput );

		XGpio_DiscreteWrite( &xOutputGPIOInstance, 1, 0 );
	}

	/* Initialise the GPIO for the button inputs. */
	if( xStatus == XST_SUCCESS )
	{
		xStatus = XGpio_Initialize( &xInputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID );
	}

	if( xStatus == XST_SUCCESS )
	{
		/* Install the handler defined in this task for the button input.
		*NOTE* The FreeRTOS defined xPortInstallInterruptHandler() API function
		must be used for this purpose. */
		xStatus = xPortInstallInterruptHandler( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR, gpio_intr, NULL );


		if( xStatus == pdPASS )
		{

			/* Set buttons to input. */
			XGpio_SetDataDirection( &xInputGPIOInstance, 2, 0 );

			/* Enable the button input interrupts in the interrupt controller.
			*NOTE* The vPortEnableInterrupt() API function must be used for this
			purpose. */

			vPortEnableInterrupt( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR );

			/* Enable GPIO channel interrupts. */
			XGpio_InterruptEnable( &xInputGPIOInstance, 1 );
			XGpio_InterruptGlobalEnable( &xInputGPIOInstance );
		}
	}

	configASSERT( ( xStatus == pdPASS ) );
}

