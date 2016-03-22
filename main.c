/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt
 * timing.  The maximum measured jitter time is latched in the usMaxJitter
 * variable, and displayed on the LCD by the 'Check' as described below.
 * The fast interrupt is configured and handled in the timer_test.c source
 * file.
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the LCD directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting
 * for messages - waking and displaying the messages as they arrive.  The LCD
 * task is defined in lcd.c.
 *
 * "Check" task -  This only executes every three seconds but has the highest
 * priority so is guaranteed to get processor time.  Its main function is to
 * check that all the standard demo tasks are still operational.  Should any
 * unexpected behaviour within a demo task be discovered the 'check' task will
 * write "FAIL #n" to the LCD (via the LCD task).  If all the demo tasks are
 * executing with their expected behaviour then the check task writes the max
 * jitter time to the LCD (again via the LCD task), as described above.
 */

/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

#include "uart.h"

static char buf[20];

/* Setup Configuration For ET-BASE dsPIC30F4011 */
_FOSC(CSW_FSCM_OFF & XT_PLL16);								// Disable Clock Switching,Enable Fail-Salf Clock
                                                            // Clock Source = Primary XT + (PLL x 16)
_FWDT(WDT_OFF);												// Disable Watchdog 
_FBORPOR(PBOR_OFF & PWRT_64 & MCLR_EN);					    // Disable Brown-Out ,Power ON = 64mS,Enable MCLR
_FGS(CODE_PROT_OFF);										// Code Protect OFF
/* End Configuration For ET-BASE dsPIC30F4011 */

volatile int Cnt_A, Cnt_B;    

/* The check task may require a bit more stack as it calls sprintf(). */
#define mainCHECK_TAKS_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )

/* The execution period of the check task. */
#define mainCHECK_TASK_PERIOD				( ( TickType_t ) 3000 / portTICK_PERIOD_MS )

/* The number of flash co-routines to create. */
#define mainNUM_FLASH_COROUTINES			( 5 )

/* The frequency at which the "fast interrupt test" interrupt will occur. */
#define mainTEST_INTERRUPT_FREQUENCY		( 20000 )

/* The number of processor clocks we expect to occur between each "fast
interrupt test" interrupt. */
#define mainEXPECTED_CLOCKS_BETWEEN_INTERRUPTS ( configCPU_CLOCK_HZ / mainTEST_INTERRUPT_FREQUENCY )

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ( ( unsigned short ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Dimension the buffer used to hold the value of the maximum jitter time when
it is converted to a string. */
#define mainMAX_STRING_LENGTH				( 20 )


void init_uart(void);

static void prvSetupHardware( void );


static void Task_A( void *pvParameters )  
{     
   portTickType xLastWakeTime = xTaskGetTickCount ();  

   for( ;; )  
   {     
      vTaskDelayUntil( &xLastWakeTime, 100 );  
      Cnt_A += 1;    
		
   }        
} 


static void Task_B( void *pvParameters )  
{     
const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

   Cnt_B = 0;  
   for( ;; )  
   {     
      vTaskDelay(xDelay );  
      LATBbits.LATB0 ^= 1;   
   }    
} 
static void Task_C( void *pvParameters )  
{     
const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

 portTickType xLastWakeTime = xTaskGetTickCount ();  

   for( ;; )  
   {     
       vTaskDelay(xDelay );   
	
      sprintf(buf,"test %d\r\n",Cnt_A);  
	  putsUART2(buf);   
   }    
}

	

int main( void )
{
	prvSetupHardware();

   xTaskCreate( Task_A, ( signed char * ) "Task_A", 50, NULL, 1, NULL );  
   xTaskCreate( Task_B, ( signed char * ) "Task_B", 50, NULL, 2, NULL );  
   xTaskCreate( Task_C, ( signed char * ) "Task_C", 150, NULL, 3, NULL );  

   vTaskStartScheduler(); 
 
	return 0;
}

static void prvSetupHardware( void )
{
	// Inititial hardware
	TRISBbits.TRISB0 = 0;
	init_uart();
    Cnt_A = 0;  
}


void vApplicationIdleHook( void )
{
	/* Schedule the co-routines from within the idle task hook. */
	vCoRoutineSchedule();
}
/*-----------------------------------------------------------*/

void init_uart(void)
{		  
  CloseUART2();												// Disable UART1 Before New Config

   // Config UART1 Interrupt ontrol
  ConfigIntUART2(UART_RX_INT_DIS &							// ENABLE RX Interrupt
    		     UART_RX_INT_PR2 &							// RX Interrupt Priority = 2
    		     UART_TX_INT_DIS &
    		     UART_TX_INT_PR3 );
    		     
										// ET-BASE dsPIC30F4011 UART Baudrate = 9600 BPS = 191 //  Buad Rate 19200 = 95  ,, Buadrate 38400 = 47 ,  Buadrate 115200 = 15
  			
  			
  			// Open UART1 = Mode,Status,Baudrate              
  OpenUART2(UART_EN	&										// Enable UART(UART Mode)
            UART_IDLE_STOP &								// Disable UART in IDLE Mode 
 			UART_ALTRX_ALTTX & 								// Select U1TX=RC13,U1RX=RC14
            UART_DIS_WAKE &									// Disable Wake-Up
			UART_DIS_LOOPBACK &								// Disable Loop Back
			UART_DIS_ABAUD &								// Disable Auto Baudrate
  			UART_NO_PAR_8BIT &								// UART = 8 Bit, No Parity
 			UART_1STOPBIT,									// UART = 1 Stop Bit

	  		// Config UART1 Status
  			UART_INT_TX & 									// Select Interrupt After TX Complete
	 		UART_TX_PIN_NORMAL &							// Normal U1TX Mode
 			UART_TX_ENABLE &								// Enable U1TX
 	 		UART_INT_RX_CHAR &							// Flasg Set After RX Complete 
  			UART_ADR_DETECT_DIS &              				// Disable Check Address 
			UART_RX_OVERRUN_CLEAR,							// Clear Overrun Flag

  			// ET-BASE dsPIC30F4011 Hardware Board
  			// XTAL = 7.3728MHz
  			// Fosc = 7.3728 MHz x 16 = 117.9648 MHz
  			// Fcy(UART) = Fosc / 4 
  			//           = 117.9648 / 4 = 29.4912 MHz
  			// U1BRG = [Fcy/(16xBaud)]-1
  			//       = [29.4912 MHz / (16x9600)] - 1
  			//       = 191 = BFH			
  			191);											// ET-BASE dsPIC30F4011 UART Baudrate = 9600 BPS = 191 //  Buad Rate 19200 = 95  ,, Buadrate 38400 = 47  , Buadrate 115200 = 15
}


