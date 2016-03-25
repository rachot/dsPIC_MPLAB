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
#include "timers.h"

#include "uart.h"
#include "qei.h"   
#include "adc10.h"


// PID parameter
float SP,PV;
float kp;	
float ki;	
float kd;
float ee;	
float pterm;
float iterm;
float dterm;
float pid;	
float lastee;		
float sumee;

// ADC
float voltage;


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

/* The number of processor clocks we expect to occur between each "fast interrupt test" interrupt. */

#define mainEXPECTED_CLOCKS_BETWEEN_INTERRUPTS ( configCPU_CLOCK_HZ / mainTEST_INTERRUPT_FREQUENCY )

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ( ( unsigned short ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Dimension the buffer used to hold the value of the maximum jitter time when
it is converted to a string. */
#define mainMAX_STRING_LENGTH				( 20 )


void init_uart(void);
void Init_QEI(void);
static void prvSetupHardware( void );
void init_timer(void);
void adc_init(void);

int motor_position;
int keypad_num; 

TaskHandle_t xTimerGetTimerDaemonTaskHandle( void );

static void Task_A( void *pvParameters )  
{ 
int key[4];
   portTickType xLastWakeTime = xTaskGetTickCount ();  

   for( ;; )  
   {     
      vTaskDelayUntil( &xLastWakeTime, 10);  
	if( Cnt_A > 10000)
		 Cnt_A =0;

      Cnt_A += 1;    
	  if(PORTDbits.RD0 == 1)
		key[0] = 1;
	  else
		key[0] = 0;

	  if(PORTDbits.RD1 == 1)
		key[1] = 2;
	  else
		key[1] = 0;

	  if(PORTDbits.RD2 == 1)
		key[2] = 4;
	  else
		key[2] = 0;

	  if(PORTDbits.RD3 == 1)
		key[3] = 8;
	  else
		key[3] = 0;
		
	    keypad_num = key[0]+key[1]+key[2]+key[3];		
   }        
} 


static void Task_B( void *pvParameters )  
{     
const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

   for( ;; )  
   {     
  	 
   }    
} 
static void Task_C( void *pvParameters )  
{     
const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
static unsigned char buf[50];

   for( ;; )  
   {     
      vTaskDelay(xDelay );   
	  sprintf(buf,"\fDebugger dsPIC30F4011 with RTOS\r\n");  
	  putsUART2(buf);   
      sprintf(buf,"cnt : %d\r\n",Cnt_A);  
	  putsUART2(buf);   
   }    
}

static void Task_D( void *pvParameters )  
{ 
const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
/*int k;
float temp;
float buff1_sum;

	for( ;; )  
    { 
		for(k=1;k<=1500;k++)  // Low pass
		{
			ADCON1bits.SAMP = 1;                            
	        ConvertADC10();                 				
	        temp= temp + ReadADC10(0);                    	// Current
		}
			buff1_sum =temp/1500;
			voltage = buff1_sum;
		vTaskDelay(xDelay ); 
	}*/
}

#define NUM_TIMERS 1

 // An array to hold handles to the created timers. 
 TimerHandle_t xTimers[ NUM_TIMERS ];

 // An array to hold a count of the number of times each timer expires. 
 long lExpireCounters[ NUM_TIMERS ] = { 0 };

 // Define a callback function that will be used by multiple timer instances.
 //The callback function does nothing but count the number of times the
 //associated timer expires, and stop the timer once the timer has expired
 //10 times. 
 void vTimerCallback( TimerHandle_t pxTimer )
 {
 	  PV = (float)POSCNT*0.92307692307692307692307692307692;
	  ee = ((int)50 -(int)PV);			
	  pterm = (ee*kp);
	  iterm = (ki * sumee);    
      dterm = (kd * (ee-lastee));
	  sumee += ee;
	  lastee = ee;		
	  pid = pterm + iterm + dterm;
//	  vTaskDelay(xDelay );    
     // LATBbits.LATB0 ^= 1; 
 }


int main( void )
{
	prvSetupHardware();

   //  xTaskCreate( vTaskCode, "NAME", STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY,&xHandle );
   xTaskCreate( Task_A, ( signed char * ) "Cnt", 50, NULL, 1, NULL );  // STACK_SIZE = 50 bytes  in program memory
   //xTaskCreate( Task_B, ( signed char * ) "Task_B", 100, NULL, 3, NULL );  
   xTaskCreate( Task_C, ( signed char * ) "Serial", 100, NULL, 3, NULL );  
   //xTaskCreate( Task_D, ( signed char * ) "ADC", 10, NULL, 4, NULL );  

 
 long x;

     // Create then start some timers.  Starting the timers before the RTOS
     //scheduler has been started means the timers will start running
     //immediately that the RTOS scheduler starts. 
     for( x = 0; x < NUM_TIMERS; x++ )
     {
         xTimers[ x ] = xTimerCreate("PID", 1, pdTRUE,( void * ) x,vTimerCallback);  // Call back every 1 tick

         if( xTimers[ x ] == NULL )
         {
             // The timer was not created. 
         }
         else
         {
             // Start the timer.  No block time is specified, and even if one
             //was it would be ignored because the RTOS scheduler has not yet
             //been started. 
             if( xTimerStart( xTimers[ x ], 0 ) != pdPASS )
             {
                 // The timer could not be set into the Active state. 
             }
         }
     }

   vTaskStartScheduler(); 
 
	return 0;
}

static void prvSetupHardware( void )
{
	ADPCFG = 0xFFFF;
	// Inititial hardware
	//TRISBbits.TRISB0 = 0;
	TRISDbits.TRISD0 = 1;
	TRISDbits.TRISD1 = 1;
	TRISDbits.TRISD2 = 1;
	TRISDbits.TRISD3 = 1;
	
	init_uart();
	Init_QEI();
	adc_init();
    Cnt_A = 0;  
	motor_position = 0;
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

void Init_QEI(void)
{
  // Enable QEI Interrupt and Priority to "1"
  ConfigIntQEI( QEI_INT_ENABLE );     // Set the Interrupt enable bit

  OpenQEI(QEI_DIR_SEL_CNTRL &       // Control/Status Bit, QEICON<11>, Defines Timer Counter (POSCNT) Direction  
          QEI_INT_CLK &             // Internal clock (FOSC/4)
          QEI_INDEX_RESET_DISABLE & // Index Pulse does not reset Position Counter
          QEI_CLK_PRESCALE_256 &    // QEI 1:256 prescale value
          QEI_GATED_ACC_DISABLE &   // QEI Timer gated time accumulation enable
          QEI_LOGIC_CONTROL_IO &    // QEI logic controls state of I/O pin
          QEI_INPUTS_NOSWAP &       // QEI Phase A and Phase B inputs not swapped
          QEI_MODE_x4_MATCH &       // x4 mode with position counter reset by match (MAXCNT)
          QEI_IDLE_CON,             // QEI Discontinue module operation when device enters a idle mode.
          0                         // configures the Digital Filters (DFLTCON)
         );
  POSCNT = 0;                      // Set Position Counter Register
  MAXCNT = 65535;                 // Set Maximum Count Register

}

void adc_init(void)
{
unsigned int Channel, PinConfig, Scanselect, Adcon3_reg, Adcon2_reg,Adcon1_reg;
                ADCON1bits.ADON = 0;                    			// Turn off ADC     
                Channel = ADC_CH0_POS_SAMPLEA_AN0 &					// Channel 0 positive input select AN0
                          ADC_CH0_POS_SAMPLEA_AN1 & 				// Channel 0 positive input select AN1
                          ADC_CH0_POS_SAMPLEA_AN2 & 				// Channel 0 positive input select AN2
                          ADC_CH0_POS_SAMPLEA_AN3 & 				// Channel 0 positive input select AN3           
						  ADC_CH0_POS_SAMPLEA_AN4 & 				// Channel 0 positive input select AN4           
						  ADC_CH0_POS_SAMPLEA_AN5 & 				// Channel 0 positive input select AN5           
                          ADC_CH0_NEG_SAMPLEA_NVREF ;             	// Channel 0 negative VREF

                SetChanADC10(Channel);                               // Set channel configuration             
                ConfigIntADC10(ADC_INT_DISABLE);                     // Disable interrupt for ADC
                PinConfig = ENABLE_AN0_ANA &                         // Enable AN0-AN3 analog port
                            ENABLE_AN1_ANA &
                            ENABLE_AN2_ANA &
                            ENABLE_AN3_ANA &                                          
							ENABLE_AN4_ANA &                                          
							ENABLE_AN5_ANA ;                                          
                Scanselect = SKIP_SCAN_AN6 &                         // Scan for AN0-AN3
                             SKIP_SCAN_AN7 &						 // Skip Scan for AN4-A7
                             SKIP_SCAN_AN8 &
                             SKIP_SCAN_AN9;                                                              
                Adcon3_reg = ADC_SAMPLE_TIME_1 &                 	// Sample for 10 time
                             ADC_CONV_CLK_INTERNAL_RC &             // Internal Clock
                             ADC_CONV_CLK_Tcy; 
                Adcon2_reg = ADC_VREF_AVDD_AVSS &                   // Vref at Vdd and Vss
                             ADC_SCAN_ON &                          // Enable scan for ADC
                             ADC_ALT_BUF_OFF &                      // Disable alternate buffer
                             ADC_ALT_INPUT_OFF &                    // Disable alternate input
                             ADC_CONVERT_CH0&                       // Select CH0 convert         
                             ADC_SAMPLES_PER_INT_16;                // 16 sample between interrupt
                Adcon1_reg = ADC_MODULE_ON &                        // Enable module ADC
                             ADC_IDLE_CONTINUE &                    // ADC run on idle mode
                             ADC_FORMAT_INTG &                      // Output value integer format
                             ADC_CLK_MANUAL &                       // ADC manual clock
                             ADC_SAMPLE_SIMULTANEOUS &              // ADC sampling simultaneous
                             ADC_AUTO_SAMPLING_ON;                  // ADC auto sampling    
                    
OpenADC10(Adcon1_reg, Adcon2_reg,Adcon3_reg,PinConfig, Scanselect);  // Turn on ADC module     
}

