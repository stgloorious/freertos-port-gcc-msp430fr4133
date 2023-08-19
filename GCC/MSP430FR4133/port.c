/*
 * Copyright (C) 2023 Stefan Gloor
 * Forked from FreeRTOS Kernel V10.6.0
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the MSP430X port.
 *----------------------------------------------------------*/

/* Constants required for hardware setup.  The tick ISR runs off the ACLK,
not the MCLK. */
#define portACLK_FREQUENCY_HZ           ( ( TickType_t ) 32768 )
#define portINITIAL_CRITICAL_NESTING    ( ( uint16_t ) 10 )
#define portFLAGS_INT_ENABLED           ( ( StackType_t ) 0x08 )

/* We require the address of the pxCurrentTCB variable, but don't want to know
any details of its type. */
typedef void TCB_t;
extern volatile TCB_t * volatile pxCurrentTCB;

/* Each task maintains a count of the critical section nesting depth.  Each
time a critical section is entered the count is incremented.  Each time a
critical section is exited the count is decremented - with interrupts only
being re-enabled if the count is zero.

usCriticalNesting will get set to zero when the scheduler starts, but must
not be initialised to zero as this will cause problems during the startup
sequence. */
volatile uint16_t usCriticalNesting = portINITIAL_CRITICAL_NESTING;
/*-----------------------------------------------------------*/

/*
 * Sets up the periodic ISR used for the RTOS tick.  This uses timer 0, but
 * could have alternatively used the watchdog timer or timer 1.
 */
void vPortSetupTimerInterrupt( void );
static void prvSetupTimerInterrupt( void );
/*-----------------------------------------------------------*/

/* 
 * Macro to restore a task context from the task stack.  This is effectively
 * the reverse of portSAVE_CONTEXT().  First the stack pointer value is
 * loaded from the task control block.  Next the value for usCriticalNesting
 * used by the task is retrieved from the stack - followed by the value of all
 * the general purpose msp430 registers.
 *
 * The bic instruction ensures there are no low power bits set in the status
 * register that is about to be popped from the stack.
 */
#define portRESTORE_CONTEXT()								\
	asm volatile (	"mov.w	pxCurrentTCB, r12		\n\t"	\
					"mov.w	@r12, r1				\n\t"	\
					"pop	r15						\n\t"	\
					"mov.w	r15, usCriticalNesting	\n\t"	\
					"pop	r15						\n\t"	\
					"pop	r14						\n\t"	\
					"pop	r13						\n\t"	\
					"pop	r12						\n\t"	\
					"pop	r11						\n\t"	\
					"pop	r10						\n\t"	\
					"pop	r9						\n\t"	\
					"pop	r8						\n\t"	\
					"pop	r7						\n\t"	\
					"pop	r6						\n\t"	\
					"pop	r5						\n\t"	\
					"pop	r4						\n\t"	\
					"bic	#(0xf0),0(r1)			\n\t"	\
					"reti							\n\t"	\
				);

#define portSAVE_CONTEXT()									\
	asm volatile (	"push	r4						\n\t"	\
					"push	r5						\n\t"	\
					"push	r6						\n\t"	\
					"push	r7						\n\t"	\
					"push	r8						\n\t"	\
					"push	r9						\n\t"	\
					"push	r10						\n\t"	\
					"push	r11						\n\t"	\
					"push	r12						\n\t"	\
					"push	r13						\n\t"	\
					"push	r14						\n\t"	\
					"push	r15						\n\t"	\
					"mov.w	usCriticalNesting, r14	\n\t"	\
					"push	r14						\n\t"	\
					"mov.w	pxCurrentTCB, r12		\n\t"	\
					"mov.w	r1, @r12				\n\t"	\
				);

/*
 * Initialise the stack of a task to look exactly as if a call to
 * portSAVE_CONTEXT had been called.
 *
 * See the header file portable.h.
 */
StackType_t *pxPortInitialiseStack( StackType_t *pusTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
uint32_t ulTemp;

    /*
        Place a few bytes of known values on the bottom of the stack.
        This is just useful for debugging and can be included if required.

        *pusTopOfStack = ( StackType_t ) 0x1111;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) 0x2222;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) 0x3333;
        pusTopOfStack--;
    */

    /* Data types are 16 bits in small data and code model */
    pusTopOfStack = ( uint16_t * ) pusTopOfStack;
    ulTemp = ( uint32_t )( uint16_t )pxCode;
    *pusTopOfStack = ( uint16_t ) ulTemp;

    pusTopOfStack--;
    *pusTopOfStack = portFLAGS_INT_ENABLED;
    pusTopOfStack -= ( sizeof( StackType_t ) / 2 );

    /* Next the general purpose registers. */
    #ifdef PRELOAD_REGISTER_VALUES
        *pusTopOfStack = ( StackType_t ) 0xffff;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) 0xeeee;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) 0xdddd;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) pvParameters;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) 0xbbbb;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) 0xaaaa;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) 0x9999;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) 0x8888;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) 0x5555;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) 0x6666;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) 0x5555;
        pusTopOfStack--;
        *pusTopOfStack = ( StackType_t ) 0x4444;
        pusTopOfStack--;
    #else
        pusTopOfStack -= 3;
        *pusTopOfStack = ( StackType_t ) pvParameters;
        pusTopOfStack -= 9;
    #endif

    /* A variable is used to keep track of the critical section nesting.
    This variable has to be stored as part of the task context and is
    initially set to zero. */
    *pusTopOfStack = ( StackType_t ) portNO_CRITICAL_SECTION_NESTING;

    /* Return a pointer to the top of the stack we have generated so this can
    be stored in the task control block for the task. */
    return pusTopOfStack;
}

BaseType_t xPortStartScheduler( void )
{
	/* Setup the hardware to generate the tick.  Interrupts are disabled when
	this function is called. */
	prvSetupTimerInterrupt();

	/* Restore the context of the first task that is going to run. */
	portRESTORE_CONTEXT();

	/* Should not get here as the tasks are now running! */
	return pdTRUE;
}

void vPortEndScheduler( void )
{
    /* It is unlikely that the MSP430 port will get stopped.  If required simply
    disable the tick interrupt here. */
}
/*-----------------------------------------------------------*/

/*
 * Hardware initialisation to generate the RTOS tick.
 */
void vPortSetupTimerInterrupt( void )
{
    prvSetupTimerInterrupt();
}
/*-----------------------------------------------------------*/

void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) vTickISREntry (void)
{
extern void vPortTickISR( void );

    __bic_SR_register_on_exit( SCG1 + SCG0 + OSCOFF + CPUOFF );
    #if configUSE_PREEMPTION == 1
        extern void vPortPreemptiveTickISR( void );
        vPortPreemptiveTickISR();
    #else
        extern void vPortCooperativeTickISR( void );
        vPortCooperativeTickISR();
    #endif
}

void vPortPreemptiveTickISR(){
    
}
/*
 * Manual context switch called by portYIELD or taskYIELD.  
 *
 * The first thing we do is save the registers so we can use a naked attribute.
 */
void vPortYield( void ) __attribute__ ( ( naked ) );
void vPortYield( void )
{
	/* We want the stack of the task being saved to look exactly as if the task
	was saved during a pre-emptive RTOS tick ISR.  Before calling an ISR the 
	msp430 places the status register onto the stack.  As this is a function 
	call and not an ISR we have to do this manually. */
	asm volatile ( "push	r2" );
	_DINT();

	/* Save the context of the current task. */
	portSAVE_CONTEXT();

	/* Switch to the highest priority task that is ready to run. */
	vTaskSwitchContext();

	/* Restore the context of the new task. */
	portRESTORE_CONTEXT();
}

static void prvSetupTimerInterrupt( void )
{
	/* Ensure the timer is stopped. */
	TA0CTL = 0;

	/* Run the timer of the ACLK. */
	TA0CTL = TASSEL_1;

	/* Clear everything to start with. */
	TA0CTL |= TACLR;

	/* Set the compare match value according to the tick rate we want. */
	TA0CCR0 = portACLK_FREQUENCY_HZ / configTICK_RATE_HZ;

	/* Enable the interrupts. */
	TA0CCTL0 = CCIE;

	/* Start up clean. */
	TA0CTL |= TACLR;

	/* Up mode. */
	TA0CTL |= MC_1;
}

