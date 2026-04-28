/* includes */
#include <stdio.h>
#include "MyRio.h"
#include "T1.h"
#include "AIO.h"
#include "TimerIRQ.h"
#include <pthread.h>
#include "matlabfiles.h"
#include "Encoder.h"
#include "ctable2.h"
#include "math.h"
#include "DIO.h"

/* Global Variables */
typedef struct{
	NiFpga_IrqContext irqContext; //IRQ context reserved
	table *a_table; //table
	NiFpga_Bool irqThreadRdy; 	//IRQ thread ready flag
} ThreadResource;

MyRio_Encoder encC0; //structure that maintains the current status and count value
static uint32_t timeoutValue = 5000; //time until next interrupt-[microseconds]

/* prototypes */
void *Timer_Irq_Thread(void *resource); //interrupt timer service routine
double vel(void); //calculates velocity from encoder count
double pos(void); //calculates position from encoder count

/* definitions */
/*---------------------------------------------------------------------------------------
 * Function: main
 * Purpose: holds our main thread and does the following:
 * 			(1) initializes the table editor values and calls the table editor
 * 			(2) initializes timer interrupt and registers and creates timer thread
 * 			(3) when DEL is pressed, table editor is exits, and signals timer thread to terminate and terminates program
 --------------------------------------------------------------------------------------*/
int main(int argc, char **argv){
	NiFpga_Status status;
    status = MyRio_Open();		    			// open FPGA session
    if (MyRio_IsNotSuccess(status)) return status;

    /* initialize table editor values */
    char *Table_Title = "Velocity Controller"; //table name
    table my_table[] = {
    		{"P: rev ", 0, 0.000},
    		{"V: rpm ", 0, 0.000},
    };


    int32_t irq_status;
    MyRio_IrqTimer irqTimer0;
    ThreadResource irqThread0;
    pthread_t thread;

    /*Registers corresponding to the IRQ channel */
    irqTimer0.timerWrite = IRQTIMERWRITE;
    irqTimer0.timerSet = IRQTIMERSETTIME;
    irq_status = Irq_RegisterTimerIrq(&irqTimer0, &irqThread0.irqContext, timeoutValue); //register timer IRQ

    /* set up new thread */
    irqThread0.a_table = my_table; //initialize thread resource to point to table
    irqThread0.irqThreadRdy = NiFpga_True; //set the indicator to allow new thread
    irq_status = pthread_create(&thread, NULL, Timer_Irq_Thread, &irqThread0);

    ctable2(Table_Title, my_table, 2); //call table--terminates when DEL is pressed

    /* terminate and unregister threads */
    irqThread0.irqThreadRdy = NiFpga_False;  //signal thread to end
    irq_status = pthread_join(thread, NULL);
    irq_status = Irq_UnregisterTimerIrq(&irqTimer0, irqThread0.irqContext);
    return irq_status;

	status = MyRio_Close();						// close FPGA session
	return status;
}
/*---------------------------------------------------------------------------------------------------
 * Function: Timer_Irq_Thread
 * Purpose: timer interrupt service routine that does the following:

 ---------------------------------------------------------------------------------------------------*/
void *Timer_Irq_Thread(void *resource){
	extern NiFpga_Session myrio_session;
	/* cast input arguments */
	ThreadResource* threadResource = (ThreadResource*) resource;
	double *p = &((threadResource->a_table+0)->value); //position (show)
	double *v = &((threadResource->a_table+1)->value); //velocity (show)

	double T; //time
	double timeout;
	timeout = (double) timeoutValue; //cast timeoutValue
	T = timeout/1000000; //time in seconds
	double a;


	/* initialize encoder*/
	EncoderC_initialize(myrio_session, &encC0); //initializes encoder interface

	while (threadResource->irqThreadRdy == NiFpga_True){ //while main thread has not signaled this thread to stop
		/* pause loop while waiting for interrupt */
		uint32_t irqAssert = 0;
		Irq_Wait(threadResource->irqContext, TIMERIRQNO, &irqAssert, (NiFpga_Bool*) &(threadResource->irqThreadRdy));
		/* schedule next interrupt */
		NiFpga_WriteU32(myrio_session, IRQTIMERWRITE, timeoutValue); //time in microseconds
		NiFpga_WriteBool(myrio_session, IRQTIMERSETTIME, NiFpga_True);

		/* service interrupt */
		if(irqAssert){ //if IRQ has been asserted (non-zero)
			a = pos(); //position in BDI
			*p = a/1425; //position in rev
			*v = (60/(1425*T))*vel(); //velocity in rpm
			Irq_Acknowledge(irqAssert); //acknowledge interrupt
		}
	}

	pthread_exit(NULL); //terminate new thread
	return NULL;
}
/*-----------------------------------------------------------------------
 * Function: vel
 * Purpose: reads the encoder counter, then calculates the velocity of motor in BDI/DTI
 * Parameters: (in) none
 * Return: (out) diff - the velocity (defined as the difference between the current and previous count of encoder)
 ----------------------------------------------------------------------------*/
double vel(void) {
	static int cn; //current count of encoder
	static int cn1; //previous count of encoder
	static int first = 0; //determine first time vel is called
	double diff; //speed in BDI/BTI
	cn = Encoder_Counter(&encC0); //reading current count of encoder
	if (first == 0){
		cn1 = cn; //first time vel is called, set value of previous counter to the current counter
		first = 1; //no longer the first time vel is called
	}
	diff = cn - cn1; //speed is the difference between current and previous count (since BTI=1)
	cn1 = cn; //replace previous count with current count for use in next BTI
	return diff; //returns speed to calling function in units of BDI/BTI
}
/*-----------------------------------------------------------------------
 * Function: pos
 * Purpose: reads the encoder counter, then calculates the displacement of motor in BDI relative to the first position
 * Parameters: (in) none
 * Return: (out) diff - the displacement (defined as the difference between the current and first count of encoder)
 ----------------------------------------------------------------------------*/
double pos(void) {
	static int pcn; //current count of encoder
	static int pcn0; //first count of encoder
	static int pfirst = 0; //determine first time pos is called
	double pdiff; //displacement in BDI
	pcn = Encoder_Counter(&encC0); //reading current count of encoder
	if (pfirst == 0){
		pcn0 = pcn; //first time pos is called, set value of previous counter to the current counter--first position
		pfirst = 1; //no longer the first time pos is called
	}
	pdiff = pcn - pcn0; //displacement relative to first position
	return pdiff; //returns pos to calling function in units of BDI
}
