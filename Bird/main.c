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
#include <time.h>
#include "PWM.h"


/* Global Variables */
typedef struct{
	NiFpga_IrqContext irqContext; //IRQ context reserved
	table *a_table; //table
	NiFpga_Bool irqThreadRdy; 	//IRQ thread ready flag
} ThreadResource;

MyRio_Encoder encC0; //structure that maintains the current status and count value
NiFpga_Status status;
MyRio_Pwm pwmA0;
uint8_t selectReg;

static int targetrpm;
static int incrementrpm;
static int incrementtime;
static double pitchoff;
static double pitchamp;
#define Neutral 7500 //neutral position PWM width
#define Max 17857 //PWM counter for defining frequency
static double width_increment;

struct biquad{ //structure that contains elements of 1 biquad section
	double b0; double b1; double b2; //numerator
	double a0; double a1; double a2; //denominator
	double x0; double x1; double x2; //input
	double y1; double y2; //output
};

#define SATURATE(x, lo, hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))

#define LoopDuration    60  /* How long to output the signal, in seconds */

#define IMAX 250 //max points
static double speed[IMAX]; //angular velocity buffer
static double *bp_speed = speed; //angular velocity buffer pointer
static double Ua[IMAX]; //DAC output voltage/amplifier input buffer
static double *bp_ua = Ua; //voltage buffer pointer

/* prototypes */
void *Timer_Irq_Thread(void *resource); //interrupt timer service routine
double cascade(double xin, //input
			   struct biquad *fa, //biquad array
			   int ns, //number of segments
			   double ymin, //min output
			   double ymax); //max output
double vel(void); //calculates velocity from encoder count
double pos(void); //calculates position from encoder count
void wait(void); //makes program waits for 5 milliseconds

/* definitions */
/*---------------------------------------------------------------------------------------
 * Function: main
 * Purpose: holds our main thread and does the following:
 * 			(1) initializes the table editor values and calls the table editor
 * 			(2) initializes timer interrupt and registers and creates timer thread
 * 			(3) when DEL is pressed, table editor is exits, and signals timer thread to terminate and terminates program
 --------------------------------------------------------------------------------------*/
int main(int argc, char **argv){

    status = MyRio_Open();		    			// open FPGA session
    if (MyRio_IsNotSuccess(status)) return status;

    printf("Enter the target rpm: ");
    scanf("%d", &targetrpm); //will step rpm up from zero to targetrpm by user specified amount and time
    printf("Enter speed increment amount in rpm: ");
    scanf("%d", &incrementrpm); //after a certain amount of time, increment ref_speed by this amount
    printf("Enter increment time in s: ");
    scanf("%d", &incrementtime); //amount of time between ref_speed increments
    printf("Enter pitch offset in percent: ");
    scanf("%lf", &pitchoff);
    pitchoff = pitchoff/100;
    printf("Enter pitch amplitude in degrees (0-60 degrees): "); //amplitude pitching from horizontal to peak
    scanf("%lf", &pitchamp);
    width_increment = pitchamp*Max*(0.2/60); //converting amplitude to number that code can use


    /* initialize table editor values */
    char *Table_Title = "Velocity Controller"; //table name
    table my_table[] = {
    		{"V_R: rpm ", 0, 0.000},
    		{"V_J: rpm   ", 0, 0.000},
    		{"VDAout: mV ", 0, 0.000},
    		{"Kp: V-s/r ", 1, 0.040},
    		{"Ki: V/r ", 1, 0.800},
    		{"BTI: ms ", 1, 2.000},
    		{"Duty Cycle %: ", 0, 42.00},
    		{"Pos: rev ", 0, 0.000},
    		{"Start: ", 1, 0.000}
    };

    uint32_t timeoutValue = 5000; //time until next interrupt-[microseconds]
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

    ctable2(Table_Title, my_table, 9); //call table--terminates when DEL is pressed

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
 * 			(1) cast thread resource and table editor as convenient variables
 * 			(2) initialize analog output and sets up encoder interface
 * 			(3) sets up biquad for our specific system
 * 			(4) sets up next timer interrupt
 * 			(5) when interrupt occurs, calculates current motor speed, calculate and update new biquad coefficients,
 * 				use coefficients and speed to find voltage output of PI controller and send output to analog output
 * 			(6) save results and acknowledge interrupt
 * 			(7) when thread is terminated, send results to MATLAB and terminate thread
 * Parameters: (in) resource - thread resource
 * 				(out) NULL - return to main
 ---------------------------------------------------------------------------------------------------*/
void *Timer_Irq_Thread(void *resource){
	extern NiFpga_Session myrio_session;
	/* cast input arguments */
	ThreadResource* threadResource = (ThreadResource*) resource;
	double *Omega_R = &((threadResource->a_table+0)->value); //reference velocity (edit)
	double *Omega_J = &((threadResource->a_table+1)->value); //actual velocity (show)
	double *VDAout = &((threadResource->a_table+2)->value); //output voltage (show)
	double *Kp = &((threadResource->a_table+3)->value); //Kp-value (edit)
	double *Ki = &((threadResource->a_table+4)->value); //Ki-value (edit)
	double *bti = &((threadResource->a_table+5)->value); //BTI-value (edit)
	double *duty = &((threadResource->a_table+6)->value); // PWM duty cycle
	double *rev = &((threadResource->a_table+7)->value); //position in rev
	double *S = &((threadResource->a_table+8)->value); //0=stop, 1=start

	*Omega_R = targetrpm;
	int system_ns = 1; //number of sections
	double err; //error signal = speed ref - speed actual
	double old_vel; //previous angular velocity-[rad/s]
	old_vel = 0;
	double save_old; //saving previous angular velocity
	double save_curr; //saving current angular velocity
	double bti_s; //BTI in seconds
	double w_J; //actual velocity in rad/s
	double w_R; //reference velocity in rad/s
	double V; //PI output voltage in Volts
	double p; //position in BDI
	p=0;
	double e;
	e = 0.1; //position error in rev
	int time_len; //number of cycles/interrupts between each rpm increment
	time_len = (int) incrementtime/0.002;
	int time_count; //count for number of interrupts between each increment
	time_count = 0;
	int control_pulse; //counter to indicate when to send pulse to load cell
	control_pulse = 0;
	int width; //pulse width used by code to control servo
	int c; //position of wing: low = 1, high = 2;
	c=0;
	int estop; //if speed too high, decrement speed to zero via controller
	estop = 0;


	struct biquad system[] = {
				{0,  0, 0,
				1, -1, 0, 0, 0, 0, 0, 0}
	}; //our PI controller

	/* initialize AIO, DIO, and encoder*/
	MyRio_Aio AOC0; //connector C analog output 0
	Aio_InitCO0(&AOC0); //initialize output 0
	Aio_Write(&AOC0, 0); //turn off motor
	EncoderC_initialize(myrio_session, &encC0); //initializes encoder interface
	pos(); //set zero position
	MyRio_Dio dataPulse; //Connector A digital output
	dataPulse.dir = DIOA_70DIR;
	dataPulse.out = DIOA_70OUT;
	dataPulse.in = DIOA_70IN;
	dataPulse.bit = 1;
	Dio_WriteBit(&dataPulse, NiFpga_False);

	/* initialize PWM */
	pwmA0.cnfg = PWMA_0CNFG;
	pwmA0.cs = PWMA_0CS;
	pwmA0.max = PWMA_0MAX;
	pwmA0.cmp = PWMA_0CMP;
	pwmA0.cntr = PWMA_0CNTR;
    Pwm_Configure(&pwmA0, Pwm_Invert | Pwm_Mode,
	            Pwm_NotInverted | Pwm_Enabled); //initializes PWM signal
    Pwm_ClockSelect(&pwmA0, Pwm_4x); //sets freq. of PWM internal clock (Pwm_4x=10 Mhz)
    /* The counter increments at 40 MHz / 4 = 10 MHz and the counter counts
     * from 0 to 17857. The frequency of the PWM waveform is 10 MHz / 17857
     * = 560Hz. */
    Pwm_CounterMaximum(&pwmA0, Max); //Set the maximum counter value=17857
	Pwm_CounterCompare(&pwmA0, Neutral); //sets number of counts when PWM is high
	*duty = (double) Neutral/Max;
	status = NiFpga_ReadU8(myrio_session, SYSSELECTA, &selectReg);
	MyRio_ReturnValueIfNotSuccess(status, status,
	        "Could not read from the SYSSELECTA register!")
	selectReg = selectReg | (1 << 2);
	status = NiFpga_WriteU8(myrio_session, SYSSELECTA, selectReg);
	MyRio_ReturnValueIfNotSuccess(status, status,
	        "Could not write to the SYSSELECTA register!")


	Dio_WriteBit(&dataPulse, NiFpga_True);
	while (threadResource->irqThreadRdy == NiFpga_True){ //while main thread has not signaled this thread to stop
		/* pause loop while waiting for interrupt */
		uint32_t irqAssert = 0;
		Irq_Wait(threadResource->irqContext, TIMERIRQNO, &irqAssert, (NiFpga_Bool*) &(threadResource->irqThreadRdy));
		/* schedule next interrupt */
		NiFpga_WriteU32(myrio_session, IRQTIMERWRITE, *bti*1000); //time in microseconds
		NiFpga_WriteBool(myrio_session, IRQTIMERSETTIME, NiFpga_True);

		/* service interrupt */
		if(irqAssert){ //if IRQ has been asserted (non-zero)

			if (old_vel != *Omega_R){ //if reference value is different
				/* reset index back to zero */
				bp_speed = speed;
				bp_ua = Ua;
				/*Save previous and current velocity in rad/s */
				save_old = old_vel*(2*M_PI/60); //previous vel
				save_curr = *Omega_R*(2*M_PI/60); //current vel
				old_vel = *Omega_R; //set current velocity as previous for next loop
			}

			/* calculate speed and check if it is too fast */
			bti_s = *bti/1000; //BTI in seconds
			*Omega_J = (1000*60/(*bti*1425.1))*vel(); //measure velocity in rpm
			w_J = *Omega_J*(2*M_PI/60); //measured velocity in rad/s
			if (*Omega_J >200) {
				estop = 1;
				printf("Too fast. Emergency Stop. After motor stopped, press DEL and restart program");
			}

			/* Increment/decrement ref_speed */
			if (*S==0 || estop == 1){ //decrement ref_speed to make controller turn off motor
				if (*Omega_R > 0.02){
					*Omega_R = *Omega_R - 0.02; //decrement ref_speed by 0.02 during each BTI
				}
				else if (*Omega_R <= 0.02){
					*Omega_R = 0; //do not want speed to become negative
				}
				control_pulse = 0; //reset control pulse
				Dio_WriteBit(&dataPulse, NiFpga_False); //reset control pulse
				width = (int) Neutral + width_increment; //set pitch to steepest to reduce air-resistance in slowing
			}
			else if (*S==1){ //increment ref_speed to targetrpm specified by user
				if (control_pulse == 0){ //send pulse to load cell to sync data
					Dio_WriteBit(&dataPulse, NiFpga_True);
					control_pulse = 1;
				}
				time_count++;
				if (time_count==time_len && *Omega_R != targetrpm){ //at each time increment when ref_speed is not targetspeed
					if (*Omega_R < targetrpm-incrementrpm){
						/*when ref_speed is less than targetrpm and the next time incrementing ref_speed does not exceed targetrpm */
						*Omega_R = *Omega_R + incrementrpm;
					}
					else if (*Omega_R < targetrpm){ //if difference between ref_speed and targetrpm is less than increment amount
						*Omega_R = targetrpm;
					}
					time_count = 0;
				}
			}
			w_R = *Omega_R*(2*M_PI/60); //reference velocity in rad/s

			/* Check position of wing and move servo to position */
			p = pos(); //position in BDI
			*rev = p/1425.1; //position in rev
			if (p!=0 && S!=0){ //when the wing moves and is not stopping
				if (fmod(*rev+pitchoff, 0.5) < e){ //half or whole int
					if (fmod(*rev+pitchoff, 1) < e){
						c = 1;
					}
					else {
						c = 2;
					}
				}
				if (c==1){
					width = (int) Neutral - width_increment;
				}
				if (c==2){
					width = (int) Neutral + width_increment;
				}
			}
			Pwm_CounterCompare(&pwmA0, width);
			*duty = (double) width/Neutral;

			/* Implement controller to change actual velocity */
			system->b0 = (*Kp)+0.5*(*Ki)*(bti_s); //recalculate b0
			system->b1 = -(*Kp)+0.5*(*Ki)*(bti_s); //recalculate b1
			err = w_R - w_J; //current error in rad/s
			V = cascade(err, system, system_ns, -10, 10); //find output voltage from PI in Volts
			*VDAout = V*1000; //output voltage from PI in mV
			Aio_Write(&AOC0, V); //send output value to AOC0/motor
			if (bp_speed<speed+IMAX) *bp_speed++ = w_J; //store the current velocity
			if (bp_ua<Ua+IMAX) *bp_ua++ = V; //store current output voltage
			Irq_Acknowledge(irqAssert); //acknowledge interrupt
		}
	}
	Aio_Write(&AOC0, 0); //turn off motor
	Pwm_Configure(&pwmA0, Pwm_Invert | Pwm_Mode,
		            Pwm_NotInverted | Pwm_Disabled); //turn off PWM signal
	Dio_WriteBit(&dataPulse, NiFpga_False);

	/* Saving to MATLAB
	MATFILE *mf; //the file where the buffer for speed will be written onto
	int error = 0;
	mf = openmatfile("Lab7.mat", &error);
	if(!mf) printf("Can't open mat file %d\n", error);
	matfile_addstring(mf, "myName", "Phi Nguyen");
	matfile_addmatrix(mf, "KP", Kp, 1, 1, 0);
	matfile_addmatrix(mf, "KI", Ki, 1, 1, 0);
	matfile_addmatrix(mf, "BTI", &bti_s, 1, 1, 0); //in seconds
	matfile_addmatrix(mf, "prev_vel", &save_old, 1, 1, 0); //in rad/s
	matfile_addmatrix(mf, "curr_vel", &save_curr, 1, 1, 0); //in rad/s
	matfile_addmatrix(mf, "vel", speed, IMAX, 1, 0); //in rad/s
	matfile_addmatrix(mf, "volt", Ua, IMAX, 1, 0); //in Volts
	matfile_close(mf);*/

	pthread_exit(NULL); //terminate new thread
	return NULL;
}
/*--------------------------------------------------------------------------------------------------
 * Function: cascade
 * Purpose: computes all sections of biquad cascade by looping through difference equation for each biquad section
 * Parameters: (in) xin - current system input
 * 			   (in) fa - array of biquad structures
 * 			   (in) ns - number of biquad sections
 * 			   (in) ymin - lower saturation limit
 * 			   (in) ymax - upper saturation limit
 * Return: (out) y0 - current value of system output
 ----------------------------------------------------------------------------------------------*/
double cascade(double xin, struct biquad *fa, int ns, double ymin, double ymax){
	struct biquad *f;
	double y0; //system output
	int i;
	y0 = xin; //setting output as system input to be used in loop
	f = fa; //set pointer to first element of array
	for (i=0; i<ns;  i++){ //loop ns times (the number of biquad sections)
		f->x0 = y0; //previous value of biquad is input value of current biquad
		y0 =(f->b0 * f->x0 + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2)/(f->a0); //calculate section output
		if (i==ns-1){ //final biquad
			y0 = SATURATE(y0, ymin, ymax); //saturates y0 to +-10V
		}
		/* assign to previous instances for next iteration/section */
		f->x2 = f->x1; f->x1 = f->x0;
		f->y2 = f->y1; f->y1 = y0;
		f++; //increment to next biquad section
	}
	return y0;
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
	static int cn; //current count of encoder
	static int cn0; //first count of encoder
	static int first = 0; //determine first time pos is called
	double diff; //displacement in BDI
	cn = Encoder_Counter(&encC0); //reading current count of encoder
	if (first == 0){
		cn0 = cn; //first time pos is called, set value of previous counter to the current counter--first position
		first = 1; //no longer the first time pos is called
	}
	diff = cn - cn0; //displacement relative to first position
	return diff; //returns pos to calling function in units of BDI
}
/*--------------
 * Function: wait
 * Purpose: waits for 5 milliseconds
 */
void wait(void){
	uint32_t i;
	i = 417000;
	while (i>0){
		i--;
	}
	return;
}
