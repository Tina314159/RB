//---2nd order Butterworth low-pass filter
//---12-May-2026 15:15:23
int myFilter1_ns = 2; // number of sections
uint32_t timeoutValue = 20000; // time interval - us; f_s = 50 Hz
static	struct	biquad myFilter1[]={ // define the array of floating point biquads
        {7.529780e-02, 7.529830e-02, 0.000000e+00, 1.000000e+00, -2.282609e-01, 0.000000e+00, 0, 0, 0, 0, 0},
        {1.000000e+00, 1.999993e+00, 9.999934e-01, 1.000000e+00, -5.983046e-01, 3.788565e-01, 0, 0, 0, 0, 0}
        };
