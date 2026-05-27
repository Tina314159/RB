//---2nd order Butterworth low-pass filter, 4:08pm
//---19-May-2026 17:53:58
int myFilter1_ns = 2; // number of sections
uint32_t timeoutValue = 20000; // time interval - us; f_s = 50 Hz
static	struct	biquad myFilter1[]={ // define the array of floating point biquads
        {2.292529e-01, 2.292544e-01, 0.000000e+00, 1.000000e+00, 1.137254e-01, 0.000000e+00, 0, 0, 0, 0, 0},
        {1.000000e+00, 1.999993e+00, 9.999934e-01, 1.000000e+00, 3.019660e-01, 3.447804e-01, 0, 0, 0, 0, 0}
        };
