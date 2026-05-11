//---Phi PIDF position control
//---13-Mar-2026 19:15:26
int PIDF_ns = 1; // number of sections
uint32_t timeoutValue = 5000; // time interval - us; f_s = 200 Hz
static	struct	biquad PIDF[]={ // define the array of floating point biquads
        {2.123397e+01, -4.181512e+01, 2.058278e+01, 1.000000e+00, -1.403380e+00, 4.033796e-01, 0, 0, 0, 0, 0}
        };
