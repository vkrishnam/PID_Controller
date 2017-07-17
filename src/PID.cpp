#include "PID.h"
#include <ctime>
#include <iostream>
#include <math.h>

using namespace std;

#define KP_PD 0.01
#define KI_PD 0.0001
#define KD_PD 1.0


/*
* TODO: Complete the PID class.
*/

//boolean changeIndex = false;
int scenario = 0;

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;

    num_steps = 0;

    total_error = 0;
    best_error = 1e8;

    dp[0] = KP_PD;
    dp[1] = KI_PD;
    dp[2] = KD_PD;

    go_up[0] = go_up[1] = go_up[2] = 0;

    idx = 0;
}

#define BIG_SAMPLE 200
#define SMALL_SAMPLE 10

double previous_time = 0;
double cte_prev = 0;
double cte_sum = 0;

void PID::UpdateError(double cte) {
    /*
     * P error.
     */
    p_error = cte;

    /*
     * D error
     */
    d_error = (cte - cte_prev);
    cte_prev = cte;

    /*
     * I error
     */
    cte_sum += cte;
    i_error = cte_sum;


    /*
     * Update total error
     */
    total_error += pow(cte, 2);

    if(num_steps%BIG_SAMPLE == 0)
	{
 
		best_error = 1e8;//total_error;
                total_error = 0.0;
                //Also reset the dps
    		dp[0] = KP_PD;
    		dp[1] = KI_PD;
    		dp[2] = KD_PD;

        }

    /* Update number of steps. */
    num_steps += 1;
}


//#define STEPS 2
void PID::twiddle() {
    double p[3] = {Kp, Ki, Kd};
    
    //if(dp[0]+dp[1]+dp[2] < 0.00001)
    //	return;
    
    if((num_steps%SMALL_SAMPLE) != 0)
        return;


    if (scenario == 0) {
        p[idx] += dp[idx];
        scenario = 1;
    	Kp = p[0];
    	Ki = p[1];
    	Kd = p[2];
        return;
    }


    double err = total_error;
    
    cout << "num steps : " << num_steps << "err :" << err << "best err :" <<best_error <<endl;
 cout << "Kp: " << Kp << "Kd: " << Kd << "Ki: " << Ki <<endl;


    if (scenario == 1) {
        if(err < best_error) {
        	best_error = err;
        	dp[idx] *= 1.1;
		//next index
        	idx = (idx+1)%3;
        	scenario = 0;
		return;
        }
       else {
            p[idx] -= 2*dp[idx];
            scenario = 2;
    	    Kp = p[0];
    	    Ki = p[1];
            Kd = p[2];
           return;
       }
    }
    if(scenario == 2) {
	if(err < best_error) {
      		best_error = err;
                dp[idx] *= 1.1;
		//next index
        	idx = (idx+1)%3;
        	scenario = 0;
		return;
	}
	else {
            p[idx] += dp[idx];
	    dp[idx] *= 0.9;
	    //next index
            idx = (idx+1)%3;
            scenario = 0;
    	    Kp = p[0];
    	    Ki = p[1];
            Kd = p[2];
           return;
	}
    }
    
}        



double PID::getOutput() {
    double pid_out;

    twiddle();

    pid_out = (-1*Kp*p_error) + (-1*Kd*d_error) + (-1*Ki*i_error);
    if (pid_out > 1) {
        pid_out = 1;
    } else if (pid_out < -1) {
        pid_out = -1;
    }

    return pid_out;
}
