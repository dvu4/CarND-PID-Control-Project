#include "PID.h"
#include <cmath>
#include <iostream>
#include <time.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    p_error = 0.0;
    d_error = 0.0;
    i_error = 0.0;

    // Twiddling parameters
    p = {Kp, Kd, Ki};
    dp = {1, 1, 1};
    iter = 1;

    pid_index = 0;  // 0:Kp, 1:Kd, Ki:2
    n_iter = 2000;

    diff_cte = 0;
    incr_cte = 0; 
    total_error = 0;
    best_error = std::numeric_limits<double>::max();

    twiddle_adding = false; 
    twiddle_subtracting = false;

    prev_time = clock();  
}

void PID::UpdateError(double cte) {
    if (iter == 1) {
        // to get correct initial d_error
        p_error = cte;
    }

    diff_cte = cte - p_error;
    incr_cte += cte;

    p_error = cte;
    d_error = diff_cte;
    i_error = incr_cte; 

    if (iter % n_iter == 0){
        cout << "Iteration: " << iter << endl;
        cout << "total error: " << total_error << endl;
        cout << "best error: " << best_error << endl;

        if (total_error < best_error) {
            //cout << "improvement!" << endl;
            best_error = total_error;
            if (iter !=  n_iter) {
                dp[pid_index] *= 1.1;            
            }
            // next parameter
            pid_index = (pid_index + 3) % 3;
            twiddle_adding = false;
            twiddle_subtracting = false;
        }
        if (!twiddle_adding && !twiddle_subtracting) {
            // add dp[i] to parameter p[i]
            p[pid_index] += dp[pid_index];
            twiddle_adding = true;
        }
        else if (twiddle_adding && !twiddle_subtracting) {
            // subtract dp[i] from parameter p[i]
            p[pid_index] +=  -2 * dp[pid_index];     
            twiddle_subtracting = true;         
        }
        else {
            //reduce dp[i], move on to next parameter
            p[pid_index] += dp[pid_index];    
            dp[pid_index] *= 0.9;

            // next parameter
            pid_index = (pid_index + 3) % 3;
            twiddle_adding = false;
            twiddle_subtracting = false;
        }

        total_error = 0;
        cout << "new PID parameters" << endl;
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;        
    }
    iter++;
}

double PID::TotalError() {
    double current_time = clock();
    double dt = (current_time - prev_time);
    prev_time = current_time;

    double total_error = -Kp * p_error - Kd * d_error - Ki * i_error;
    return total_error;  
}


/*
double twiddle(tol=0.2){
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p
}
*/

