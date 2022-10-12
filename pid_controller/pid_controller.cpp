/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  PID::k_p = Kpi;
  PID::k_i = Kii;
  PID::k_d = Kdi;
  PID::output_limit_max = output_lim_maxi;
  PID::output_limit_min = output_lim_mini;
  PID::prev_error = 0;
  PID::limit_flag = false;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  PID::p_error = cte;
//   if limit flag is true then no more integration of errors 
  PID::i_error = PID::limit_flag ? PID::i_error : PID::i_error + cte*PID::delta_t; // *0.5 using this ratio will help reduce the swirle 
  PID::d_error = PID::delta_t > 0 ? (PID::p_error - PID::prev_error)/PID::delta_t : 0;
  PID::prev_error = PID::p_error;
//   std::cout<< p_error << " P  " << d_error << "  d  " << i_error << "   i   " << std::endl;

}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
    control = -1*PID::p_error * PID::k_p -PID::d_error * PID::k_d - PID::i_error * PID::k_i; 
    if (control > PID::output_limit_max)
    {
       control = PID::output_limit_max;
       PID::limit_flag = true;
    }
    else if (control < PID::output_limit_min)
    {
       control = PID::output_limit_min;
       PID::limit_flag = true;
    }
    else 
    {
       PID::limit_flag = false;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  PID::delta_t = new_delta_time;
  return PID::delta_t;
}