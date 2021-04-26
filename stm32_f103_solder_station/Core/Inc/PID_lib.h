/*
 *	Project: Soldering station
 *	PID library - Khai Minh Ma
 */

#ifndef PID_LIB_H
#define PID_LIB_H

void PID_init();

void PID_set_gain(float kP_set, float kI_set, float kD_set);

void PID_set_clamp(double kI_clamp_MIN_set, double kI_clamp_MAX_set, double output_MIN_set, double output_MAX_set);

double PID_compute(double set_point, double input, double time_interval_sec);


#endif
