/*
 *	Project: Soldering station
 *	PID library - Khai Minh Ma
 */

#include "main.h"
#include "stdio.h"
#include "PID_lib.h"
//#include "i2c-lcd.h"

double kP, kI, kD;
double error, last_error;
double kI_error_sum, kD_error_der;

double kI_clamp_MIN, kI_clamp_MAX;
double output_MIN, output_MAX, output;

void PID_init()
{
	// set all working variable to 0
	error = 0;
	last_error = 0;

	kI_error_sum = 0;
	kD_error_der = 0;
}

void PID_set_gain(float kP_set, float kI_set, float kD_set)
{
	kP = kP_set;
	kI = kI_set;
	kD = kD_set;
}


void PID_set_clamp(double kI_clamp_MIN_set, double kI_clamp_MAX_set, double output_MIN_set, double output_MAX_set)
{
	kI_clamp_MIN = kI_clamp_MIN_set;
	kI_clamp_MAX = kI_clamp_MAX_set;

	output_MIN = output_MIN_set;
	output_MAX = output_MAX_set;
}

double PID_compute(double set_point, double input, double time_interval_sec) {

	// Calculate error from set_point and input
	error = set_point - input;


	// Calculate Integral part and clamp
	kI_error_sum += kI * error * time_interval_sec;

	if (kI_error_sum > kI_clamp_MAX){
		kI_error_sum = kI_clamp_MAX;
	}
	else if (kI_error_sum < kI_clamp_MIN){
		kI_error_sum = kI_clamp_MIN;
	}

	// Calculate Derivative part
	kD_error_der = kD * (error - last_error) / time_interval_sec;


	// Calculate the sum and clamp the PID output
	output = kP * error + kI_error_sum + kD_error_der;

	if (output > output_MAX) {
		output = output_MAX;
	}
	else if (output < output_MIN) {
		output = output_MIN;
	}

	last_error = error;

//	char display_pid[16];
//	sprintf(display_pid, "%.0f  ", kP * error);
//	lcd_goto_XY(3, 0);
//	lcd_send_string(display_pid);
//
//	sprintf(display_pid, "%.0f  ", kI_error_sum);
//	lcd_goto_XY(3, 5);
//	lcd_send_string(display_pid);
//
//	sprintf(display_pid, "%.0f  ", kD_error_der);
//	lcd_goto_XY(3, 10);
//	lcd_send_string(display_pid);

	return output;
}
