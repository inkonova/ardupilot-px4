/*
	Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once 

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

#include "AC_Controller.h"

class AC_RateControl : public AC_Controller {
public: 
	AC_RateControl( ); 
	void input_roll_rate(float rate); 
	void input_pitch_rate(float rate); 
	void input_yaw_rate(float rate); 
	void input_throttle(float thr); 
	void input_measured_rates(const Vector3f &rates); 

	float get_motor_roll(void); 
	float get_motor_pitch(void); 
	float get_motor_yaw(void); 

	void update(float dt); 
private: 
	Vector3f _target_rate, _gyro_rate; 
	float _out_roll, _out_pitch, _out_yaw; 
}; 
