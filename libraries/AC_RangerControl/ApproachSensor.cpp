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

#if 0
#include "ApproachSensor.h"

#define SENSOR_Q 10
#define SENSOR_R 50
#define SENSOR_DT 1.0f

ApproachSensor::ApproachSensor(AP_RangeScanner_6DOF &r) : 
	_front(SENSOR_Q, SENSOR_R, SENSOR_DT),
	_back(SENSOR_Q, SENSOR_R, SENSOR_DT),
	_right(SENSOR_Q, SENSOR_R, SENSOR_DT),
	_left(SENSOR_Q, SENSOR_R, SENSOR_DT),
	_bottom(SENSOR_Q, SENSOR_R, SENSOR_DT),
	_top(SENSOR_Q, SENSOR_R, SENSOR_DT),
	_range(r){

}

void ApproachSensor::update(){
	long long last_reading = _range.last_update_millis(); 
	if(_last_range_reading != last_reading){	
		float front, back, right, left, bottom, top; 
		_range.get_readings_m(&front, &back, &right, &left, &bottom, &top); 

/*
		if(front > 1.5f) front = 1.5f; 
		if(back > 1.5f) back = 1.5f; 
		if(right > 1.5f) right = 1.5f; 
		if(left > 1.5f) left = 1.5f; 
		if(bottom > 1.5f) bottom = 1.5f; 
		if(top > 1.5f) top = 1.5f; 
*/
		_front.predict(); 
		_back.predict(); 
		_right.predict(); 
		_left.predict(); 
		_bottom.predict(); 
		_top.predict(); 

		if(front > 0.3) _front.input_position(front); 
		if(back > 0.3) _back.input_position(back); 
		if(right > 0.3) _right.input_position(right); 
		if(left > 0.3) _left.input_position(left); 
		if(bottom > 0.3) _bottom.input_position(bottom); 
		if(top > 0.3) _top.input_position(top); 

		_last_range_reading = last_reading; 
	}
}

Vector3f ApproachSensor::get_safest_point(){
	float x = 0, y = 0, z = 0; 
	Eigen::Matrix<float, 2, 1> px, py; 
	px = _front.get_prediction(); 
	py = _back.get_prediction(); 
	x = (px(0) - py(0)) * 0.5f; 
	px = _right.get_prediction(); 
	py = _left.get_prediction(); 
	y = (px(0) - py(0)) * 0.5f; 
	px = _bottom.get_prediction(); 
	py = _top.get_prediction(); 
	z = (px(0) - py(0)) * 0.5; 
	return Vector3f(x, y, z); 
}

Vector3f ApproachSensor::get_desired_angles(){
	float roll = 0, pitch = 0, yaw = 0; 
	float scale = 3.0f; 
	float min_thresh = 1.5f; 
	Eigen::Matrix<float, 2, 1> px, py; 
	px = _front.get_prediction(); 
	py = _back.get_prediction(); 
	pitch += -px(1) * scale; 
	pitch += py(1) * scale; 
	//if(px(0) < min_thresh && py(0) < min_thresh) pitch *= 0.5;
	px = _left.get_prediction(); 
	py = _right.get_prediction(); 
	roll += -px(1) * scale; 
	roll += py(1) * scale;
	//if(px(0) < min_thresh && py(0) < min_thresh) roll *= 0.5;
		
	//roll = constrain_float(roll * roll, -1.0f, 1.0f); 
	//pitch = constrain_float(pitch * pitch, -1.0f, 1.0f); 
	//if(fabsf(roll) < 0.05) roll = 0; 
	//if(fabsf(pitch) < 0.05) pitch = 0; 

	return Vector3f(roll, pitch, yaw); 
}
#endif
