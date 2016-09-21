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

#include <kalman/include/ConstantVelocityPositionFilter.hpp>
#include <AP_RangeScanner/AP_RangeScanner_6DOF.h>
#include <AP_Math/AP_Math.h>

class ApproachSensor {
public: 
	ApproachSensor(AP_RangeScanner_6DOF &ranger); 
	Vector3f get_desired_angles(); 
	Vector3f get_safest_point(); 
	void update(); 
private: 
	//Eigen::filter::ConstantVelocityPositionFilter<float> _front, _back, _right, _left, _bottom, _top;  
	AP_RangeScanner_6DOF &_range; 
	long long _last_range_reading; 
}; 
