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
#include "AttitudeEstimatorOld.h"

static const matrix::Matrix<float, 3, 3> I = matrix::Matrix<float, 3, 3>::Identity(); 

static matrix::Matrix<float, 3, 3> _crossSkew(const matrix::Vector3f &w){
	matrix::Matrix<float, 3, 3> W;
	
	W(0, 0) = 0;
	W(0, 1) = -w(2);
	W(0, 2) = w(1);

	W(1, 0) = w(2);
	W(1, 1) = 0;
	W(1, 2) = -w(0);

	W(2, 0) = -w(1);
	W(2, 1) = w(0);
	W(2, 2) = 0;

	return W;
}

static matrix::Quaternion<float> _quatFromAccel(const matrix::Vector3f &acc){
	// attitude estimation based on application note AN3461
	// more information: http://stackoverflow.com/questions/3755059/3d-accelerometer-calculate-the-orientation
	float miu = 0.001f; 
	float sign = (acc(2) > 0.0f)?1.0f:-1.0f; 
	float roll = atan2(acc(1), sign * sqrt(acc(2) * acc(2) + miu * acc(0) + acc(0)));
	float pitch = atan2(-acc(0), sqrt(acc(1) * acc(1) + acc(2) * acc(2)));
	return matrix::Quaternion<float>(matrix::Vector3f(0, 1, 0), roll) * 
		matrix::Quaternion<float>(matrix::Vector3f(1, 0, 0), pitch);   
}

AttitudeEstimator::AttitudeEstimator(){
	// start with a large uncertainty
	P.setIdentity(); 
	P = P * (M_PI*M_PI); 

	aCov = matrix::Matrix<float, 3, 3>((const float[3][3]){
		{ 100.0, 0.0, 0.0 }, 
		{ 0.0, 100.0, 0.0 }, 
		{ 0.0, 0.0, 100.0 }
	}); 
	wCov.setIdentity(); 
	_is_stable = true; 
}
/*
static matrix::Quaternion<float> _applyGyroRates(const matrix::Quaternion<float> &q, const matrix::Vector3f &gyro, float dt){
	matrix::Quaternion<float> w(0, gyro(0), gyro(1), gyro(2)); 
	// euler quaternion integration
	//matrix::Quaternion<float> r = q + (0.5f * q * w) * dt; 
	// RK4
	const static float half = static_cast<float>(0.5);
	const static float two = static_cast<float>(2);

	matrix::Quaternion<float> qw = q * w * half;
	matrix::Quaternion<float> k2 = (q + qw * dt * half) * w * half;
	matrix::Quaternion<float> k3 = (q + k2 * dt * half) * w * half;
	matrix::Quaternion<float> k4 = (q + k3 * dt) * w * half;

	matrix::Quaternion<float> r = q + (qw + k2 * two + k3 * two + k4) * (dt / 6);
	r.normalize(); 
	return r; 
}
*/
void AttitudeEstimator::input_measured_gyro_rates(const matrix::Vector3f &gyro){
	if(isnan(gyro(0)) || isnan(gyro(1)) || isnan(gyro(2))) return;  
	// TODO: estimate or update bias

	_w = gyro - _gyro_bias; 
}

void AttitudeEstimator::input_measured_acceleration(const matrix::Vector3f &accel){
	if(isnan(accel(0)) || isnan(accel(1)) || isnan(accel(2))) return;  
	_a = accel; 
}

void AttitudeEstimator::input_measured_magnetic_field(const matrix::Vector3f &mag){
	if(isnan(mag(0)) || isnan(mag(1)) || isnan(mag(2))) return;  
	_m = mag; 
}

const matrix::Quaternion<float> &AttitudeEstimator::get_estimated_orientation(){
	return _q; 
}

#include <AP_Math/AP_Math.h>

void AttitudeEstimator::update(float dt){
	// -- predict
	// calculate error state jacobian (state transition matrix)
	F = I - _crossSkew(_w * dt); 

	// add gyro rates to quaternion
	//_q = _applyGyroRates(_q, _w, dt); 

	// noise jacobian 
	matrix::Matrix<float, 3, 3> G = -I * dt; 
	matrix::Matrix<float, 3, 3> Q = G * wCov * G.transposed(); 

	// update system covariance
	P = F * P * F.transposed() + Q; 

	// -- observe 
	// predict acceleration vector in earth frame based on current rotation prediction
	matrix::Vector3f gpred = _q * _a;  

	// calculate jacobian
	matrix::Matrix<float, 3, 3> H = _crossSkew(gpred); 

	// calculate innovation
	//matrix::Vector3f innovation = aPred - _a; 
	matrix::Vector3f innovation = gpred - matrix::Vector3f(0, 0, -9.82); 
	matrix::Matrix<float, 3, 3> innovation_cov = H * P * H.transposed() + aCov; 

	// prevent disaster 
	/* no determinant function in lib yet
	if(innovation_cov.determinant() < 1e-5f){
		_is_stable = false; 
		return; 
	} else {
		_is_stable = true; 
	}
	*/

	// -- update
	// calculate kalman gain
	matrix::Matrix<float, 3, 3> K = P * H.transposed() * matrix::inversed(innovation_cov); 

	// use approximation to update orientation quaternion 
	matrix::Vector3f dw = K * innovation; 

	matrix::Quaternion<float> w(0, _w(0), _w(1), _w(2)); 
	matrix::Quaternion<float> qw = (0.5f * _q * w); 

	//_q = _q + qw * dt; 
	//_q.normalize(); 
	_q.applyRates(_w, dt); 

	//if(_w.norm() < 0.00f){
	//if(fabsf(_a.norm() - 9.82) < 0.5){
	
	{
		matrix::Vector3f n = _a.normalized(); 
		matrix::Vector3f g = matrix::Vector3f(0, 0, -1); 
		float d = n.dot(g); 
		matrix::Quaternion<float> qacc; 

		float theta = acos(d); 	
		matrix::Vector3f rx = n.cross(g); 
		qacc.from_axis_angle(rx, theta); 
	
		static long long time = 0; 
		if((_a(0) * _a(0) + _a(1) * _a(1)) < 0.1){
			if((time + 500) < AP_HAL::millis()) {
			} 
			{
				printf(">>> correcting error of %f degrees\n", degrees(theta));  
				_q = _q * matrix::slerp(matrix::Quatf(), qacc, 0.2f, 0.001f); 
			}
		} else {
			time = AP_HAL::millis(); 
		}
		//_q = matrix::slerp(_q, qacc, gain); 
		//_q.normalize(); 
	}
	#if 0
	if(fabsf(_a.norm() - 9.82) < 0.5){
		matrix::Vector3f _agnorm = (_q * _a).normalized(); 
		float theta = acos(_agnorm.dot(matrix::Vector3f(0, 0, -1))); 
		matrix::Vector3f rx = _agnorm.cross(matrix::Vector3f(0, 0, -1)); 
		matrix::Quaternion<float> corr(rx, theta); 
	
		// correct attitude using a small fraction of the correction quaternion
		_q = matrix::lerp(_q, corr, 0.1f); 
		_q.normalize(); 
	} 
	#endif
	//_q = _q * matrix::Quaternion<float>(1, dw(0) * 0.5f, dw(1) * 0.5f, dw(2) * 0.5f); 
	//_q.normalize(); 

	// update system covariance
	P = P - K * H * P; 
}
#endif
