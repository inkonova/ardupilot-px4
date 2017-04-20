/*
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

/*
 *       AP_MotorsOctaQuad.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include "AP_MotorsOctaQuad.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsOctaQuad::var_info[] = {
	// @Param: TILT_SERVO_ON
    // @DisplayName: Turms the tilt servo on or off
    // @Description: If off, then servo is centered
    // @Range: 0 1
    // @Increment: 1

	AP_GROUPINFO("ON", 0, AP_MotorsOctaQuad, _servo_on, 1),

	// @Param: TILT_SERVO_CHANNEL
    // @DisplayName: Channel for the tilt servo
    // @Description: Tilt servo signal will go on this channel (use values starting from 1!)
    // @Range: 1 32
    // @Increment: 1

	AP_GROUPINFO("CHANNEL", 1, AP_MotorsOctaQuad, _servo_channel, AP_MOTORS_MAX_NUM_MOTORS),

	// @Param: TILT_SERVO_TRAVEL
    // @DisplayName: Tilt servo travel in degrees in each direction
    // @Description: Specifies how far tilt servo is allowed to move
    // @Range: 0 90
    // @Increment: 1

	AP_GROUPINFO("TRAVEL", 2, AP_MotorsOctaQuad, _servo_travel, 45),

	AP_GROUPEND
};

// setup_motors - configures the motors for a octa
void AP_MotorsOctaQuad::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // hard coded config for supported frames
    if( _flags.frame_orientation == AP_MOTORS_PLUS_FRAME ) {
        // plus frame set-up
        add_motor(AP_MOTORS_MOT_1,    0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
        add_motor(AP_MOTORS_MOT_2,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
        add_motor(AP_MOTORS_MOT_3,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
        add_motor(AP_MOTORS_MOT_4,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
        add_motor(AP_MOTORS_MOT_5,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
        add_motor(AP_MOTORS_MOT_6,    0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
        add_motor(AP_MOTORS_MOT_7,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_8,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
    }else if( _flags.frame_orientation == AP_MOTORS_V_FRAME ) {
        // V frame set-up
        add_motor(AP_MOTORS_MOT_1,   45,  0.7981f, 1);
        add_motor(AP_MOTORS_MOT_2,  -45, -0.7981f, 7);
        add_motor(AP_MOTORS_MOT_3, -135,  1.0000f, 5);
        add_motor(AP_MOTORS_MOT_4,  135, -1.0000f, 3);
        add_motor(AP_MOTORS_MOT_5,  -45,  0.7981f, 8);
        add_motor(AP_MOTORS_MOT_6,   45, -0.7981f, 2);
        add_motor(AP_MOTORS_MOT_7,  135,  1.0000f, 4);
        add_motor(AP_MOTORS_MOT_8, -135, -1.0000f, 6);
    }else if( _flags.frame_orientation == AP_MOTORS_H_FRAME ) {
        // H frame set-up - same as X but motors spin in opposite directions
        add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7);
        add_motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
        add_motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8);
        add_motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        add_motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
        add_motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
    }else{
        // X frame set-up
        add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
        add_motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
        add_motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
        add_motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
        add_motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
        add_motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);

        // This is for the UFO Build,
        add_motor_raw(AP_MOTORS_MOT_9, 0.0f, 0.0f, 0.0f, 9);
        add_motor_raw(AP_MOTORS_MOT_10, 0.0f, 0.0f, 0.0f, 10);

    }

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();
}

void AP_MotorsOctaQuad::output(){
	// NOTE: all of this is really crappy and hardcoded. Just to make it work quick...

	float servo_scale = (constrain_float(_servo_travel, 0, 90) / 90.0f); // 0.0 - 1.0.
	float tilt_pitch = _tilt_pitch * 3.0f;
	uint16_t servo_pwm = constrain_int16(1500 + 500 * constrain_float(tilt_pitch / 90.0f, -1.0f, 1.0f), 1000, 2000);
	uint16_t inv_servo_pwm = constrain_int16(1500 + 500 * constrain_float(-tilt_pitch / 90.0f, -1.0f, 1.0f), 1000, 2000);

	if(_servo_on && _servo_channel > 0){
		rc_write(_servo_channel, servo_pwm);
		rc_write(_servo_channel + 1, inv_servo_pwm);
	} else {
		// center servo
		rc_write(_servo_channel, 1500);
		rc_write(_servo_channel + 1, 1500);
	}


	AP_MotorsMatrix::output();
}


