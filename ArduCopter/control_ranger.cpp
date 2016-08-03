#include "Copter.h"

// same controller as the stabi
bool Copter::control_ranger_init(bool ignore_checks){
	hal.console->printf("Ranger mode init.\n"); 
	stabilize_init(ignore_checks); 
	range_avoid.reset(); 	

#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Alt Hold if the Rotor Runup is not complete
    if (!ignore_checks && !motors.rotor_runup_complete()){
        return false;
    }
#endif

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    // stop takeoff if running
    takeoff_stop();

	return true; 
}

FILE *logfile = 0; 

// stabilize_ptilt_run - runs the main stabilize controller
// pitch is not sent to the pid, rather it is later calcualted by the mixer from rc input and sent directly to the servo
// roll and yaw compensation however needs to be done here. 
// otherwise it does pretty much the same job as the stabilization controller
// should be called at 100hz or more
void Copter::control_ranger_run()
{
    float target_roll, target_pitch;
    float target_yaw, throttle;

	attitude_control.enable(false); 

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
	
    get_pilot_desired_angle_rates(channel_roll->get_control_in(), channel_pitch->get_control_in(), channel_yaw->get_control_in(), target_roll, target_pitch, target_yaw);

	target_roll = channel_roll->get_control_in() / 4500.0f; 
	target_pitch = channel_pitch->get_control_in() / 4500.0f; 
	target_yaw = channel_yaw->get_control_in() / 4500.0f; 
	throttle = channel_throttle->get_control_in() / 1000.0f; 

	//::printf("%f %f %f %f\n", target_roll, target_pitch, target_yaw, throttle); 
	/*
	_rate_control.input_roll_rate(target_roll);
	_rate_control.input_pitch_rate(target_pitch); 
	_rate_control.input_yaw_rate(target_yaw); 
	_rate_control.input_throttle(throttle); 
	_rate_control.update(G_Dt); 
	*/

	_angle_control.input_roll_angle(radians(target_roll * 45.0));
	_angle_control.input_pitch_angle(radians(target_pitch * 45.0)); 
	_angle_control.input_yaw_angle_rate(radians(target_yaw * 45.0)); 
	_angle_control.input_throttle(throttle); 

	_angle_control.update(G_Dt); 
	_rate_control.update(G_Dt); 

#if 0
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (channel_throttle->get_control_in() > get_takeoff_trigger_throttle()) && motors.rotor_runup_complete());
#else
    bool takeoff_triggered = (ap.land_complete && (channel_throttle->get_control_in() > get_takeoff_trigger_throttle()) && motors.spool_up_complete());
#endif

    AltHoldModeState althold_state;

    // Alt Hold State Machine Determination
    if (!motors.armed() || !motors.get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (!ap.auto_armed){
        althold_state = AltHold_NotAutoArmed;
    } else if (takeoff_state.running || takeoff_triggered){
        althold_state = AltHold_Takeoff;
    } else if (ap.land_complete){
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // get pilot's desired throttle
    //pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());
#if 0 
	//Vector2f velocity = optflow.bodyRate(); 
	//Vector2f flow_rate = optflow.flowRate(); 
	// INKO_TILT: here we have to feed pilot rc control 0 because we will then apply rc control to the servo directly. 
	// INKO_TILT: in tilt mode, motor speeds are not effected by pilot pitch input. 
	float rc_4 = constrain_float((hal.rcin->read(4) - 1000.0), 0, 1000) * 0.001; 
	float rc_5 = constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.001; 

	if(!is_zero(rc_5)){
		float ku = 10 + rc_4 * 35;  
		float tu = 5 + rc_5 * 15; 

		float rc_vel_p = constrain_float((hal.rcin->read(4) - 1000.0), 0, 1000) * 0.08; // 6.32
		float rc_vel_i = 0;//constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.008; // 5.92 
		float rc_vel_d = constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.008; // 3.73 
		
		float rc_center_p = 0.6 * ku; 
		float rc_center_i = 0.8; //(1.2 * ku) / tu; //constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.01; 
		float rc_center_d = (0.6 * ku * tu) / 8; //constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.008; 

		::printf("%f, %f, %f, %f, %f\n", ku, tu, rc_center_p, rc_center_i, rc_center_d); 

		range_avoid.set_vel_kP(rc_vel_p); 
		range_avoid.set_vel_kI(rc_vel_i); 
		range_avoid.set_vel_kD(rc_vel_d); 
		
		range_avoid.set_center_kP(rc_center_p); 
		range_avoid.set_center_kI(rc_center_i); 
		range_avoid.set_center_kD(rc_center_d); 
	}

	::printf("state %d\n", althold_state); 
	if(althold_state == AltHold_Flying){
		/*if(!logfile) {
			logfile = fopen("/fs/microsd/log.hex", "a"); 
			if(logfile) printf("Opened logfile\n"); 
		}
		*/
		range_avoid.input_desired_velocity_ms(-target_pitch / 4500.0, target_roll / 4500.0); 

		range_avoid.update(1.0 / 400.0); 

		/** BLACK BOX LOGGING HERE. Should not be in production! */
		/*
		long long last_reading = rangefinders.last_update_millis(); 
		static long long _last_range_reading = 0;  
		if(_last_range_reading != last_reading){	
			struct frame {
				float fx, front, back, vx; 
				float fy, right, left, vy; 
				float bottom; //9 
				float ax, ay, az;//10 
				float gx, gy, gz;//13 
				float px, py, pz;//16 
				float motor_front_left, motor_front_right, motor_back_right, motor_back_left; 
				float pilot_throttle, pilot_yaw, pilot_pitch, pilot_roll; 
			}; 
			struct frame f; 
			Vector2f flow = optflow.flowRate(); 
			float top; 
			rangefinders.get_readings_m(&f.front, &f.back, &f.right, &f.left, &f.bottom, &top); 
			motors.get_motor_outputs(&f.motor_front_left, &f.motor_front_right, &f.motor_back_right, &f.motor_back_left);  
			f.pilot_throttle = channel_throttle->get_control_in();
			f.pilot_yaw = target_yaw_rate; 
			f.pilot_pitch = target_pitch * 0.01; 
			f.pilot_roll = target_roll * 0.01; 
			Vector3f pos_ef = ranger_nav.get_position_ef(); 
			Vector3f vel = ranger_nav.get_velocity(); 
			Vector3f accel = ins.get_accel(); 
			Vector3f gyro = ins.get_gyro(); 
			f.fx = flow.x; 
			f.fy = flow.y; 
			f.vx = vel.x; 
			f.vy = vel.y;
			f.ax = accel.x; f.ay = accel.y; f.az = accel.z; 
			f.gx = gyro.x; f.gy = gyro.y; f.gz = gyro.z; 
			f.px = pos_ef.x; 
			f.py = pos_ef.y; 
			f.pz = pos_ef.z; 
			char *data = (char*)&f; 
			for(size_t c = 0; c < sizeof(f); c++){
				fprintf(logfile, "%02x", (unsigned int)*(data+c)); 
			}
			fprintf(logfile, "\n"); 
			_last_range_reading = last_reading; 

			//hal.console->printf("%d %f %f %f\n", (int) althold_state, (double)f.bottom, (double)target_pitch * 0.01, (double)target_roll * 0.01); 
		}
		*/

		target_pitch = constrain_float(range_avoid.get_desired_pitch_angle(), -45.0f, 45.0f) * 100.0f; 
		target_roll = constrain_float(range_avoid.get_desired_roll_angle(), -45.0f, 45.0f) * 100.0f; 

	} else {
		/*
		if(logfile){
			fclose(logfile); 
			printf("Closed logfile\n"); 
			logfile = 0; 
		}
		*/
		range_avoid.reset(); 
	}

#if FRAME_CONFIG == QUAD_PTILT_FRAME
	float cosAngle = cos(radians(target_pitch * 0.01)); 

	float rollComp = target_roll * cosAngle;
	float rollCompInv = target_roll - rollComp;
	float yawComp = target_yaw_rate * cosAngle;
	float yawCompInv = target_yaw_rate - yawComp;
	target_roll = yawCompInv + rollComp; 
	target_yaw_rate = yawComp + rollCompInv; 

	target_pitch = constrain_float(target_pitch, -4500, 4500); 
	target_roll = constrain_float(target_roll, -4500, 4500);
	
	// support body pitch on channel 4 so we can adjust body tilting when we fly. 
	// this will also rotate the tilt motors in the opposite direction. 
	// TODO: make all of this stuff configurable
	//float bodyTilt = (hal.rcin->read(4) - 1500) / 500.0 * 90.0; 
	//bodyTilt = 0; // disable for now
	//float motorTilt = target_pitch * 0.01; 	

	//motors.set_motor_tilt(motorTilt - bodyTilt); 

	// zero the target_pitch because we will be handling forward movement using motor tilt
	//target_pitch = 0; 
#endif

    //attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, bodyTilt * 100, target_yaw_rate, get_smoothing_gain());

	// compensate throttle for motor tilt (throttle is 0-1.0 here)
	// compensating for rc input pitch + current body pitch because body may pitch as well and we need to adjust thrust for that as well 
	// total angle is constrained between 0 and 45 degrees. Beyond that we don't do compensation
	// the cos here is always between 0.7 and 1.0 so we don't need to worry about div by zero

	// disable throttle compensation for now because we are using altitude hold below
	//float compPitch = constrain_float(abs(ahrs.pitch_sensor * 0.01 + target_pitch * 0.01), 0, 45.0); 
	//pilot_throttle_scaled = pilot_throttle_scaled / cos(radians(compPitch)); 
#endif

	// ===========================================================
	// Run altitude hold logic here

    // output pilot's throttle
    //attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
    float takeoff_climb_rate = 0.0f;
	
    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // get pilot desired lean angles
    //get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    //float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
#if FRAME_CONFIG == HELI_FRAME    
        // helicopters are capable of flying even with the motor stopped, therefore we will attempt to keep flying
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // force descent rate and call position controller
        pos_control.set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        pos_control.update_z_controller();
#else
        // Multicopters do not stabilize roll/pitch/yaw when motor are stopped
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-motors.get_throttle_hover());
#endif
        break;

    case AltHold_NotAutoArmed:

        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
#if FRAME_CONFIG == HELI_FRAME
        // Helicopters always stabilize roll/pitch/yaw
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else
        // Multicopters do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-motors.get_throttle_hover());
        break;

    case AltHold_Takeoff:

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
        break;

    case AltHold_Landed:

#if FRAME_CONFIG == HELI_FRAME
        attitude_control.set_yaw_target_to_current_heading();
#endif
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(channel_throttle->get_control_in()),false,g.throttle_filt);
        // set motors to spin-when-armed if throttle at zero, otherwise full range
        if (ap.throttle_zero) {
            motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-motors.get_throttle_hover());
        break;

    case AltHold_Flying: {
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }
/*		
		static AC_PID _pid_z(0.7, 0.1, 0.01, 2.0, 1.0, 1.0/400.0); 
		static float _target_rate = 0; 
    	float pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());
		if(pilot_throttle_scaled < 0.4) _target_rate = fmax(_target_rate - G_Dt, 0.0f); 
		if(pilot_throttle_scaled > 0.6) _target_rate = fmin(_target_rate + G_Dt, 1.0f); 
		Vector3f vel = inertial_nav.get_velocity() * 0.01f; 
		_pid_z.set_input_filter_all(_target_rate - vel.z); 	
		float th = 0.5 + constrain_float(_pid_z.get_pid(), -0.5f, 0.5f); 
		
		::printf("vel: %f targ: %f th: %f, err: %f\n", vel.z, _target_rate, th, _target_rate - vel.z); 
    	attitude_control.set_throttle_out(th, true, g.throttle_filt);
*/
        // call position controller
        //pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        //pos_control.update_z_controller();
        break; }
    }
#endif
}
