#include "Copter.h"
#include <matrix/matrix/Euler.hpp>
//#include <kalman/include/ConstantVelocityPositionFilter.hpp>

// same controller as the stabi
bool Copter::control_ranger_init(bool ignore_checks){
	hal.console->printf("Ranger mode init.\n"); 
	stabilize_init(ignore_checks); 
	//range_avoid.reset(); 	

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
#if 0 
	attitude_control.enable(false); 

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

	float rc_4 = constrain_float((hal.rcin->read(4) - 1000.0), 0, 1000) * 0.001; 
	float rc_5 = constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.001; 

	float ku = rc_4 * 4.0;  
	float tu = rc_5 * 10.0f; 

	if(is_zero(tu)) tu = 0.01; 

	float kp = 0.6 * ku; 
	float ki = (1.2 * ku) / tu; //constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.01; 
	float kd = (0.6 * ku * tu) / 8; //constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.008; 

	::printf("ku/tu: %f %f, pid: %f %f %f\n", ku, tu, kp, ki, kd); 

	_pid.set_position_tuning(
		matrix::Vector3f(0.100, 0.100, 1.000), 
		matrix::Vector3f(0.000, 0.000, 0.000), 
		matrix::Vector3f(0.100, 0.100, 0.200) 
	); 
	_pid.set_velocity_tuning(
		matrix::Vector3f(0.079, 0.079, 0.480), 
		matrix::Vector3f(0.018, 0.018, 0.018), 
		matrix::Vector3f(0.083, 0.083, 0.120) 
	); 
	_pid.set_angle_tuning(
		matrix::Vector3f(4.000, 4.000, 1.000), 
		matrix::Vector3f(0.100, 0.100, 0.000), 
		matrix::Vector3f(0.031, 0.031, 0.010) 
	); 
	_pid.set_angular_velocity_tuning(
		matrix::Vector3f(0.600, 0.600, 1.000), 
		matrix::Vector3f(0.086, 0.086, 0.000), 
		//matrix::Vector3f(0.103, 0.103, 0.010)
		matrix::Vector3f(0.400, 0.103, 0.010)
	); 
	
	float target_roll = channel_roll->get_control_in() / 4500.0f; 
	float target_pitch = channel_pitch->get_control_in() / 4500.0f; 
	float target_yaw = channel_yaw->get_control_in() / 4500.0f; 
	float throttle = channel_throttle->get_control_in() / 1000.0f; 

	Vector3f vel; 
	// use gps velocity if available and if not then we use inertial (accellerometer) velocity 
	if(!ahrs.get_velocity_NED(vel)){
		::printf("No velocity!\n"); 
		vel = inertial_nav.get_velocity() * 0.01; 
	}

	Vector3f pos; 
	static Vector3f _target_pos; 
	if(!ahrs.get_relative_position_NED(pos)){
		::printf("No position!\n"); 
		pos = Vector3f(0, 0, 0); 
	} else if(is_zero(pos.length())){
		_target_pos = pos; 
	}

	// we will rotate velocity into body frame
	Quaternion qyaw = Quaternion(cos(ahrs.yaw / 2), 0, 0, sin(ahrs.yaw / 2)); 
	Quaternion qyaw_inv = qyaw.inversed(); 
	vel = qyaw_inv * vel; 

	Vector3f gyro = ins.get_gyro(); 
	Vector3f acc = ins.get_accel(); 
	
	_att.input_measured_gyro_rates(gyro.x, gyro.y, gyro.z); 
	_att.input_measured_acceleration(acc.x, acc.y, acc.z); 
	_att.update(G_Dt); 

	float qdata[4], w[3]; 
	_att.get_estimated_quaternion(qdata); 
	_att.get_estimated_omega(w); 
	matrix::Quaternion<float> q(qdata);  

	matrix::Euler<float> euler = matrix::eulerAngles(q); 
	printf("Att: %f %f %f, w: %f %f %f, ahrs: %f %f %f\n", 
		degrees(euler.roll()), degrees(euler.pitch()), degrees(euler.yaw()), 
		w[0], w[1], w[2],
		degrees(ahrs.roll), degrees(ahrs.pitch), degrees(ahrs.yaw)); 

	// convert throttle into up/down rate with max speed of 1m/s
	float thr = throttle - 0.5; 
	if(thr > -0.1f && thr < 0.1f) thr = 0; 
	float descend_velocity = constrain_float(thr * 2.0f, -1.0, 1.0); 

	//Vector3f sp; 
	//_obstacle_sensor.update(G_Dt); 
	//_obstacle_sensor.get_safest_position_offset(sp); 

	// input measured parameters
	_pid.input_measured_position(matrix::Vector3f(pos.x, pos.y, pos.z)); 
	_pid.input_measured_velocity(matrix::Vector3f(vel.x, vel.y, vel.z)); 
	_pid.input_measured_angles(matrix::Vector3f(ahrs.roll, ahrs.pitch, ahrs.yaw)); 
	//_pid.input_measured_angles(matrix::Vector3f(euler.roll(), euler.pitch(), euler.yaw())); 
	_pid.input_measured_angular_velocity(matrix::Vector3f(gyro.x, gyro.y, gyro.z)); 

	Vector3f tp = (qyaw * Vector3f(-target_pitch, target_roll, 0) * 10.0f + Vector3f(0, 0, -descend_velocity) * 4.0f); 

	/*
	if(!is_zero(sp.x) || !is_zero(sp.y) || !is_zero(target_pitch) || !is_zero(target_roll)){
		_target_pos = pos; 
		if(!is_zero(sp.x) || !is_zero(sp.y))
			_target_pos += (qyaw * Vector3f(sp.x, sp.y, 0)) * 2.0f; 
		if(!is_zero(target_pitch) || !is_zero(target_roll))
			_target_pos += Vector3f(tp.x, tp.y, 0); 
	}
*/
	if(!is_zero(target_pitch) || !is_zero(target_roll)){
		_target_pos = pos + Vector3f(tp.x, tp.y, 0); 
	}

	_pid.input_target_position(
		matrix::Vector3f(_target_pos.x, _target_pos.y, _target_pos.z) 
	); 

	matrix::Vector3f v = _pid.get_desired_velocity(); 
	Vector3f dvel = qyaw_inv * Vector3f(v(0), v(1), 0); 

	_pid.input_target_velocity(matrix::Vector3f(dvel.x, dvel.y, -descend_velocity)); 
	/*_pid.input_target_velocity(matrix::Vector3f(
		-(target_pitch * 20.0f - sp.x), 
		(target_roll * 20.0f + sp.y), 
		-descend_velocity)); 
	*/
	_approach.update(); 
	Vector3f u = _approach.get_desired_angles(); 
	//Vector3f c = _approach.get_safest_point(); 
	target_roll = target_roll * 3.0f; // + c.y * 1.5; 		
	target_pitch = target_pitch * 3.0f;// + -c.x * 1.5f; 		
	//target_roll += c.y * 0.2 + u.x; 
	//target_pitch += -c.x * 0.2 + u.y; 
	_pid.input_measured_velocity(matrix::Vector3f(u.y * 4.0f, -u.x * 4.0f, vel.z)); 
	_pid.input_target_velocity(matrix::Vector3f(
		-(target_pitch), 
		(target_roll), 
		-descend_velocity)); 

	matrix::Vector3f ta = _pid.get_desired_acceleration(); 
	_pid.input_target_angles(matrix::Vector3f(
		constrain_float(ta(1), radians(-45.0f), radians(45.0f)), 
		constrain_float(-ta(0), radians(-45.0f), radians(45.0f)), 
		0.0f));

	/*static FILE *logfile = 0; 
	if(!logfile) logfile = fopen("/tmp/log.log", "w"); 
	static long long _last_range_reading = 0; 
	long long last_reading = rangefinders.last_update_millis(); 
	if(_last_range_reading != last_reading){	
		float front, back, right, left, bottom, top; 
		rangefinders.get_readings_m(&front, &back, &right, &left, &bottom, &top); 
		fprintf(logfile, "%f, %f, %f, %f, %f, %f, %f, %f\n", front, back, right, left, u.x, u.y, c.x, c.y); 
		//fprintf(logfile, "%f, %f, %f, %f, %f, %f, %f, %f\n", front, back, right, left, bottom, top, p(0), p(1)); 
		_last_range_reading = last_reading; 
	}*/

	//_pid.input_target_angles(matrix::Vector3f(target_roll * radians(45.0f), target_pitch * radians(45.0f), 0.0f)); 
	//_pid.input_target_angles(matrix::Vector3f(ta(1), -ta(0), 0.0f)); 
	matrix::Vector3f ar = _pid.get_desired_angular_velocity(); 
	_pid.input_target_angular_velocity(matrix::Vector3f(ar(0), ar(1), target_yaw * 5.0f)); 
	_pid.update(G_Dt); 

	matrix::Vector3f rates = _pid.get_desired_angular_acceleration(); 
	motors.set_roll(rates(0)); 
	motors.set_pitch(rates(1)); 
	motors.set_yaw(rates(2)); 

	// convert throttle to 0-1.0 range 
	motors.set_throttle(-constrain_float(ta(2), -1.0f, 1.0f) * 0.5f + 0.5f); 
	//motors.set_throttle(throttle); 

	/*fprintf(logfile, "%f, %f, %f, %f, %f, %f\n", 
		degrees(euler.roll()), degrees(euler.pitch()), degrees(euler.yaw()), 
		degrees(ahrs.roll), degrees(ahrs.pitch), degrees(ahrs.yaw)); 	
		*/
	//fflush(logfile); 
#endif
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

//#if FRAME_CONFIG == QUAD_PTILT_FRAME
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
//#endif

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
