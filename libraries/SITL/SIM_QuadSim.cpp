/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  simulator connector for ardupilot version of QuadSim
*/

#include "SIM_QuadSim.h"

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

QuadSim::QuadSim(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str)
    //sock(true)
{
	_frame = NULL; 
	_mode = MODE_CLIENT_SIM; 
	_frame_type = FRAME_QUAD_X; 
	_packet_timeout = 0; 
	if(frame_str){
		printf("Looking up frame %s\n", frame_str); 
		_frame = Frame::find_frame("x");
		if (_frame == NULL) {
			printf("Frame '%s' not found", frame_str);
			exit(1);
		}
		_frame->init(1.5, 0.5, 60, 4*radians(360));
		frame_height = 0.1;
		ground_behavior = GROUND_BEHAVIOR_NONE;
	}
    // try to bind to a specific port so that if we restart ArduPilot
    // QuadSim keeps sending us packets. Not strictly necessary but
    // useful for debugging
    //sock.bind("127.0.0.1", 9005);

    //sock.reuseaddress();
    //sock.set_blocking(false);

	quadsim_stdout = -1; 
}

static bool _expect(int fd, const char *str){
    const char *basestr = str;
    while (*str) {
        char c;
        if (read(fd, &c, 1) != 1) {
            return false;
        }
        if (c == *str) {
            str++;
        } else {
            str = basestr;
        }
    }
    return true;
}

bool QuadSim::start_sim(void){ 
	int p[2];
	if(quadsim_stdout > 0) return true; 

	int devnull = open("/dev/null", O_RDWR);
	if (pipe(p) != 0) {
		AP_HAL::panic("Unable to create pipe");
	}

	// setup shared memory!

	int out = shmget(9003, sizeof(struct server_packet), 0666); 
	if(out < 0) perror("shmget"); 
	int in = shmget(9005, sizeof(struct client_packet), 0666); 
	if(in < 0) perror("shmget"); 

	_shmin = (char*)shmat(in, NULL, 0); 
	_shmout = (char*)shmat(out, NULL, 0); 

	memset(_shmin, 0, sizeof(struct client_packet)); 

	pid_t child_pid = fork();
	if (child_pid == 0) {
		// in child
		setsid();
		dup2(p[1], 0);
		dup2(p[1], 1);
		close(p[0]);
		for (uint8_t i=3; i<100; i++) {
			close(i);
		}
		
		if (chdir("../quadsim") != 0) {
			perror("quadsim");
			exit(1);
		}

		int ret = execlp("./start-quadsim.sh", NULL);
		if (ret != 0) {
			perror("quadsim");
		}
		exit(1);
	}
	close(p[1]);
	quadsim_stdout = p[0];

	sleep(1); 
	/*
	// read startup to be sure it is running
	char c;
	if (read(quadsim_stdout, &c, 1) != 1) {
		AP_HAL::panic("Unable to start quadsim");
	}

    if (!_expect(quadsim_stdout, "Irrlicht Engine version")) {
        AP_HAL::panic("Failed to start QuadSim");
    }*/

	fcntl(quadsim_stdout, F_SETFL, fcntl(quadsim_stdout, F_GETFL, 0) | O_NONBLOCK);

	close(devnull);

	return true;
}

void QuadSim::send_state(const struct sitl_input &input){
    server_packet pkt;
	memset(&pkt, 0, sizeof(pkt)); 

	for(int c = 0; c < 8; c++){
		pkt.servo[c] = input.servos[c]; 
	}

	float r, p, y;
	dcm.to_euler(&r, &p, &y);

	Vector3f velocity = dcm.transposed() * velocity_ef; 
	pkt.mode = _mode; 
	pkt.frame = (uint8_t)_frame_type; 
	pkt.euler[0] = r; pkt.euler[1] = p; pkt.euler[2] = y; 
	pkt.pos[0] = position.x; pkt.pos[1] = position.y; pkt.pos[2] = position.z; 
	pkt.vel[0] = velocity.x; pkt.vel[1] = velocity.y; pkt.vel[2] = velocity.z; 
	pkt.acc[0] = accel_body.x; pkt.acc[1] = accel_body.y; pkt.acc[2] = accel_body.z; 
	pkt.mag[0] = mag_bf.x; pkt.mag[1] = mag_bf.y; pkt.mag[2] = mag_bf.z; 

	memcpy(_shmout, &pkt, sizeof(pkt)); 

	printf("Servos: "); 
	for(int c = 0; c < 8; c++){
		printf("%d: %d, ", c, input.servos[c]); 
	}
	printf("\n"); 
}

#include <errno.h>

void QuadSim::drain_control_socket()
{
	/*
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    do {
        received = sock.recv(buf, buflen, 0);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                fprintf(stderr, "error recv on control socket: %s",
                        strerror(errno));
            }
        } else {
            // fprintf(stderr, "received from control socket: %s\n", buf);
        }
    } while (received > 0);
	*/
}

void QuadSim::recv_fdm(const struct sitl_input &input){

}

/*
  update the QuadSim simulation by one time step
 */
void QuadSim::update(const struct sitl_input &input){
	long long tnow = AP_HAL::millis(); 

	if(tnow > _packet_timeout){ 
    	send_state(input);
		_packet_timeout = tnow + 10; 
	}

	if(!start_sim()) return; 

	client_packet pkt;
	memset(&pkt, sizeof(pkt), 0); 

	static long long _calls_since = 0; 
	_calls_since++; 

	memcpy(&pkt, _shmin, sizeof(client_packet)); 

	if(_mode == MODE_CLIENT_SIM){
		accel_body = Vector3f(pkt.accel[0], pkt.accel[1], pkt.accel[2]); 
		gyro = Vector3f(pkt.gyro[0], pkt.gyro[1], pkt.gyro[2]); 
		location.lat = pkt.loc[0]; 
		location.lng = pkt.loc[1]; 
		location.alt = pkt.loc[2]; 
		mag_bf = Vector3f(pkt.mag[0], pkt.mag[1], pkt.mag[2]); 
		//printf("acc(%f %f %f)\n", accel_body.x, accel_body.y, accel_body.z); 
		position = Vector3f(pkt.pos[0], pkt.pos[1], pkt.pos[2]); 
		velocity_ef = Vector3f(pkt.vel[0], pkt.vel[1], pkt.vel[2]);
	}

	//::printf("\033[H\033[2J\n"); 
	/*
	if(pkt.id != (_last_packet_id + 1)) ::printf("DROPPED %d packets!\n", pkt.id - _last_packet_id); 
	else ::printf("packet %d\n", pkt.id); 
	_last_packet_id = pkt.id; 
	::printf("calls since last packet: %d\n", _calls_since); 
	_calls_since = 0; 
	::printf("acc(%f %f %f)\n", accel_body.x, accel_body.y, accel_body.z); 
	::printf("ax: %f\t%f\nay: %f\t%f\naz: %f\t%f\n", pkt.accel[0], accel_body.x, pkt.accel[1], accel_body.y, pkt.accel[2], accel_body.z); 
	::printf("gx: %f\t%f\ngy: %f\t%f\ngz: %f\t%f\n", pkt.gyro[0], gyro.x, pkt.gyro[1], gyro.y, pkt.gyro[2], gyro.z); 
	::printf("mx: %f\nmy: %f\nmz: %f\n", pkt.mag[0], pkt.mag[1], pkt.mag[2]); 
	::printf("pos: %f %f %f\n", position.x, position.y, position.z); 
	::printf("vel: %f %f %f\n", velocity_ef.x, velocity_ef.y, velocity_ef.z); 
	::printf("loc: %d %d %d\n", location.lat, location.lng, location.alt); 
	::printf("eu: %f %f %f\n", pkt.euler[0], pkt.euler[1], pkt.euler[2]); 
	::printf("6dof: %f %f %f %f %f %f\n", pkt.range[0], pkt.range[1], pkt.range[2], pkt.range[3], pkt.range[4], pkt.range[5]); 
	Vector3f a = dcm * accel_body; 	
	::printf("accelef: %f %f %f\n", a.x, a.y, a.z); 
	//::printf("acc(%f %f %f)\n", accel_body.x, accel_body.y, accel_body.z); 
	::printf("pos_sitl: %f %f %f\n", position.x, position.y, position.z); 
	::printf("6dof: %f %f %f %f %f %f\n", pkt.range[0], pkt.range[1], pkt.range[2], pkt.range[3], pkt.range[4], pkt.range[5]); 
*/
	rcin_chan_count = 8; 
	for(unsigned c = 0; c < 8; c++) rcin[c] = pkt.rcin[c]; 

	memcpy(scan6dof, pkt.range, sizeof(scan6dof)); 

	adjust_frame_time(1000);

	if(_mode == MODE_SERVER_SIM){
		// get wind vector setup
		update_wind(input);

		Vector3f rot_accel;

    	_frame->calculate_forces(*this, input, rot_accel, accel_body);

		update_dynamics(rot_accel);

		// update lat/lon/altitude
		update_position();

		// update magnetic field
		update_mag_field_bf();

		update_time();
	} else if(_mode == MODE_CLIENT_SIM){
		update_wind(input); 

		// velocity relative to air mass, in earth frame
		velocity_air_ef = velocity_ef - wind_ef;
		
		// velocity relative to airmass in body frame
		velocity_air_bf = dcm.transposed() * velocity_air_ef;
		
		// airspeed 
		airspeed = velocity_air_ef.length();

		// airspeed as seen by a fwd pitot tube (limited to 120m/s)
		airspeed_pitot = constrain_float(velocity_air_bf * Vector3f(1, 0, 0), 0, 120);

		update_time();
	}

	drain_control_socket(); 
}

} // namespace SITL
