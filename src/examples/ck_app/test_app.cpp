#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <uORB/uORB.h>

#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/debug_key_value.h>
// #include <uORB/topics/vehicle_attitude.h> // to compare results
#include <poll.h>
#include <matrix/math.hpp>
#include <math.h>
#include <ctime>
#include <string.h>
#include "ck_filters.h"

#define PI 3.14159

extern "C" __EXPORT int test_app_main(int argc, char *argv[]);

int test_app_main(int argc, char *argv[])
{
	PX4_INFO("Testing Attitude Estimator");

	time_t start, finish;
	double time_diff;

	// subscribe to topics
	int gps_fd = orb_subscribe(ORB_ID(sensor_gps));
	int mag_fd = orb_subscribe(ORB_ID(sensor_mag));
	int gyro_fd = orb_subscribe(ORB_ID(sensor_gyro));
	int accel_fd = orb_subscribe(ORB_ID(sensor_accel));

	/* advertise named debug value */
	struct debug_key_value_s dbg_key;
	orb_advert_t pub_dbg_key = orb_advertise(ORB_ID(debug_key_value), &dbg_key);

	// limit update rate
	// orb_set_interval(sensor_sub_fd, 200);

	px4_pollfd_struct_t fds[] = {
		{ .fd = gps_fd,	.events = POLLIN},
		{ .fd = mag_fd, .events = POLLIN},
		{ .fd = gyro_fd,.events = POLLIN},
		{ .fd = accel_fd, .events = POLLIN}
	};

	struct sensor_gps_s raw_gps;
	struct sensor_mag_s raw_mag;
	struct sensor_gyro_s raw_gyro;
	struct sensor_accel_s raw_accel;

	double gps_lat, gps_long; // Degrees
	double mag_x, mag_y, mag_z; // Gauss
	double omega_x, omega_y, omega_z; // rad/sec
	double accel_x, accel_y, accel_z; // m/s/s

	double dt_gps, dt_mag, dt_omega, dt_accel;
	double gps_t{0.0}, mag_t{0.0}, omega_t{0.0}, accel_t{0.0};

	double accel_x_prev{0.0}, accel_y_prev{0.0}, accel_z_prev{0.0};
	matrix::Vector3<double> acceleration(0.0,0.0,0.0);

	double mag_x_prev{0.0}, mag_y_prev{0.0}, mag_z_prev{0.0};
	matrix::Vector3<double> magnet(0.0,0.0,0.0);

	bool new_accel{false};
	bool new_mag{false};

	// NED (unit) vectors expressed in body frame
	matrix::Vector3<double> X_b;
	matrix::Vector3<double> Y_b;
	matrix::Vector3<double> Z_b;

	// body frame
	matrix::Vector3<double> i_hat{1.0,0.0,0.0};
	matrix::Vector3<double> j_hat{0.0,1.0,0.0};
	matrix::Vector3<double> k_hat{0.0,0.0,1.0};


	double pitch, heading;

	//for (int i = 0; i < 1000; i++) {
	time(&start);

	while (true) {

	time(&finish);
	time_diff = difftime(finish, start);

		// wait for update of 4 fds for 1000ms
		int poll_ret = px4_poll(fds, 4, 1000);

		if (poll_ret <= 0) {
			PX4_ERR("Error at poll return - prob no data within time specified.");
		} else {
			for (int j = 0; j < 4; j++) {
			// hopefully this allows for different data rates (Hz)
				if (fds[j].revents & POLLIN) {

					switch(j) {
						case 0: // GPS
							orb_copy(ORB_ID(sensor_gps), gps_fd, &raw_gps);
							gps_lat = raw_gps.lat/10000000.0;
							gps_long = raw_gps.lon/10000000.0;
							dt_gps = ((double)raw_gps.timestamp - gps_t)/1e6;
							gps_t = (double)raw_gps.timestamp;
							// PX4_INFO("GPS LAT: %f===========================", gps_lat);
							// PX4_INFO("GPS LONG: %f===========================", gps_long);
							// PX4_INFO("GPS timestamp %f", dt_gps);
							break;
						case 1: // Mag
							new_mag = true;
							orb_copy(ORB_ID(sensor_mag), mag_fd, &raw_mag);
							mag_x = (double)raw_mag.x;
							mag_y = (double)raw_mag.y;
							mag_z = (double)raw_mag.z;
							dt_mag = ((double)raw_mag.timestamp - mag_t)/1e6;
							mag_t = (double)raw_mag.timestamp;
							magnet(0) = CK_Fun::lowPassFilter(dt_mag, 10*dt_mag, mag_x, mag_x_prev);
							mag_x_prev = mag_x;
							magnet(1) = CK_Fun::lowPassFilter(dt_mag, 10*dt_mag, mag_y, mag_y_prev);
							mag_y_prev = mag_y;
							magnet(2) = CK_Fun::lowPassFilter(dt_mag, 10*dt_mag, mag_z, mag_z_prev);
							mag_z_prev = mag_z;
							break;
						case 2: // Gyro
							orb_copy(ORB_ID(sensor_gyro), gyro_fd, &raw_gyro);
							omega_x = raw_gyro.x;
							omega_y = raw_gyro.y;
							omega_z = raw_gyro.z;
							dt_omega = ((double)raw_gyro.timestamp - omega_t)/1e6;
							omega_t = (double)raw_gyro.timestamp;
							// PX4_INFO("Gyro %8.4f", (double)raw_gyro.x);
							break;
						case 3: // Accel
							new_accel = true;
							orb_copy(ORB_ID(sensor_accel), accel_fd, &raw_accel);
							accel_x = raw_accel.x;
							accel_y = raw_accel.y;
							accel_z = raw_accel.z;
							dt_accel = ((double)raw_accel.timestamp - accel_t)/1e6;
							accel_t = (double)raw_accel.timestamp;
							//PX4_INFO("ACCEL X: %8.4f", (double)raw_accel.x);
							//PX4_INFO("ACCEL DT: %f", dt_accel);
							acceleration(0) = CK_Fun::lowPassFilter(dt_accel, 10*dt_accel, accel_x, accel_x_prev);
							accel_x_prev = accel_x;
							acceleration(1) = CK_Fun::lowPassFilter(dt_accel, 10*dt_accel, accel_y, accel_y_prev);
							accel_y_prev = accel_y;
							acceleration(2) = CK_Fun::lowPassFilter(dt_accel, 10*dt_accel, accel_z, accel_z_prev);
							accel_z_prev = accel_z;
							//if (time_diff >= 0.1) {
					 			//PX4_INFO("accel x: %f\ty: %f\tz: %f", acceleration(0), acceleration(1), acceleration(2));
								//time(&start);
							//}
							//PX4_INFO("Filtered accel: %f", acceleration(2));
							break;
					}

					//LEFT OFF: finish filtering data? Then find the attitude,
					// figure out how to put different sensors together
					// update attitude and position estimate here

					if (new_mag || new_accel) {
						Y_b = magnet.cross(acceleration);
						Y_b = Y_b.normalized();
						X_b = Y_b.cross(-acceleration);
						X_b = X_b.normalized();
						Z_b = X_b.cross(Y_b);
						Z_b = Z_b.normalized();
						// PX4_INFO("X_b x coordinate: %f", X_b(0));
						// PX4_INFO("X_b y coordinate: %f", X_b(1));
						// PX4_INFO("X_b z coordinate: %f", X_b(2));

						// find roll-pitch-yaw

						// Roll Angle - angle between j_hat and X_b-Y_b plane
						// roll = acos(pi/2 - )
						double roll = asin(j_hat.dot(Z_b));
						if (time_diff >= 0.5) {
					 		PX4_INFO("Roll angle (radians): %f", roll);
							time(&start);
						}

						strncpy(dbg_key.key, "roll", 10);
						dbg_key.value = roll;

						// publish
						orb_publish(ORB_ID(debug_key_value), pub_dbg_key, &dbg_key);

						// Pitch - 1-rotation of body frame by roll, then pitch is angle between i_hat_rot and X_b-Y_b plane
						double pitch = asin(i_hat.dot(Z_b));
						//PX4_INFO("Pitch angle (radians): %f", pitch);

						// heading: rotated X_b by pitch angle about Y_b
						double heading =

						strncpy(dbg_key.key, "pitch", 10);
						dbg_key.value = pitch;

						// publish
						orb_publish(ORB_ID(debug_key_value), pub_dbg_key, &dbg_key);


					}
					// find NED frame expressed in body frame
					// direction cosines

					// find the quaternion using mag vec and accel vec

					// integrate omega vec

					// combine those two

					// position with gps (& integrated accel?)

					// reset bool values
					new_accel = false;
					new_mag = false;

					}
				}

		}

	}

	return 0;
}

// Notes:
// This does not add additional time to dt to account for the time to
// extract the sensor data
// dt changes every time step, and this may effect the filter

// filter out magnetic disturbances
