#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/posix.h>
//#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
// #include <uORB/topics/vehicle_attitude.h> // to compare results
#include <poll.h>
#include <vector>
#include "ck_filters.h"

using std::vector;

extern "C" __EXPORT int test_app_main(int argc, char *argv[]);

int test_app_main(int argc, char *argv[])
{
	PX4_INFO("Testing Attitude Estimator");

	// subscribe to topics
	int gps_fd = orb_subscribe(ORB_ID(sensor_gps));
	int mag_fd = orb_subscribe(ORB_ID(sensor_mag));
	int gyro_fd = orb_subscribe(ORB_ID(sensor_gyro));
	int accel_fd = orb_subscribe(ORB_ID(sensor_accel));

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

	double accel_x_prev{0.0};


	//for (int i = 0; i < 1000; i++) {
	while (true) {
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
							orb_copy(ORB_ID(sensor_mag), mag_fd, &raw_mag);
							mag_x = (double)raw_mag.x;
							mag_y = (double)raw_mag.y;
							mag_z = (double)raw_mag.z;
							dt_mag = ((double)raw_mag.timestamp - mag_t)/1e6;
							mag_t = (double)raw_mag.timestamp;
							//PX4_INFO("Mag reading: %8.4f", (double)raw_mag.x);
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
							orb_copy(ORB_ID(sensor_accel), accel_fd, &raw_accel);
							accel_x = raw_accel.x;
							accel_y = raw_accel.y;
							accel_z = raw_accel.z;
							dt_accel = ((double)raw_accel.timestamp - accel_t)/1e6;
							accel_t = (double)raw_accel.timestamp;
							PX4_INFO("ACCEL X: %8.4f", (double)raw_accel.x);
							PX4_INFO("ACCEL DT: %f", dt_accel);
							double accel_x_lpf = CK_Fun::lowPassFilter(dt_accel, 10*dt_accel, accel_x, accel_x_prev);
							accel_x_prev = accel_x;
							PX4_INFO("Filtered accel: %f", accel_x_lpf);
							break;
					}

					//LEFT OFF: finish filtering data? Then find the attitude,
					// figure out how to put different sensors together
					// update attitude and position estimate here

					// filter data first
					CK_Fun::unitCrossProduct(vector<double> x, vector<double> y);
					// direction cosines

					// find the quaternion using mag vec and accel vec

					// integrate omega vec

					// combine those two

					// position with gps (& integrated accel?)

					}
				}

		}

	}

	return 0;
}

// Notes:
// This does not add additional time to dt to account for the time to
// extract the sensor data
