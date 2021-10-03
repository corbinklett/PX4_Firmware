#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/posix.h>
//#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_attitude.h> // to compare results

#include <poll.h>

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

	double gps_lat;//, gps_long; // Degrees
	// double mag_x, mag_y, mag_z; // Gauss
	// double omega_x, omega_y, omega_z; // rad/sec
	// double accel_x, accel_y, accel_z; // m/s/s


	for (int i = 0; i < 1000; i++) { // while(true)
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
								gps_lat = (double)raw_gps.lat/1e-7;
								PX4_INFO("GPS LAT: %f", gps_lat);
								break;
							case 1: // Mag
								orb_copy(ORB_ID(sensor_mag), mag_fd, &raw_mag);
								PX4_INFO("Mag reading: %8.4f", (double)raw_mag.x);
								break;
							case 2: // Gyro
								orb_copy(ORB_ID(sensor_gyro), gyro_fd, &raw_gyro);
								PX4_INFO("Gyro %8.4f", (double)raw_gyro.x);
								break;
							case 3: // Accel
								orb_copy(ORB_ID(sensor_accel), accel_fd, &raw_accel);
								PX4_INFO("ACCEL X: %8.4f", (double)raw_accel.x);
								break;
						}

						// update attitude and position estimate here

						// struct sensor_accel_s raw;
						// orb_copy(ORB_ID(sensor_accel), accel_fd, &raw);
						// PX4_INFO("%8.4f", (double)raw.x);
						// PX4_INFO("Acceleromater:\t%8.4f\t%8.4f\t%8.4f",
						// (double)raw.accelerometer_m_s2[0],
						// (double)raw.accelerometer_m_s2[0],
						// (double)raw.accelerometer_m_s2[0]);
					}
				}

		}

	}

	// while (true) {
	// 	//int poll_ret = px4_poll(fds, 1, 1000);

	// 	if (fds[0].revents & POLLIN) {
	// 		struct sensor_combined_s raw;

	// 		orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
	// 		PX4_INFO("Acceleromater:\t%8.4f\t%8.4f\t%8.4f",
	// 			(double)raw.accelerometer_m_s2[0],
	// 			(double)raw.accelerometer_m_s2[0],
	// 			(double)raw.accelerometer_m_s2[0]);
	// 	}
	// }

	return 0;
}
