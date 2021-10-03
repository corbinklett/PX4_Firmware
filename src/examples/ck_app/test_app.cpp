#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/sensor_combined.h>
#include <poll.h>

extern "C" __EXPORT int test_app_main(int argc, char *argv[]);

int test_app_main(int argc, char *argv[])
{
	PX4_INFO("This is the test applicashe");

	// PX4_INFO("Argv:");

	// for (int i = 0; i < argc; ++i) {
	// 	PX4_INFO("  %d: %s", i, argv[i]);
	// }

	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,	.events = POLLIN}
	};

	while (true) {
		int poll_ret = px4_poll(fds, 1, 1000);

		if (fds[0].revents & POLLIN) {
			struct sensor_combined_s raw;

			orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
			PX4_INFO("Acceleromater:\t%8.4f\t%8.4f\t%8.4f",
				(double)raw.accelerometer_m_s2[0],
				(double)raw.accelerometer_m_s2[0],
				(double)raw.accelerometer_m_s2[0]);
		}
	}

	return 0;
}
