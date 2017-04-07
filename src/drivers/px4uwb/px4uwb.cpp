/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4uwb.cpp
 * @author Henry Zhang
 * @author Ban Siesta <zhanghui629@gmail.com>
 *
 * Driver for the PX4UWB module connected via I2C.
 */

#include <px4_config.h>
#include <px4_defines.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <geo/geo.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_px4uwb.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <board_config.h>

/* Configuration Constants */
#define I2C_UWB_ADDRESS 		0x24	///< 7-bit address. 8-bit address is 0x84, range 0x42 - 0x49

/* PX4UWB Registers addresses */
#define PX4UWB_POS_X		0x30

#define PX4UWB_CONVERSION_INTERVAL	100000	///< in microseconds! 20000 = 50 Hz 100000 = 10Hz
#define PX4UWB_I2C_MAX_BUS_SPEED	400000	///< 400 KHz maximum speed


#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class PX4UWB: public device::I2C
{
public:
	PX4UWB(int bus, int address = I2C_UWB_ADDRESS);
	virtual ~PX4UWB();

	virtual int     init() override;

	virtual ssize_t read(struct file* filp, char* buffer, size_t buflen) override;

	virtual int     ioctl(struct file* filp, int cmd, unsigned long arg) override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void    print_info();

protected:
	virtual int    probe() override;

private:

	work_s _work;
	ringbuffer::RingBuffer* _reports;
	bool _sensor_ok;
	int  _measure_ticks;
	bool _collect_phase;
	orb_advert_t _px4uwb_topic;

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

	struct map_projection_reference_s _pos_ref;

	float last_x = 0.0f;
	float last_y = 0.0f;
	float last_z = 0.0f;
	hrt_abstime last_time = 0;

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return		True if the device is present.
	 */
	int probe_address(uint8_t address);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void cycle();
	int  measure();
	int  collect();
	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void cycle_trampoline(void *arg);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int px4uwb_main(int argc, char *argv[]);

PX4UWB::PX4UWB(int bus, int address):
	I2C("PX4UWB", PX4UWB0_DEVICE_PATH, bus, address, PX4UWB_I2C_MAX_BUS_SPEED), /* 100-400 KHz */
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_px4uwb_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "px4f_read")),
	_comms_errors(perf_alloc(PC_COUNT, "px4f_com_err")),
	_pos_ref{}
{
	// disable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

PX4UWB::~PX4UWB()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int PX4UWB::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(vehicle_gps_position_s));

	if (_reports == nullptr) {
		return ret;
	}

	if (!map_projection_initialized(&_pos_ref)) {
		map_projection_init(&_pos_ref, 47.378301f, 8.538777f);
	}

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;


	return ret;
}

int PX4UWB::probe()
{
	uint8_t whoami = 0;

	if (transfer(0x00, 1, &whoami, 1) != OK || whoami != 0x43) {
		return -EIO;
	}

	// that worked, so start a measurement cycle
	return measure();
}

int PX4UWB::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(PX4UWB_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(PX4UWB_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t PX4UWB::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct vehicle_gps_position_s);
	struct vehicle_gps_position_s *rbuf = reinterpret_cast<struct vehicle_gps_position_s*>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int PX4UWB::measure()
{
	// int ret;

	// /*
	//  * Send the command to begin a measurement.
	//  */
	// uint8_t cmd = PX4UWB_REG;
	// ret = transfer(&cmd, 1, nullptr, 0);

	// if (OK != ret) {
	// 	perf_count(_comms_errors);
	// 	DEVICE_DEBUG("i2c::transfer returned %d", ret);
	// 	return ret;
	// }

	// ret = OK;

	// return ret;
	return OK;
}

int
PX4UWB::collect()
{
	int ret = -EIO;

	// Read out the 3 coordinates in mm
	int32_t coordinates[3] = { 0 };

	perf_begin(_sample_perf);

	uint8_t sub_addr = PX4UWB_POS_X;

	ret = transfer(&sub_addr, 1, (uint8_t*)&coordinates[0], 3*sizeof(int32_t));

	if (ret < 0) {
		DEVICE_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	// process new NED x, y, z
	float x = coordinates[0]*1e-2f; // meters
	float y = coordinates[1]*1e-2f;
	float z = coordinates[2]*1e-2f;

	float vx = 0.0f;
	float vy = 0.0f;
	float vz = 0.0f;

	/**
	* Transforms a point in the local azimuthal equidistant plane to the
	* geographic coordinate system using the projection given by the argument
	*
	* @param x north
	* @param y east
	* @param lat in degrees (47.1234567°, not 471234567°)
	* @param lon in degrees (8.1234567°, not 81234567°)
	* @return 0 if map_projection_init was called before, -1 else
	*/
	double lat = 0.0f;
	double lon = 0.0f;
	map_projection_reproject(&_pos_ref, x, y, &lat, &lon);

	struct vehicle_gps_position_s report = {};

	// report.timestamp_time_relative = 0;
	// report.time_utc_usec = hrt_absolute_time();

	report.timestamp = hrt_absolute_time();
	report.lat = (int32_t)lat * 1e7f; // Latitude in 1E-7 degrees
	report.lon = (int32_t)lon * 1e7f; // Longitude in 1E-7 degrees

	// stolen from mavors/mocap_fake_gps.cpp
	report.alt = (int32_t)(408 - z) * 1e3f; // AMSL

	report.eph = 0.9f;
	report.epv = 1.8f;

	report.s_variance_m_s = 1.0f;


	float uwb_dt = (report.timestamp - last_time) / 1e6f;
	last_time = report.timestamp;

	if (uwb_dt > 0.000001f && uwb_dt < 0.2f) {
		vx = (x - last_x) / uwb_dt;
		vy = (y - last_y) / uwb_dt;
		vz = (z - last_z) / uwb_dt;

		last_x = x;
		last_y = y;
		last_z = z;
	}

	report.vel_n_m_s = vx;
	report.vel_e_m_s = vy;
	report.vel_d_m_s = vz;
	report.vel_m_s = sqrtf(vx*vx + vy*vy);
	report.vel_ned_valid = true;

	// Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
	report.cog_rad = atan2(vx, vy);

	report.fix_type = 3;
	report.satellites_used = 10;

	if (_px4uwb_topic == nullptr) {
		_px4uwb_topic = orb_advertise(ORB_ID(vehicle_gps_position), &report);

	} else {
		/* publish it */
		orb_publish(ORB_ID(vehicle_gps_position), _px4uwb_topic, &report);
	}

	/* post a report to the ring */
	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void PX4UWB::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&PX4UWB::cycle_trampoline, this, 1);
}

void PX4UWB::stop()
{
	work_cancel(HPWORK, &_work);
}

void PX4UWB::cycle_trampoline(void *arg)
{
	PX4UWB* dev = (PX4UWB *)arg;

	dev->cycle();
}

void PX4UWB::cycle()
{
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	/* perform collection */
	if (OK != collect()) {
		DEVICE_DEBUG("collection error");
		/* restart the measurement state machine */
		start();
		return;
	}

	work_queue(HPWORK, &_work, (worker_t)&PX4UWB::cycle_trampoline, this,
		   _measure_ticks);

}

void PX4UWB::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("px4uwb report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace px4uwb
{

PX4UWB* g_dev = nullptr;
bool start_in_progress = false;

const int START_RETRY_COUNT = 5;
const int START_RETRY_TIMEOUT = 1000;

int     start();
void    stop();
void    test();
void    reset();
void    info();

/**
 * Start the driver.
 */
int start()
{
	/* entry check: */
	if (start_in_progress) {
		PX4_WARN("start already in progress");
		return 1;
	}

	start_in_progress = true;

	if (g_dev != nullptr) {
		start_in_progress = false;
		warnx("already started");
		return 1;
	}

	PX4_WARN("scanning I2C buses for device..");

	int retry_nr = 0;

	while (true) {
		const int busses_to_try[] = {
			PX4_I2C_BUS_EXPANSION,
#ifdef PX4_I2C_BUS_ONBOARD
			PX4_I2C_BUS_ONBOARD,
#endif
			-1
		};

		const int* cur_bus = busses_to_try;

		while (*cur_bus != -1) {
			/* create the driver */
			/* PX4_WARN("trying bus %d", *cur_bus); */
			g_dev = new PX4UWB(*cur_bus);

			if (g_dev == nullptr) {
				/* this is a fatal error */
				break;
			}

			/* init the driver: */
			if (OK == g_dev->init()) {
				/* success! */
				break;
			}

			/* destroy it again because it failed. */
			delete g_dev;
			g_dev = nullptr;

			/* try next! */
			cur_bus++;
		}

		/* check whether we found it: */
		if (*cur_bus != -1) {

			/* check for failure: */
			if (g_dev == nullptr) {
				break;
			}

			/* set the poll rate to default, starts automatic data collection */
			int fd = open(PX4UWB0_DEVICE_PATH, O_RDONLY);

			if (fd < 0) {
				break;
			}

			if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX) < 0) {
				break;
			}

			/* success! */
			start_in_progress = false;
			return 0;
		}

		if (retry_nr < START_RETRY_COUNT) {
			/* lets not be too verbose */
			// warnx("PX4UWB not found on I2C busses. Retrying in %d ms. Giving up in %d retries.", START_RETRY_TIMEOUT, START_RETRY_COUNT - retry_nr);
			usleep(START_RETRY_TIMEOUT * 1000);
			retry_nr++;

		} else {
			break;
		}
	}

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	start_in_progress = false;
	return 1;
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void test()
{
	struct vehicle_gps_position_s report;
	ssize_t sz;
	int ret;

	int fd = open(PX4UWB0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'px4uwb start' if the driver is not running", PX4UWB0_DEVICE_PATH);
	}


	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		warnx("immediate read failed");
	}

	PX4_WARN("single read");
	if (report.timestamp != 0) {
		PX4_WARN("position lock: %d, satellites: %d, last update: %8.4fms ago", (int)report.fix_type,
			 report.satellites_used, (double)(hrt_absolute_time() - report.timestamp) / 1000.0);
		PX4_WARN("lat: %d, lon: %d, alt: %d", report.lat, report.lon, report.alt);
		PX4_WARN("vel: %.2fm/s, %.2fm/s, %.2fm/s", (double)report.vel_n_m_s,
			 (double)report.vel_e_m_s, (double)report.vel_d_m_s);
		PX4_WARN("hdop: %.2f, vdop: %.2f", (double)report.hdop, (double)report.vdop);
		PX4_WARN("eph: %.2fm, epv: %.2fm", (double)report.eph, (double)report.epv);
	}

	/* start the sensor polling at 10Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 10)) {
		errx(1, "failed to set 10Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 10; ++i) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != OK) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		PX4_WARN("periodic read %u", i);

		if (report.timestamp != 0) {
			PX4_WARN("position lock: %d, satellites: %d, last update: %8.4fms ago", (int)report.fix_type,
				 report.satellites_used, (double)(hrt_absolute_time() - report.timestamp) / 1000.0);
			PX4_WARN("lat: %d, lon: %d, alt: %d", report.lat, report.lon, report.alt);
			PX4_WARN("vel: %.2fm/s, %.2fm/s, %.2fm/s", (double)report.vel_n_m_s,
				 (double)report.vel_e_m_s, (double)report.vel_d_m_s);
			PX4_WARN("hdop: %.2f, vdop: %.2f", (double)report.hdop, (double)report.vdop);
			PX4_WARN("eph: %.2fm, epv: %.2fm", (double)report.eph, (double)report.epv);
		}

	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void reset()
{
	int fd = open(PX4UWB0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	PX4_INFO("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace

int px4uwb_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		return px4uwb::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		px4uwb::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		px4uwb::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		px4uwb::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		px4uwb::info();
	}

	err(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
