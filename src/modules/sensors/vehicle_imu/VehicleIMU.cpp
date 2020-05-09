/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "VehicleIMU.hpp"

#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;

namespace sensors
{

VehicleIMU::VehicleIMU(uint8_t accel_index, uint8_t gyro_index) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::navigation_and_controllers),
	_sensor_accel_sub(this, ORB_ID(sensor_accel), accel_index),
	_sensor_gyro_sub(this, ORB_ID(sensor_gyro), gyro_index),
	_accel_corrections(this, SensorCorrections::SensorType::Accelerometer),
	_gyro_corrections(this, SensorCorrections::SensorType::Gyroscope)
{
}

VehicleIMU::~VehicleIMU()
{
	Stop();
	perf_free(_publish_interval_perf);
	perf_free(_accel_update_perf);
	perf_free(_gyro_update_perf);
}

bool VehicleIMU::Start()
{
	// force initial updates
	ParametersUpdate(true);

	//const float interval_us = 1e6f / _param_imu_integ_rate.get();
	//set_interval_us(interval_us);

	return _sensor_accel_sub.registerCallback() && _sensor_gyro_sub.registerCallback();
}

void VehicleIMU::Stop()
{
	// clear all registered callbacks
	_sensor_accel_sub.unregisterCallback();
	_sensor_gyro_sub.unregisterCallback();

	Deinit();
}

void VehicleIMU::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		_accel_corrections.ParametersUpdate();
		_gyro_corrections.ParametersUpdate();
	}
}

void VehicleIMU::Run()
{
	ParametersUpdate();
	_accel_corrections.SensorCorrectionsUpdate();
	_gyro_corrections.SensorCorrectionsUpdate();

	while (_sensor_accel_sub.updated() || _sensor_gyro_sub.updated()) {

		sensor_accel_s accel;
		sensor_gyro_s gyro;
		bool accel_updated = _sensor_accel_sub.update(&accel);
		bool gyro_updated = _sensor_gyro_sub.update(&gyro);

		bool accel_reset = false;
		Vector3f delta_velocity;
		uint32_t accel_integral_dt = 0;

		if (accel_updated) {
			perf_count_interval(_accel_update_perf, accel.timestamp_sample);
			_accel_corrections.set_device_id(accel.device_id);

			const Vector3f accel_corrected{_accel_corrections.Correct(Vector3f{accel.x, accel.y, accel.z})};

			if (_accel_integrator.put(accel.timestamp_sample, accel_corrected, delta_velocity, accel_integral_dt)) {
				accel_reset = true;
				_integrator_reset_timestamp_accel = accel.timestamp_sample;
			}

			// collect sample interval average for filters
			if ((_timestamp_sample_last_accel > 0) && (accel.timestamp_sample > _timestamp_sample_last_accel)) {
				_interval_sum_accel += (accel.timestamp_sample - _timestamp_sample_last_accel);
				_interval_count_accel++;

			} else {
				_interval_sum_accel = 0.f;
				_interval_count_accel = 0.f;
			}

			_timestamp_sample_last_accel = accel.timestamp_sample;

			if (_interval_count_accel > 3000) {
				// calculate sensor update rate
				const float sample_interval_avg = _interval_sum_accel / _interval_count_accel;

				if (PX4_ISFINITE(sample_interval_avg) && (sample_interval_avg > 0.0f)) {
					// check if sample rate error is greater than 1%
					if ((fabsf(_update_interval_accel - sample_interval_avg) / _update_interval_accel) > 0.01f) {

						_update_interval_accel = sample_interval_avg;

						const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();
						float ratio = configured_interval_us / sample_interval_avg;

						_accel_interval_us = roundf(roundf(ratio) * sample_interval_avg);
						_accel_integrator.set_autoreset_interval(_accel_interval_us);
						_sensor_accel_sub.set_required_updates(roundf(ratio));
						PX4_DEBUG("accel ratio: %.3f", (double)roundf(ratio));
					}
				}

				// reset sample interval accumulator
				_timestamp_sample_last_accel = 0;
			}
		}

		bool gyro_reset = false;
		Vector3f delta_angle;
		uint32_t gyro_integral_dt = 0;

		if (gyro_updated) {
			perf_count_interval(_gyro_update_perf, gyro.timestamp_sample);
			_gyro_corrections.set_device_id(gyro.device_id);

			const Vector3f gyro_corrected{_gyro_corrections.Correct(Vector3f{gyro.x, gyro.y, gyro.z})};

			if (_gyro_integrator.put(gyro.timestamp_sample, gyro_corrected, delta_angle, gyro_integral_dt)) {
				gyro_reset = true;
				_integrator_reset_timestamp_gyro = gyro.timestamp_sample;
			}

			// collect sample interval average for filters
			if ((_timestamp_sample_last_gyro > 0) && (gyro.timestamp_sample > _timestamp_sample_last_gyro)) {
				_interval_sum_gyro += (gyro.timestamp_sample - _timestamp_sample_last_gyro);
				_interval_count_gyro++;

			} else {
				_interval_sum_gyro = 0.f;
				_interval_count_gyro = 0.f;
			}

			_timestamp_sample_last_gyro = gyro.timestamp_sample;

			if (_interval_count_gyro > 3000) {
				// calculate sensor update rate
				const float sample_interval_avg = _interval_sum_gyro / _interval_count_gyro;

				if (PX4_ISFINITE(sample_interval_avg) && (sample_interval_avg > 0.0f)) {
					// check if sample rate error is greater than 1%
					if ((fabsf(_update_interval_gyro - sample_interval_avg) / _update_interval_gyro) > 0.01f) {
						_update_interval_gyro = sample_interval_avg;

						const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();
						float ratio = configured_interval_us / sample_interval_avg;

						_gyro_interval_us = roundf(roundf(ratio) * sample_interval_avg);
						_gyro_integrator.set_autoreset_interval(_gyro_interval_us);
						_sensor_gyro_sub.set_required_updates(roundf(ratio));
						PX4_DEBUG("gyro ratio: %.3f", (double)roundf(ratio));
					}
				}

				// reset sample interval accumulator
				_timestamp_sample_last_gyro = 0;
			}
		}

		bool publish = false;

		if (accel_reset && gyro_reset) {
			publish = true;

		} else if (accel_reset != gyro_reset) {
			// force integrator reset to keep in sync
			if (!accel_reset) {
				delta_velocity = _accel_integrator.reset(accel_integral_dt);
				_integrator_reset_timestamp_accel = _timestamp_sample_last_accel;
			}

			if (!gyro_reset) {
				delta_angle = _gyro_integrator.reset(gyro_integral_dt);
				_integrator_reset_timestamp_gyro = _timestamp_sample_last_gyro;
			}

			if (accel_integral_dt > 0 && gyro_integral_dt > 0) {
				publish = true;

			} else {
				PX4_ERR("accel_integral_dt: %d gyro_integral_dt: %d", accel_integral_dt, gyro_integral_dt);
			}
		}

		if (publish) {
			// publich vehicle_imu
			perf_count(_publish_interval_perf);

			vehicle_imu_s imu{};
			imu.timestamp_sample = (_integrator_reset_timestamp_accel + _integrator_reset_timestamp_gyro) / 2;
			imu.accel_device_id = _accel_corrections.get_device_id();
			imu.gyro_device_id = _gyro_corrections.get_device_id();

			delta_angle.copyTo(imu.delta_angle);
			delta_velocity.copyTo(imu.delta_velocity);

			imu.dt = (accel_integral_dt + gyro_integral_dt) / 2;
			imu.dt_accel = accel_integral_dt;
			imu.dt_gyro = gyro_integral_dt;
			//imu.clip_count = accel.clip_count;
			imu.timestamp = hrt_absolute_time();

			_vehicle_imu_pub.publish(imu);
			return;
		}
	}
}

void VehicleIMU::PrintStatus()
{
	PX4_INFO("selected IMU: accel: %d gyro: %d ", _accel_corrections.get_device_id(), _gyro_corrections.get_device_id());
	perf_print_counter(_publish_interval_perf);
	perf_print_counter(_accel_update_perf);
	perf_print_counter(_gyro_update_perf);
	_accel_corrections.PrintStatus();
	_gyro_corrections.PrintStatus();
}

} // namespace sensors
