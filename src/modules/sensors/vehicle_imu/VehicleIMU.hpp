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

#pragma once

#include <sensor_corrections/SensorCorrections.hpp>

#include <lib/drivers/device/integrator.h>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_imu.h>

namespace sensors
{

class VehicleIMU : public ModuleParams, public px4::WorkItem
{
public:
	VehicleIMU() = delete;
	VehicleIMU(uint8_t accel_index = 0, uint8_t gyro_index = 0);

	~VehicleIMU() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void ParametersUpdate(bool force = false);
	void Run() override;

	uORB::PublicationMulti<vehicle_imu_s> _vehicle_imu_pub{ORB_ID(vehicle_imu)};
	uORB::Subscription _params_sub{ORB_ID(parameter_update)};
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub;
	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub;

	SensorCorrections _accel_corrections;
	SensorCorrections _gyro_corrections;

	Integrator _accel_integrator{5000, false}; // 200 Hz default
	Integrator _gyro_integrator{5000, true};  // 200 Hz default, coning compensation enabled

	hrt_abstime _integrator_reset_timestamp_accel{0};
	hrt_abstime _integrator_reset_timestamp_gyro{0};

	hrt_abstime _timestamp_sample_last_accel{0};
	float _interval_sum_accel{0.f};
	float _interval_count_accel{0.f};
	float _update_interval_accel{0.f};

	hrt_abstime _timestamp_sample_last_gyro{0};
	float _interval_sum_gyro{0.f};
	float _interval_count_gyro{0.f};
	float _update_interval_gyro{0.f};


	uint32_t _accel_interval_us{800};
	uint32_t _gyro_interval_us{800};

	perf_counter_t _publish_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval")};
	perf_counter_t _accel_update_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": accel update interval")};
	perf_counter_t _gyro_update_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": gyro update interval")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IMU_INTEG_RATE>) _param_imu_integ_rate
	)
};

} // namespace sensors
