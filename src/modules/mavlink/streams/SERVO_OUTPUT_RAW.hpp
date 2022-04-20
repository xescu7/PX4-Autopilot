/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef SERVO_OUTPUT_RAW_HPP
#define SERVO_OUTPUT_RAW_HPP

#include <uORB/topics/actuator_outputs.h>

template <int N>
class MavlinkStreamServoOutputRaw : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamServoOutputRaw<N>(mavlink); }

	static constexpr const char *get_name_static()
	{
		switch (N) {
		case 0:
			return "SERVO_OUTPUT_RAW_0";

		case 1:
			return "SERVO_OUTPUT_RAW_1";
		}
	}

	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SERVO_OUTPUT_RAW; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _act_sub.advertised() ? MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamServoOutputRaw(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	// ALDORATECH edit: commented and rewrited this part
	// uORB::Subscription _act_sub{ORB_ID(actuator_outputs), N};

	// bool send() override
	// {
	// 	actuator_outputs_s act;

	// 	if (_act_sub.update(&act)) {
	// 		mavlink_servo_output_raw_t msg{};

	// 		static_assert(sizeof(act.output) / sizeof(act.output[0]) >= 16, "mavlink message requires at least 16 outputs");

	// 		msg.time_usec = act.timestamp;
	// 		msg.port = N;
	// 		msg.servo1_raw = act.output[0];
	// 		msg.servo2_raw = act.output[1];
	// 		msg.servo3_raw = act.output[2];
	// 		msg.servo4_raw = act.output[3];
	// 		msg.servo5_raw = act.output[4];
	// 		msg.servo6_raw = act.output[5];
	// 		msg.servo7_raw = act.output[6];
	// 		msg.servo8_raw = act.output[7];
	// 		msg.servo9_raw = act.output[8];
	// 		msg.servo10_raw = act.output[9];
	// 		msg.servo11_raw = act.output[10];
	// 		msg.servo12_raw = act.output[11];
	// 		msg.servo13_raw = act.output[12];
	// 		msg.servo14_raw = act.output[13];
	// 		msg.servo15_raw = act.output[14];
	// 		msg.servo16_raw = act.output[15];

	// 		mavlink_msg_servo_output_raw_send_struct(_mavlink->get_channel(), &msg);

	// 		return true;
	// 	}

	// 	return false;
	// }

	// This way all Mains (8) and Auxs (6) are sent via the same message
	uORB::Subscription _act_sub_0{ORB_ID(actuator_outputs), 0};
	uORB::Subscription _act_sub_1{ORB_ID(actuator_outputs), 1};

	bool send() override
	{
		actuator_outputs_s actMain;
		actuator_outputs_s actAux;

		if (_act_sub_0.update(&actMain) && _act_sub_1.update(&actAux)) {
			mavlink_servo_output_raw_t msg{};

			static_assert(sizeof(actMain.output) / sizeof(actMain.output[0]) >= 16, "mavlink message requires at least 16 outputs");
			static_assert(sizeof(actAux.output) / sizeof(actAux.output[0]) >= 16, "mavlink message requires at least 16 outputs");

			msg.time_usec = actMain.timestamp;
			msg.port = 0;
			msg.servo1_raw = actMain.output[0];
			msg.servo2_raw = actMain.output[1];
			msg.servo3_raw = actMain.output[2];
			msg.servo4_raw = actMain.output[3];
			msg.servo5_raw = actMain.output[4];
			msg.servo6_raw = actMain.output[5];
			msg.servo7_raw = actMain.output[6];
			msg.servo8_raw = actMain.output[7];
			msg.servo9_raw = actAux.output[0];
			msg.servo10_raw = actAux.output[1];
			msg.servo11_raw = actAux.output[2];
			msg.servo12_raw = actAux.output[3];
			msg.servo13_raw = actAux.output[4];
			msg.servo14_raw = actAux.output[5];
			msg.servo15_raw = actMain.output[14];
			msg.servo16_raw = actMain.output[15];

			mavlink_msg_servo_output_raw_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // SERVO_OUTPUT_RAW_HPP
