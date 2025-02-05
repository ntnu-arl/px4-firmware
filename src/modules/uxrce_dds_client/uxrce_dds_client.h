/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <src/modules/uxrce_dds_client/dds_topics.h>

#include <lib/timesync/Timesync.hpp>

#include "srv_base.h"

#define MAX_NUM_REPLIERS 5

class UxrceddsClient : public ModuleBase<UxrceddsClient>, public ModuleParams
{
public:
	enum class Transport {
		Serial,
		Udp
	};

	UxrceddsClient(Transport transport, const char *device, int baudrate, const char *host, const char *port,
		       bool localhost_only, bool custom_participant, const char *client_namespace, bool synchornize_timestamps);

	~UxrceddsClient();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static UxrceddsClient *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/**
	 * @brief Method to add a new replyer to the replier array.
	 * @param replier pointer to the new replier.
	 * @return Returns false iff successful, otherwise false.
	 */
	bool add_replier(SrvBase *replier);

	/**
	 * @brief Method to process new incoming requests and dispatch them to the appropriate server.
	 * @param object_id replier object id
	 * @param sample_id pointer to specific request.
	 * @param time_offset_us time offset between agent and client.
	 * @param ub pointer to the received request data
	 */
	void process_requests(uxrObjectId object_id, SampleIdentity *sample_id, ucdrBuffer *ub, const int64_t time_offset_us);

	/**
	 * @brief Method to process the available replies.
	 * @return Returns false iff successful, otherwise false.
	 */
	void process_replies();

	/**
	 * @brief Method to delete all repliers.
	 * @return Returns false iff successful, otherwise false.
	 */
	void delete_repliers();

private:
	int setBaudrate(int fd, unsigned baud);

	const bool _localhost_only;
	const bool _custom_participant;
	const char *_client_namespace;
	const bool _synchronize_timestamps;

	// max port characters (5+'\0')
	static const uint8_t PORT_MAX_LENGTH = 6;
	// max agent ip characters (15+'\0')
	static const uint8_t AGENT_IP_MAX_LENGTH = 16;

#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	char _port[PORT_MAX_LENGTH];
	char _agent_ip[AGENT_IP_MAX_LENGTH];
#endif

	SendTopicsSubs *_subs{nullptr};
	RcvTopicsPubs *_pubs{nullptr};

	SrvBase *repliers_[MAX_NUM_REPLIERS];
	uint8_t num_of_repliers{0};

	uxrSerialTransport *_transport_serial{nullptr};
	uxrUDPTransport *_transport_udp{nullptr};
	uxrCommunication *_comm{nullptr};
	int _fd{-1};

	int _last_payload_tx_rate{}; ///< in B/s
	int _last_payload_rx_rate{}; ///< in B/s
	bool _connected{false};

	Timesync _timesync{timesync_status_s::SOURCE_PROTOCOL_DDS};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UXRCE_DDS_DOM_ID>) _param_xrce_dds_dom_id,
		(ParamInt<px4::params::UXRCE_DDS_KEY>) _param_xrce_key
	)
};
