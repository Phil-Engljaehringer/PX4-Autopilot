/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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
#include "dummy.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#define MODULE_NAME "dummy"

Dummy::Dummy() :
	ModuleBase(),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

Dummy::~Dummy()
{
}

bool Dummy::init()
{
	// Initialize libcanard
	canardInit(&_canard, _canard_memory_pool, sizeof(_canard_memory_pool),
	           nullptr, nullptr, nullptr);

	canardSetLocalNodeID(&_canard, _node_id);

	PX4_INFO("Initialized with node ID %d (using direct FDCAN hardware access)", _node_id);
	PX4_INFO("FDCAN1 base: 0x%08lX", (unsigned long)_fdcan_base);

	ScheduleOnInterval(1_s); // Run every second
	return true;
}

bool Dummy::fdcan_transmit(uint32_t id, bool extended, const uint8_t *data, uint8_t len)
{
#ifdef __PX4_NUTTX
	// Direct access to FDCAN hardware registers
	volatile uint32_t *fdcan = (volatile uint32_t *)_fdcan_base;

	// Check if FDCAN is in normal mode (not in INIT mode)
	uint32_t cccr = fdcan[STM32_FDCAN_CCCR_OFFSET / 4];
	if (cccr & 0x00000001) {  // INIT bit
		PX4_ERR("FDCAN in INIT mode (CCCR=0x%08lX) - not operational", (unsigned long)cccr);
		PX4_ERR("Is UAVCAN started? Try: uavcan start");
		return false;
	}

	// Check if TX FIFO has space (TFQS bit in TXFQS register)
	uint32_t txfqs = fdcan[STM32_FDCAN_TXFQS_OFFSET / 4];

	// Debug: Print TXFQS register value
	static int debug_count = 0;
	if (debug_count++ < 5) {
		PX4_INFO("TXFQS register: 0x%08lX, TFQF bit: %d, put_index: %lu",
		         (unsigned long)txfqs,
		         (txfqs & FDCAN_TXFQS_TFQF) ? 1 : 0,
		         (unsigned long)((txfqs & FDCAN_TXFQS_TFQPI_MASK) >> FDCAN_TXFQS_TFQPI_SHIFT));
	}

	if ((txfqs & FDCAN_TXFQS_TFQF) != 0) {
		PX4_WARN("FDCAN TX FIFO full (TXFQS=0x%08lX)", (unsigned long)txfqs);
		return false;
	}

	// Get put index
	uint32_t put_index = (txfqs & FDCAN_TXFQS_TFQPI_MASK) >> FDCAN_TXFQS_TFQPI_SHIFT;

	// Calculate TX buffer address (in Message RAM)
	// For STM32H7, TX buffer starts at offset 0x200 in FDCAN SRAM
	uint32_t *tx_buffer = (uint32_t *)(STM32_CANRAM_BASE + 0x200 + (put_index * 18 * 4));

	// T0: ID and flags
	if (extended) {
		tx_buffer[0] = (id & 0x1FFFFFFF) | (1 << 30); // XTD bit
	} else {
		tx_buffer[0] = (id & 0x7FF) << 18;
	}

	// T1: DLC, BRS, FDF, etc.
	uint32_t dlc_code = len <= 8 ? len : 8;
	tx_buffer[1] = (dlc_code << 16); // DLC in bits 19:16

	// Copy data (up to 8 bytes for classic CAN)
	for (int i = 0; i < (len + 3) / 4; i++) {
		uint32_t word = 0;
		for (int j = 0; j < 4 && (i * 4 + j) < len; j++) {
			word |= data[i * 4 + j] << (j * 8);
		}
		tx_buffer[2 + i] = word;
	}

	// Request transmission by writing to TXBAR
	fdcan[STM32_FDCAN_TXBAR_OFFSET / 4] = (1 << put_index);

	return true;
#else
	PX4_ERR("FDCAN hardware access not available");
	return false;
#endif
}

void Dummy::send_dummy_message()
{
	// dummy payload
	uint8_t payload[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

	// Broadcast the message using libcanard
	int result = canardBroadcast(&_canard,
	                             DUMMY_MESSAGE_SIGNATURE,
	                             DUMMY_MESSAGE_ID,
	                             &_transfer_id,
	                             CANARD_TRANSFER_PRIORITY_MEDIUM,
	                             payload,
	                             sizeof(payload));

	if (result > 0) {
		// Transmit CAN frames using direct FDCAN hardware
		const CanardCANFrame *frame;

		while ((frame = canardPeekTxQueue(&_canard)) != nullptr) {
			bool extended = (frame->id & CANARD_CAN_FRAME_EFF) != 0;
			uint32_t id = frame->id & CANARD_CAN_EXT_ID_MASK;

			if (fdcan_transmit(id, extended, frame->data, frame->data_len)) {
				canardPopTxQueue(&_canard);
				PX4_INFO("Sent CAN frame (ID: 0x%lX, len: %d, transfer: %d)", (unsigned long)id, frame->data_len, _transfer_id);
			} else {
				PX4_WARN("FDCAN transmit failed");
				break;
			}
		}
	} else {
		PX4_ERR("canardBroadcast failed: %d", result);
	}
}

void Dummy::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	send_dummy_message();
}

int Dummy::task_spawn(int argc, char *argv[])
{
	Dummy *instance = new Dummy();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Dummy::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Dummy::print_usage(const char *reason)
{
	return 0;
}

extern "C" __EXPORT int dummy_main(int argc, char *argv[])
{
	return Dummy::main(argc, argv);
}
