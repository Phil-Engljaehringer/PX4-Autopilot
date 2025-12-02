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
#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <canard.h>

// STM32H7 FDCAN hardware access
#ifdef __PX4_NUTTX
#include <nuttx/config.h>
#include <arch/board/board.h>
#include <hardware/stm32_memorymap.h>
#include <hardware/stm32_fdcan.h>
#endif

using namespace time_literals;

class Dummy : public ModuleBase<Dummy>, public px4::ScheduledWorkItem
{
public:
	Dummy();
	~Dummy() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

private:
	bool init();
	void send_dummy_message();
	bool fdcan_transmit(uint32_t id, bool extended, const uint8_t *data, uint8_t len);

	static constexpr uint16_t DUMMY_MESSAGE_ID = 1234;
	static constexpr uint64_t DUMMY_MESSAGE_SIGNATURE = 0x0123456789ABCDEF;

	CanardInstance _canard{};
	uint8_t _canard_memory_pool[1024]{};

	// FDCAN hardware
#if defined(STM32_FDCAN1_BASE)
	uint32_t _fdcan_base{STM32_FDCAN1_BASE};
#else
	uint32_t _fdcan_base{0x4000a000}; // Default STM32H7 FDCAN1 base address
#endif
	uint8_t _node_id{127};
	uint8_t _transfer_id{0};
};
