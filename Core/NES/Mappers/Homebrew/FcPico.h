// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include "NES/BaseMapper.h"
#include "NES/BaseNesPpu.h"
#include "fcpico_cart.h"

// NES CPU/PPU integration only: the cartridge executes the native host bus model,
// not ARM instructions. CS1 selection is an explicit, uncalibrated board hypothesis.
class FcPico : public BaseMapper
{
	uint16_t _cs1Mask = 0xF000;
	uint32_t _renderReads = 0;
	uint32_t _cpuReads = 0;
	std::ofstream _trace;
	std::ofstream _fetchTrace;
	uint32_t _fetchFrame = 0;

	uint16_t GetPrgPageSize() override { return 0x8000; }
	uint16_t GetChrPageSize() override { return 0x2000; }
	uint32_t GetChrRamSize() override { return 0x2000; }
	bool EnableCustomVramRead() override { return true; }

	void InitMapper() override
	{
		SelectPrgPage(0, 0);
		SelectChrPage(0, 0);
		if(const char* mask = std::getenv("FCPICO_CS1_MASK")) {
			char* end;
			unsigned long value = std::strtoul(mask, &end, 0);
			if(*end || (value != 0xF000 && value != 0xE000)) {
				throw std::runtime_error("FCPICO_CS1_MASK must be 0xf000 or 0xe000");
			}
			_cs1Mask = static_cast<uint16_t>(value);
		}
		if(!fcpico_cart_init(_prgRom, _prgSize)) {
			throw std::runtime_error("FC PICO requires a 32 KiB tutorial PRG");
		}
		if(const char* path = std::getenv("FCPICO_TRACE")) {
			_trace.open(path, std::ios::trunc);
			if(!_trace) {
				throw std::runtime_error("Cannot open FCPICO_TRACE");
			}
			_trace << "event,ppu_frame,scanline,cycle,value,heartbeats,last_count,dma_stops,resyncs,render_reads,cpu_reads,init_actions,open_bus_reads,pattern_frames\n";
		}
		if(const char* path = std::getenv("FCPICO_FETCH_TRACE")) {
			const char* frame = std::getenv("FCPICO_FETCH_FRAME");
			if(!frame) { throw std::runtime_error("FCPICO_FETCH_FRAME is required with FCPICO_FETCH_TRACE"); }
			char* end;
			unsigned long value = std::strtoul(frame, &end, 10);
			if(*end || value > UINT32_MAX) { throw std::runtime_error("Invalid FCPICO_FETCH_FRAME"); }
			_fetchFrame = static_cast<uint32_t>(value);
			_fetchTrace.open(path, std::ios::trunc);
			if(!_fetchTrace) { throw std::runtime_error("Cannot open FCPICO_FETCH_TRACE"); }
			_fetchTrace << "ppu_frame,scanline,cycle,address,value\n";
		}
	}

	void Trace(const char* event, unsigned value)
	{
		if(!_trace.is_open()) { return; }
		NesPpuState ppu;
		_console->GetPpu()->GetState(ppu);
		const fcbus_stats_t* stats = fcpico_cart_stats();
		const fcpico_cart_metrics_t* metrics = fcpico_cart_metrics();
		_trace << event << ',' << _console->GetPpu()->GetFrameCount() << ','
			<< ppu.Scanline << ',' << ppu.Cycle << ',' << value << ',' << stats->frames
			<< ',' << stats->last_count << ',' << stats->dma_stops << ',' << stats->resyncs
			<< ',' << _renderReads << ',' << _cpuReads << ',' << metrics->init_actions
			<< ',' << metrics->open_bus_reads << ',' << metrics->pattern_frames << '\n';
		_trace.flush();
	}

public:
	~FcPico() override { fcpico_cart_shutdown(); }

	uint8_t MapperReadVram(uint16_t addr, MemoryOperationType type) override
	{
		// BaseMapper::DebugReadVram bypasses this hook and uses InternalReadVram.
		// Count only actual renderer fetches and CPU $2007 reads, once per access.
		if((addr & _cs1Mask) == 0 &&
			(type == MemoryOperationType::PpuRenderingRead || type == MemoryOperationType::Read)) {
			if(type == MemoryOperationType::PpuRenderingRead) { ++_renderReads; }
			else { ++_cpuReads; }
			uint8_t value = fcpico_cart_ppu_read();
			if(_fetchTrace.is_open() && type == MemoryOperationType::PpuRenderingRead &&
				_console->GetPpu()->GetFrameCount() == _fetchFrame) {
				NesPpuState ppu;
				_console->GetPpu()->GetState(ppu);
				_fetchTrace << _fetchFrame << ',' << ppu.Scanline << ',' << ppu.Cycle
					<< ',' << addr << ',' << static_cast<unsigned>(value) << '\n';
			}
			return value;
		}
		return InternalReadVram(addr);
	}

	void MapperWriteVram(uint16_t addr, uint8_t value) override
	{
		if((addr & _cs1Mask) == 0) {
			uint32_t frames = fcpico_cart_stats()->frames;
			fcpico_cart_ppu_write(value);
			Trace("write", value);
			if(fcpico_cart_stats()->frames != frames) {
				_renderReads = _cpuReads = 0;
			}
		} else {
			InternalWriteVram(addr, value);
		}
	}

	void EndFrame() override
	{
		uint64_t clock = _console->GetMasterClock();
		fcpico_cart_tick_ms(static_cast<uint32_t>(clock * 1000 / _console->GetMasterClockRate()));
		Trace("frame", 0);
	}
};
