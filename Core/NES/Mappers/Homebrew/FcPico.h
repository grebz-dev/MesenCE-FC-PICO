// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include "NES/BaseMapper.h"
#include "NES/BaseNesPpu.h"
#include "fcpico_cart.h"

// NES CPU/PPU integration only: the cartridge executes the native host bus model,
// not ARM instructions. Rendering selection matches the NES-001 hardware trace.
class FcPico : public BaseMapper
{
	uint16_t _cs1Mask = 0xF800;
	uint32_t _renderReads = 0;
	uint32_t _cpuReads = 0;
	std::ofstream _trace;
	std::ofstream _fetchTrace;
	uint32_t _fetchFrame = 0;

	// PRG flash (JEDEC, 4 KiB sectors, commands decoded on A10-A0 as in the
	// cartridge's flashdevice.nut MASK_A10). Operations complete instantly
	// except for a short status phase, which the fix bank's DQ3/DQ7/DQ6
	// polling loops require to observe.
	enum class FlashState { Read, Unlock1, Unlock2, Command, EraseUnlock1, EraseUnlock2 };
	FlashState _flash = FlashState::Read;
	bool _eraseSetup = false;
	uint32_t _busyReads = 0;
	uint8_t _toggle = 0;
	uint32_t _flashErases = 0;
	uint32_t _flashPrograms = 0;
	uint32_t _flashProgramErrors = 0;

	uint16_t GetPrgPageSize() override { return 0x8000; }
	uint16_t GetChrPageSize() override { return 0x2000; }
	uint32_t GetChrRamSize() override { return 0x2000; }
	bool EnableCustomVramRead() override { return true; }
	bool AllowRegisterRead() override { return true; }

	uint8_t ReadRegister(uint16_t addr) override
	{
		if(_busyReads > 0) {
			// Embedded-algorithm status: DQ7 = complement of erased data,
			// DQ6 toggles, DQ3 = 0 (sector-erase timeout window).
			--_busyReads;
			_toggle ^= 0x40;
			return _toggle;
		}
		return _prgRom[addr & 0x7FFF];
	}

	void WriteRegister(uint16_t addr, uint8_t value) override
	{
		uint16_t cmd = addr & 0x7FF;
		switch(_flash) {
			case FlashState::Read:
				_flash = (cmd == 0x555 && value == 0xAA) ? FlashState::Unlock1 : FlashState::Read;
				return;
			case FlashState::Unlock1:
				_flash = (cmd == 0x2AA && value == 0x55) ? FlashState::Unlock2 : FlashState::Read;
				return;
			case FlashState::Unlock2:
				if(cmd == 0x555 && value == 0x80) {
					_flash = FlashState::EraseUnlock1;
					_eraseSetup = true;
				} else if(cmd == 0x555 && value == 0xA0) {
					_flash = FlashState::Command;
				} else {
					_flash = FlashState::Read;
				}
				return;
			case FlashState::EraseUnlock1:
				_flash = (cmd == 0x555 && value == 0xAA) ? FlashState::EraseUnlock2 : FlashState::Read;
				return;
			case FlashState::EraseUnlock2:
				if(cmd == 0x2AA && value == 0x55) {
					_flash = FlashState::Command;
				} else {
					_flash = FlashState::Read;
					_eraseSetup = false;
				}
				return;
			case FlashState::Command:
				if(_eraseSetup) {
					if(value == 0x30) {
						std::memset(_prgRom + (addr & 0x7000), 0xFF, 0x1000);
						_busyReads = 64;
						++_flashErases;
						Trace("flash_erase", addr & 0xF000);
					}
				} else {
					// Programming can only clear bits.
					uint8_t& cell = _prgRom[addr & 0x7FFF];
					if((cell & value) != value) { ++_flashProgramErrors; }
					cell &= value;
					_busyReads = 2;
					++_flashPrograms;
				}
				_eraseSetup = false;
				_flash = FlashState::Read;
				return;
		}
	}

	void InitMapper() override
	{
		SelectPrgPage(0, 0);
		SelectChrPage(0, 0);
		if(const char* mask = std::getenv("FCPICO_CS1_MASK")) {
			char* end;
			unsigned long value = std::strtoul(mask, &end, 0);
			if(*end || (value != 0xF800 && value != 0xF000 && value != 0xE000)) {
				throw std::runtime_error("FCPICO_CS1_MASK must be 0xf800, 0xf000 or 0xe000");
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

	bool IsSelectedAddress(uint16_t addr) const
	{
		// The tutorial selects the stream with tile $80 ($0800), while its
		// font tiles at $0000 and nametable 1's tile $00 stay in CHR RAM.
		// Decode the address; do not force a read count by dropping cycles.
		// The wider masks are retained only for diagnostic comparisons.
		uint16_t base = _cs1Mask == 0xF800 ? 0x0800 : 0;
		return (addr & _cs1Mask) == base;
	}

public:
	~FcPico() override { fcpico_cart_shutdown(); }

	uint8_t MapperReadVram(uint16_t addr, MemoryOperationType type) override
	{
		// BaseMapper::DebugReadVram bypasses this hook and uses InternalReadVram.
		// Count only actual renderer fetches and CPU $2007 reads, once per access.
		bool selected = IsSelectedAddress(addr);
		if(selected &&
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
		if(IsSelectedAddress(addr)) {
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
