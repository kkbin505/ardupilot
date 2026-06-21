/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * DShot protocol support for ESP32 using RMT peripheral
 */

#pragma once

#include <driver/gpio.h>
#include <atomic>
#include <cstdint>

// RMT TX channel / encoder handles are kept as opaque void* here (rather than
// the real rmt_channel_handle_t/rmt_encoder_handle_t) because driver/rmt_tx.h
// cannot be included from this header: it would collide with the legacy
// driver/rmt.h pulled in by RCInput.h wherever both headers end up in the same
// translation unit (e.g. HAL_ESP32_Class.cpp). The real types are only used
// inside RCOutput.cpp, which includes driver/rmt_tx.h itself and casts to/from
// void* at each RMT API call site.

// ============================================================================
// DShot Protocol Constants
// ============================================================================

// Frame structure: 11-bit throttle + 1-bit telemetry + 4-bit CRC = 16 bits
static constexpr uint16_t DSHOT_THROTTLE_MAX = 2047;
static constexpr uint16_t DSHOT_THROTTLE_MIN = 48;     // Minimum non-stop value
static constexpr uint16_t DSHOT_CMD_MOTOR_STOP = 0;    // Motor stop command
static constexpr uint16_t DSHOT_CRC_MASK = 0x0F;
static constexpr uint16_t DSHOT_FULL_PACKET = 0xFFFF;

// Bit timing (microseconds) for each DShot mode
struct dshot_timing_us {
    double bit_length_us;   // Total bit period
    double t1h_length_us;   // High time for '1' bit
};

static constexpr dshot_timing_us DSHOT_TIMING[] = {
    {0.0,   0.0},      // DSHOT_OFF (unused)
    {6.67,  5.00},     // DSHOT150
    {3.33,  2.50},     // DSHOT300
    {1.67,  1.25},     // DSHOT600
    {0.83,  0.67},     // DSHOT1200
};

// ============================================================================
// ESP32 RMT Hardware Constants
// ============================================================================

static constexpr uint32_t DSHOT_RMT_RESOLUTION_HZ = 8000000;  // 8 MHz, 125 ns per tick
static constexpr uint32_t RMT_TICKS_PER_US = DSHOT_RMT_RESOLUTION_HZ / 1000000;
static constexpr uint16_t RMT_TX_BUFFER_SIZE = 64;  // Symbols per TX buffer

// ============================================================================
// DShot Channel State Structure
// ============================================================================

struct dshot_chan {
    void* tx_channel = nullptr;  // opaque rmt_channel_handle_t, real type only used in RCOutput.cpp
    void* encoder = nullptr;     // opaque rmt_encoder_handle_t, shared by speed (see dshot_encoder_cache)

    uint16_t last_throttle = 0;                 // Last throttle value sent
    uint64_t last_tx_time_us = 0;               // Time of last transmission
    uint64_t min_frame_interval_us = 0;         // Minimum time between frames
};

// DShot mode enumeration (matches AP_HAL::RCOutput output_mode)
enum dshot_mode_t {
    DSHOT_OFF = 0,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200,
};

// Encoder handle cache by speed (to avoid recreating encoders for each channel)
struct dshot_encoder_cache {
    void* encoder[5] = {};       // opaque rmt_encoder_handle_t; index 0 unused, 1-4 for DSHOT150-1200
    uint32_t ref_count[5] = {};  // Reference count per speed
};
