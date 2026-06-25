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
struct Dshot_Timing_Us {
    double bit_length_us;   // Total bit period
    double t1h_length_us;   // High time for '1' bit
};

enum class Dshot_Mode : uint8_t {
    OFF = 0,
    MODE_150,
    MODE_300,
    MODE_600,
    MODE_1200,
};

static constexpr Dshot_Timing_Us DSHOT_TIMING[] = {
    {0.0,   0.0},      // OFF (unused)
    {6.67,  5.00},     // MODE_150
    {3.33,  2.50},     // MODE_300
    {1.67,  1.25},     // MODE_600
    {0.83,  0.67},     // MODE_1200
};

// ============================================================================
// ESP32 RMT Hardware Constants
// ============================================================================

static constexpr uint32_t DSHOT_RMT_RESOLUTION_HZ = 8000000;  // 8 MHz, 125 ns per tick
static constexpr uint32_t RMT_TICKS_PER_US = DSHOT_RMT_RESOLUTION_HZ / 1000000;
// ESP32-S3 has 48 words of RMT memory per channel; using exactly 48 keeps
// each channel in one memory block so all 4 TX channels remain available.
// DShot only needs 16 symbols per frame so 48 is more than sufficient.
static constexpr uint16_t RMT_TX_BUFFER_SIZE = 48;

// ============================================================================
// DShot Channel State Structure
// ============================================================================

struct Dshot_Chan {
    void* tx_channel = nullptr;  // opaque rmt_channel_handle_t, real type only used in RCOutput.cpp
    void* encoder = nullptr;     // opaque rmt_encoder_handle_t, shared by speed (see Dshot_Encoder_Cache)

    bool is_dshot = false;                      // True once set_output_mode selects DShot for this channel,
                                                // even if tx_channel is null due to a failed RMT init.
                                                // Used to prevent silent PWM fallback on RMT failure.
    uint16_t last_throttle = 0;                 // Last throttle value sent
    uint64_t last_tx_time_us = 0;               // Time of last transmission
    uint64_t min_frame_interval_us = 0;         // Minimum time between frames
};

// Encoder handle cache by speed (to avoid recreating encoders for each channel)
struct Dshot_Encoder_Cache {
    void* encoder[5] = {};       // opaque rmt_encoder_handle_t; index 0 unused, 1-4 for MODE_150-MODE_1200
    uint32_t ref_count[5] = {};  // Reference count per speed
};
