/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file nmea.h
 *
 * @author Volga Bogoslovsky <gonzalez1139@gmail.com>
 */


#include "gps_helper.h"

#define NMEA_BAUDRATE 57600

typedef struct {
    int32_t latitude;
    int32_t longitude;
    uint8_t satellites;
    uint16_t hdop;
    uint16_t utc_time;
} nmea_message;

class GPSDriverNMEA : public GPSHelper
{
public:
    GPSDriverNMEA(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position);

    int receive(unsigned timeout);
    int configure(unsigned &baudrate, OutputMode output_mode);

private:
    int len_counter{-1};
    int parse_byte{};
    int buffer_start{0};
    int msg_start{-1};
    int msg_end{-1};

	unsigned char coma_point{};
	unsigned char char_point{};

    char msg_type[4] = {'P', 'G', 'G', 'A'};

    char latitude[16]{};
	char longitude[12]{};
    char altitude[12]{};
	char UNUSED[32]{};
	char *const GGA[15] = {UNUSED, latitude, UNUSED, longitude, UNUSED, UNUSED, UNUSED, UNUSED, altitude, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED};

    int decode(char *buf);
    struct vehicle_gps_position_s *_gps_position {nullptr};

    void decodeInit();
};
