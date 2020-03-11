/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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
 * @file nmea.cpp
 *
 * @author Volga Bogoslovsky <gonzalez1139@gmail.com>
 */

#include "nmea.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>
#include <cmath>
#include <cstdlib>

#include <uORB/uORB.h>


GPSDriverNMEA::GPSDriverNMEA(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position)
{
	decodeInit();
}

int
GPSDriverNMEA::configure(unsigned &baudrate, OutputMode output_mode)
{

	if (output_mode != OutputMode::GPS) {
		GPS_WARN("NMEA: Unsupported Output Mode %i", (int)output_mode);
		return -1;
	}

	/* set baudrate first */
	if (GPSHelper::setBaudrate(NMEA_BAUDRATE) != 0) {
		return -1;
	}

	baudrate = NMEA_BAUDRATE;

/*errout:
	GPS_WARN("nmea: config write failed");
	return -1;*/
	return 0;
}

int
GPSDriverNMEA::receive(unsigned timeout)
{
	char buf[GPS_READ_BUFFER_SIZE];

	//int time_start = hrt_absolute_time();

	int j = 0;
	char message[74];
	//buffer_start = buf_search_byte;

	int ret = read((uint8_t*)buf, sizeof(buf), timeout);
	//PX4_INFO("Read buffer size: %d", ret);

	if (ret > 0) {
		PX4_INFO("Read message size %d", ret);
		while (j < ret) {
			//PX4_INFO("Buffer serach index: %d", buf_search_byte);
			if (buf[j] == '$') {
				//PX4_INFO("Find start simbol $");
				len_counter = 0;
				msg_start = j;
			} else if (buf[j] == '*') {
				//PX4_INFO("Find end simbol *. Message size: %d", (int)len_counter);
				if (len_counter != -1) {
					msg_end = len_counter + j;
				}
			}

			if (msg_start > -1) {
				message[len_counter] = buf[j];
			}

			if (msg_start != -1 && msg_end != -1) {

				//PX4_INFO("Try decode with, start, stop: %d, %d", msg_start, msg_end);
				if (decode(message)) {
					len_counter = -1;
					msg_start = -1;
					msg_end = -1;
					PX4_INFO("Succesful decode");
					return 1;
				} else {
					len_counter = -1;
					msg_start = -1;
					msg_end = -1;
					return 0;
				}
			}

			len_counter++;
			j++;
		}
		j = 0;
	}

	return 1;
}


int
GPSDriverNMEA::decode(char *message) {
	parse_byte = 0;

	if (message[parse_byte] == '$') {
		coma_point = 0xff;
		char_point = 0;
	}

	//
	while (parse_byte < 74) {
		if (parse_byte < 6 && parse_byte > 1){
			if (message[parse_byte] != msg_type[parse_byte - 2]) {
				PX4_INFO("Out from decode");
				return 0;
			}
		}

		//PX4_INFO("Working char: %c", (char)message[parse_byte + msg_start]);
		if (message[parse_byte] == ',') {
			coma_point++;
			char_point = 0;
		} else if (message[parse_byte] == '*') {
			break;
		} else {
			GGA[coma_point][char_point++] = message[parse_byte];
		}

		parse_byte++;
	}

	double lat = std::strtod(GGA[1], NULL);
	double lon = std::strtod(GGA[3], NULL);
	double alt = std::strtod(GGA[8], NULL);

	/*Handling message*/
	_gps_position->timestamp = hrt_absolute_time();
	_gps_position->lat = lat * 1e7;
	_gps_position->lon = lon * 1e7;
	_gps_position->alt = alt;

	_gps_position->fix_type = 3;
	_gps_position->hdop = 1.0;
	_gps_position->vdop = 1.0;
	_gps_position->satellites_used = 8;

	return 1;
}

void
GPSDriverNMEA::decodeInit()
{

}
