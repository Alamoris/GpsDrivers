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

	/*int msg_start = -1;
	int msg_end = -1;
	int len_counter = -1;*/
	int j = 0;
	//buffer_start = buf_search_byte;

	int ret = read((uint8_t*)buf, sizeof(buf), timeout);
	//PX4_INFO("Read buffer size: %d", ret);
	
	if (ret > 0) {
		while (j < ret) {
			if (ret == 77) {
				if (decode(buf, buf_search_byte, 77)) {
					return 1;
				} else {
					return 0;
				}
			}
			//PX4_INFO("Buffer serach index: %d", buf_search_byte);
			/*if (buf[buf_search_byte] == '$') {
				PX4_INFO("Find start simbol $");
				//PX4_INFO("String: %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", (char)buf[buf_search_byte+0], (char)buf[buf_search_byte+1], (char)buf[buf_search_byte+2], (char)buf[buf_search_byte+3], (char)buf[buf_search_byte+4], (char)buf[buf_search_byte+5], (char)buf[buf_search_byte+6], (char)buf[buf_search_byte+7], (char)buf[buf_search_byte+8], (char)buf[buf_search_byte+9], (char)buf[buf_search_byte+10], (char)buf[buf_search_byte+11], (char)buf[buf_search_byte+12], (char)buf[buf_search_byte+13], (char)buf[buf_search_byte+14], (char)buf[buf_search_byte+15], (char)buf[buf_search_byte+16], (char)buf[buf_search_byte+17], (char)buf[buf_search_byte+18], (char)buf[buf_search_byte+19], (char)buf[buf_search_byte+20], (char)buf[buf_search_byte+21], (char)buf[buf_search_byte+22], (char)buf[buf_search_byte+23], (char)buf[buf_search_byte+24], (char)buf[buf_search_byte+25], (char)buf[buf_search_byte+26], (char)buf[buf_search_byte+27], (char)buf[buf_search_byte+28], (char)buf[buf_search_byte+29], (char)buf[buf_search_byte+30], (char)buf[buf_search_byte+31], (char)buf[buf_search_byte+32], (char)buf[buf_search_byte+33], (char)buf[buf_search_byte+34], (char)buf[buf_search_byte+35], (char)buf[buf_search_byte+36], (char)buf[buf_search_byte+37], (char)buf[buf_search_byte+38], (char)buf[buf_search_byte+39], (char)buf[buf_search_byte+40], (char)buf[buf_search_byte+41], (char)buf[buf_search_byte+42], (char)buf[buf_search_byte+43], (char)buf[buf_search_byte+44], (char)buf[buf_search_byte+45], (char)buf[buf_search_byte+46], (char)buf[buf_search_byte+47], (char)buf[buf_search_byte+48], (char)buf[buf_search_byte+49], (char)buf[buf_search_byte+50], (char)buf[buf_search_byte+51], (char)buf[buf_search_byte+52], (char)buf[buf_search_byte+53], (char)buf[buf_search_byte+54], (char)buf[buf_search_byte+55], (char)buf[buf_search_byte+56], (char)buf[buf_search_byte+57], (char)buf[buf_search_byte+58], (char)buf[buf_search_byte+59], (char)buf[buf_search_byte+60], (char)buf[buf_search_byte+61], (char)buf[buf_search_byte+62], (char)buf[buf_search_byte+63], (char)buf[buf_search_byte+64], (char)buf[buf_search_byte+65], (char)buf[buf_search_byte+66], (char)buf[buf_search_byte+67], (char)buf[buf_search_byte+68], (char)buf[buf_search_byte+69], (char)buf[buf_search_byte+70], (char)buf[buf_search_byte+71], (char)buf[buf_search_byte+72]);
				len_counter = 0;
				msg_start = buf_search_byte;
			} else if (buf[buf_search_byte] == '*') {
				PX4_INFO("Find end simbol *. Message size: %d", (int)len_counter);
				if (len_counter != -1) {
					msg_end = buf_search_byte;
				}
			}
			len_counter++;
			//PX4_INFO("Search iteration with, search_byte/buf_value: %d, %c", buf_search_byte, (char)buf[buf_search_byte]);

			if (msg_start > -1 && msg_end > -1 && len_counter == 73) {
				buffer_start = (buffer_start + ret) % GPS_READ_BUFFER_SIZE;

				PX4_INFO("Try decode with, start, stop: %d, %d", msg_start, msg_end);
				buf_search_byte = buffer_start;
				if (decode(buf, msg_start, len_counter)) {
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
			

			buf_search_byte = (buf_search_byte + 1) % GPS_READ_BUFFER_SIZE;*/
			j++;
		}
		buf_search_byte += ret;
		j = 0;
	}

	return 1;
}


int
GPSDriverNMEA::decode(char *message, int msg_start, int message_len) {
	parse_search_byte = 0;

	if (message[parse_search_byte + msg_start] == '$') {
		coma_point = 0xff;
		char_point = 0;
	}

	while (parse_search_byte < message_len) {
		if (parse_search_byte < 6 && parse_search_byte > 1){
			if (message[parse_search_byte + msg_start] != msg_type[parse_search_byte - 2]) {
				PX4_INFO("Out from decode");
				return 0;
			}
		}

		//PX4_INFO("Working char: %c", (char)message[parse_search_byte + msg_start]);
		if (message[parse_search_byte + msg_start] == ',') {
			coma_point++;
			char_point = 0;
		} else if (message[parse_search_byte + msg_start] == '*') {
			break;
		} else {
			GGA[coma_point][char_point++] = message[parse_search_byte + msg_start];
		}

		parse_search_byte++;
	}

	double lat = std::strtod(GGA[1], NULL);
	double lon = std::strtod(GGA[3], NULL);
	double alt = std::strtod(GGA[8], NULL);

	//PX4_INFO("Lat, lon, alt, values: %8.6f, %8.6f, %8.6f", (double)lat, (double)lon, (double)alt);
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