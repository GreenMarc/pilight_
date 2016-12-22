/*
	Copyright (C) 2014 CurlyMo & GreenMarc

	This file is part of pilight.

	pilight is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software
	Foundation, either version 3 of the License, or (at your option) any later
	version.

	pilight is distributed in the hope that it will be useful, but WITHOUT ANY
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
	A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with pilight. If not, see	<http://www.gnu.org/licenses/>
*/

#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/time.h>
#ifndef _WIN32
	#ifdef __mips__
		#define __USE_UNIX98
	#endif
#endif
#include <pthread.h>

#include "../../core/pilight.h"
#include "../../core/common.h"
#include "../../core/dso.h"
#include "../../core/log.h"
#include "../../core/threads.h"
#include "../../core/binary.h"
#include "../../core/gc.h"
#include "../../core/json.h"
#include "../protocol.h"
#include "bme280.h"

#if !defined(__FreeBSD__) && !defined(_WIN32)
#include "../../../wiringx/wiringX.h"

typedef struct settings_t {
	char **id;
	int nrid;
	int *fd;
	// calibration values (stored in each BME280)
	unsigned short *dig_T1;
	short *dig_T2;
	short *dig_T3;
	unsigned short *dig_P1;
  short	*dig_P2;
  short *dig_P3;
  short *dig_P4;
  short	*dig_P5;
  short *dig_P6;
  short *dig_P7;
  short	*dig_P8;
  short *dig_P9;
  unsigned char *dig_H1;
  short *dig_H2;
  unsigned char *dig_H3;
  short *dig_H4;
  short *dig_H5;
  char *dig_H6;
} settings_t;

static unsigned short loop = 1;
static int threads = 0;

static pthread_mutex_t lock;
static pthread_mutexattr_t attr;

// helper function with built-in result conversion
//static int readReg16(int fd, int reg) {
//	int res = wiringXI2CReadReg16(fd, reg);
//	// convert result to 16 bits and swap bytes
//	return ((res << 8) & 0xFF00) | ((res >> 8) & 0xFF);
//}

static void *thread(void *param) {
	struct protocol_threads_t *node = (struct protocol_threads_t *) param;
	struct JsonNode *json = (struct JsonNode *) node->param;
	struct JsonNode *jid = NULL;
	struct JsonNode *jchild = NULL;
	struct settings_t *bme280data = MALLOC(sizeof(struct settings_t));
	int y = 0, interval = 10, nrloops = 0, data = 0;
	char *stmp = NULL;
	double itmp = -1, temp_offset = 0, pressure_offset = 0;
	unsigned char oversampling = 1;

	if(bme280data == NULL) {
		fprintf(stderr, "out of memory\n");
		exit(EXIT_FAILURE);
	}

	bme280data->nrid = 0;
	bme280data->id = NULL;
	bme280data->fd = 0;
	bme280data->dig_T1 = 0;
	bme280data->dig_T2 = 0;
	bme280data->dig_T3 = 0;
	bme280data->dig_P1 = 0;
	bme280data->dig_P2 = 0;
	bme280data->dig_P3 = 0;
	bme280data->dig_P4 = 0;
	bme280data->dig_P5 = 0;
	bme280data->dig_P6 = 0;
	bme280data->dig_P7 = 0;
	bme280data->dig_P8 = 0;
	bme280data->dig_P9 = 0;
	bme280data->dig_H1 = 0;
	bme280data->dig_H2 = 0;
	bme280data->dig_H3 = 0;
	bme280data->dig_H4 = 0;
	bme280data->dig_H5 = 0;
	bme280data->dig_H6 = 0;
	
	threads++;

	if((jid = json_find_member(json, "id"))) {
		jchild = json_first_child(jid);
		while(jchild) {
			if(json_find_string(jchild, "id", &stmp) == 0) {
				if((bme280data->id = REALLOC(bme280data->id, (sizeof(char *) * (size_t)(bme280data->nrid + 1)))) == NULL) {
					fprintf(stderr, "out of memory\n");
					exit(EXIT_FAILURE);
				}
				if((bme280data->id[bme280data->nrid] = MALLOC(strlen(stmp) + 1)) == NULL) {
					fprintf(stderr, "out of memory\n");
					exit(EXIT_FAILURE);
				}
				strcpy(bme280data->id[bme280data->nrid], stmp);
				bme280data->nrid++;
			}
			jchild = jchild->next;
		}
	}

	if(json_find_number(json, "poll-interval", &itmp) == 0)
		interval = (int) round(itmp);
	json_find_number(json, "temperature-offset", &temp_offset);
	json_find_number(json, "pressure-offset", &pressure_offset);
	if(json_find_number(json, "oversampling", &itmp) == 0) {
		oversampling = (unsigned char) itmp;
	}

	// resize the memory blocks pointed to by the different pointers
	size_t sz = (size_t) (bme280data->nrid + 1);
	unsigned long int sizeShort = sizeof(short) * sz;
	unsigned long int sizeUShort = sizeof(unsigned short) * sz;
	unsigned long int sizeChar = sizeof(char) *sz;
	unsigned long int sizeUChar = sizeof(unsigned char) * sz;
	bme280data->fd = REALLOC(bme280data->fd, (sizeof(int) * sz));
	bme280data->dig_T1 = REALLOC(bme280data->dig_T1, sizeUShort);
	bme280data->dig_T2 = REALLOC(bme280data->dig_T2, sizeShort);
	bme280data->dig_T3 = REALLOC(bme280data->dig_T3, sizeShort);
	bme280data->dig_P1 = REALLOC(bme280data->dig_P1, sizeUShort);
	bme280data->dig_P2 = REALLOC(bme280data->dig_P2, sizeShort);
	bme280data->dig_P3 = REALLOC(bme280data->dig_P3, sizeShort);
	bme280data->dig_P4 = REALLOC(bme280data->dig_P4, sizeShort);
	bme280data->dig_P5 = REALLOC(bme280data->dig_P5, sizeShort);
	bme280data->dig_P6 = REALLOC(bme280data->dig_P6, sizeShort);
	bme280data->dig_P7 = REALLOC(bme280data->dig_P7, sizeShort);
	bme280data->dig_P8 = REALLOC(bme280data->dig_P8, sizeShort);
	bme280data->dig_P9 = REALLOC(bme280data->dig_P9, sizeShort);
	bme280data->dig_H1 = REALLOC(bme280data->dig_H1, sizeUChar);
	bme280data->dig_H2 = REALLOC(bme280data->dig_H2, sizeShort);
	bme280data->dig_H3 = REALLOC(bme280data->dig_H3, sizeUChar);
	bme280data->dig_H4 = REALLOC(bme280data->dig_H4, sizeShort);
	bme280data->dig_H5 = REALLOC(bme280data->dig_H5, sizeShort);
	bme280data->dig_H6 = REALLOC(bme280data->dig_H6, sizeChar);
	
	if(bme280data->dig_T1 == NULL || bme280data->dig_T2 == NULL || bme280data->dig_T3 == NULL || bme280data->dig_P1 == NULL ||
		 bme280data->dig_P2 == NULL || bme280data->dig_P3 == NULL || bme280data->dig_P4 == NULL || bme280data->dig_P5 == NULL ||
		 bme280data->dig_P6 == NULL || bme280data->dig_P7 == NULL || bme280data->dig_P8 == NULL || bme280data->dig_P9 == NULL ||
		 bme280data->dig_H1 == NULL || bme280data->dig_H2 == NULL || bme280data->dig_H3 == NULL || bme280data->dig_H4 == NULL || 
		 bme280data->dig_H5 == NULL || bme280data->dig_H6 == NULL ||	bme280data->fd == NULL) {
		fprintf(stderr, "out of memory\n");
		exit(EXIT_FAILURE);
	}

	for(y = 0; y < bme280data->nrid; y++) {
		// setup i2c
		bme280data->fd[y] = wiringXI2CSetup((int)strtol(bme280data->id[y], NULL, 16));
		if(bme280data->fd[y] > 0) {
			// read 0xD0 to check chip id: must equal 0x60
			int id = wiringXI2CReadReg8(bme280data->fd[y], 0xd0);
			if(id != 0x60) {
				logprintf(LOG_ERR, "wrong device detected");
				exit(EXIT_FAILURE);
			}

			// read calibration coefficients from register addresses
			bme280data->dig_T1[y] = (unsigned short) wiringXI2CReadReg16(bme280data->fd[y], 0x88);
			bme280data->dig_T2[y] = (short) wiringXI2CReadReg16(bme280data->fd[y], 0x8a);
			bme280data->dig_T3[y] = (short) wiringXI2CReadReg16(bme280data->fd[y], 0x8c);
			bme280data->dig_P1[y] = (unsigned short) wiringXI2CReadReg16(bme280data->fd[y], 0x8e);
			bme280data->dig_P2[y] = (short) wiringXI2CReadReg16(bme280data->fd[y], 0x90);
			bme280data->dig_P3[y] = (short) wiringXI2CReadReg16(bme280data->fd[y], 0x92);
			bme280data->dig_P4[y] = (short) wiringXI2CReadReg16(bme280data->fd[y], 0x94);
			bme280data->dig_P5[y] = (short) wiringXI2CReadReg16(bme280data->fd[y], 0x96);
			bme280data->dig_P6[y] = (short) wiringXI2CReadReg16(bme280data->fd[y], 0x98);
			bme280data->dig_P7[y] = (short) wiringXI2CReadReg16(bme280data->fd[y], 0x9a);
			bme280data->dig_P8[y] = (short) wiringXI2CReadReg16(bme280data->fd[y], 0x9c);
			bme280data->dig_P9[y] = (short) wiringXI2CReadReg16(bme280data->fd[y], 0x9e);
			bme280data->dig_H1[y] = (unsigned char) wiringXI2CReadReg8(bme280data->fd[y], 0xa1);
			bme280data->dig_H2[y] = (short) wiringXI2CReadReg16(bme280data->fd[y], 0xe1);
			bme280data->dig_H3[y] = (unsigned char) wiringXI2CReadReg8(bme280data->fd[y], 0xe3);
			bme280data->dig_H4[y] = (short) ((wiringXI2CReadReg8(bme280data->fd[y],0xe4) << 4) | (wiringXI2CReadReg8(bme280data->fd[y],0xe5) & 0xF));
			bme280data->dig_H5[y] = (short) ((wiringXI2CReadReg8(bme280data->fd[y],0xe6) << 4) | (wiringXI2CReadReg8(bme280data->fd[y],0xe5) >> 4));
			bme280data->dig_H6[y] = (char) wiringXI2CReadReg8(bme280data->fd[y],0xe7);

			// check communication: no result must equal 0 or 0xFFFF (=65535)
			if (bme280data->dig_T1[y] == 0 || bme280data->dig_T1[y] == 0xFFFF ||
				  bme280data->dig_T2[y] == 0 || bme280data->dig_T2[y] == 0xFFFF ||
				  bme280data->dig_T3[y] == 0 || bme280data->dig_T3[y] == 0xFFFF ||
				  bme280data->dig_P1[y] == 0 || bme280data->dig_P1[y] == 0xFFFF ||
				  bme280data->dig_P2[y] == 0 || bme280data->dig_P2[y] == 0xFFFF ||
				  bme280data->dig_P3[y] == 0 || bme280data->dig_P3[y] == 0xFFFF ||
				  bme280data->dig_P4[y] == 0 || bme280data->dig_P4[y] == 0xFFFF ||
				  bme280data->dig_P5[y] == 0 || bme280data->dig_P5[y] == 0xFFFF ||
				  bme280data->dig_P6[y] == 0 || bme280data->dig_P6[y] == 0xFFFF ||
				  bme280data->dig_P7[y] == 0 || bme280data->dig_P7[y] == 0xFFFF ||
				  bme280data->dig_P8[y] == 0 || bme280data->dig_P8[y] == 0xFFFF ||
				  bme280data->dig_P9[y] == 0 || bme280data->dig_P9[y] == 0xFFFF ||
				  bme280data->dig_H1[y] == 0 || bme280data->dig_H1[y] == 0xFFFF ||
				  bme280data->dig_H2[y] == 0 || bme280data->dig_H2[y] == 0xFFFF ||
				  bme280data->dig_H3[y] == 0 || bme280data->dig_H3[y] == 0xFFFF ||
				  bme280data->dig_H4[y] == 0 || bme280data->dig_H4[y] == 0xFFFF ||
				  bme280data->dig_H5[y] == 0 || bme280data->dig_H5[y] == 0xFFFF ||
				  bme280data->dig_H6[y] == 0 || bme280data->dig_H6[y] == 0xFFFF ){
				logprintf(LOG_ERR, "data communication error");
				exit(EXIT_FAILURE);
			}
		}
	}

	while (loop) {
		if (protocol_thread_wait(node, interval, &nrloops) == ETIMEDOUT) {
			pthread_mutex_lock(&lock);
			for (y = 0; y < bme280data->nrid; y++) {
				if (bme280data->fd[y] > 0) {
					// uncompensated temperature value
					int utemp = 0;
					
					//oversampling kommt später noch auswählbar....
					int osrs_t = 2, osrs_p = 2, osrs_h = 2, mode = 1;
					
					// write data into Register 0xF4 to request a temperature & presure reading.
					data = (osrs_t<<5 | osrs_p<<2 | mode);
    			wiringXI2CWriteReg8(bme280data->fd[y], 0xf4, data) ;
    			    			
					// wait at least 45ms: we suspend execution for 50000 microseconds.
					usleep(50000);

					// read the three byte result from address 0xFA to 0xFC.
					int utmsb = wiringXI2CReadReg8(bme280data->fd[y], 0xfa);
    			int utlsb = wiringXI2CReadReg8(bme280data->fd[y], 0xfb);
    			int utxlsb = wiringXI2CReadReg8(bme280data->fd[y], 0xfc);					
					utemp = (((unsigned int) utmsb << 12) | ((unsigned int) utlsb << 4) | (unsigned int) utxlsb >> 4);

					// calculate temperature (in units of 0.1 deg C) given uncompensated value
					long int var1, var2, tfine;
					var1 = ((((utemp >> 3) - (bme280data->dig_T1[y] << 1))) * (bme280data->dig_T2[y])) >> 11 ;
	  			var2 = (((((utemp >> 4) - (bme280data->dig_T1[y])) * ((utemp >> 4) - (bme280data->dig_T1[y]))) >> 12) * (bme280data->dig_T3[y])) >> 14 ;
	  			tfine = var1 + var2;
	  			int temp = ((tfine * 5 + 128) >> 8);

					// uncompensated pressure value
					unsigned int upress = 0;
					
					// read the three byte result from address 0xF7 to 0xF9.
    			int upmsb = wiringXI2CReadReg8(bme280data->fd[y], 0xf7);
    			int uplsb = wiringXI2CReadReg8(bme280data->fd[y], 0xf8);
    			int upxlsb = wiringXI2CReadReg8(bme280data->fd[y], 0xf9);
		      upress = (((unsigned int) upmsb << 12) | ((unsigned int) uplsb << 4) | (unsigned int) upxlsb >> 4);
				
					// calculate pressure (in Pa) given uncompensated value
					
					var1 = (tfine >> 1) - 64000;
					var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * bme280data->dig_P6[y];
					var2 = var2 + ((var1 * bme280data->dig_P5[y]) << 1);
					var2 = (var2 >> 2) + (bme280data->dig_P4[y] << 16);
					var1 = (((bme280data->dig_P3[y] * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((bme280data->dig_P2[y] * var1) >> 1)) >> 18;
					var1 = (((32768 + var1)* bme280data->dig_P1[y]) >> 15);
					if (var1 == 0) {
						return 0; // avoid exception caused by division by zero ** fix me **
					}
					unsigned int p = ((1048576 - upress) - (var2 >> 12)) * 3125;
					if (p < 0x80000000)	{
						p = (p << 1) / var1;
					}
					else {
						p = (p / var1) * 2;
					}
					var1 = (bme280data->dig_P9[y] * (((p >> 3) * (p >> 3)) >> 13)) >> 12;	
					var2 = (((int)(p>>2)) * bme280data->dig_P8[y]) >> 13;
					int pres = ((int)p + ((var1 + var2 + bme280data->dig_P7[y]) >> 4));
		
					
					bme280->message = json_mkobject();
					JsonNode *code = json_mkobject();
					json_append_member(code, "id", json_mkstring(bme280data->id[y]));
					json_append_member(code, "temperature", json_mknumber(((double) temp / 100) + temp_offset, 1)); // in deg C
					json_append_member(code, "pressure", json_mknumber(((double) pres / 100) + pressure_offset, 1)); // in hPa

					json_append_member(bme280->message, "message", code);
					json_append_member(bme280->message, "origin", json_mkstring("receiver"));
					json_append_member(bme280->message, "protocol", json_mkstring(bme280->id));

					if(pilight.broadcast != NULL) {
						pilight.broadcast(bme280->id, bme280->message, PROTOCOL);
					}
					json_delete(bme280->message);
					bme280->message = NULL;
				} else {
					logprintf(LOG_NOTICE, "error connecting to bme280");
					logprintf(LOG_DEBUG, "(probably i2c bus error from wiringXI2CSetup)");
					logprintf(LOG_DEBUG, "(maybe wrong id? use i2cdetect to find out)");
					protocol_thread_wait(node, 1, &nrloops);
				}
			}
			pthread_mutex_unlock(&lock);
		}
	}

	if (bme280data->id) {
		for (y = 0; y < bme280data->nrid; y++) {
			FREE(bme280data->id[y]);
		}
		FREE(bme280data->id);
	}
	
	if (bme280data->dig_T1) {
		FREE(bme280data->dig_T1);
	}

	if (bme280data->dig_T2) {
		FREE(bme280data->dig_T2);
	}
	if (bme280data->dig_T3) {
		FREE(bme280data->dig_T3);
	}

	if (bme280data->dig_P1) {
		FREE(bme280data->dig_P1);
	}

	if (bme280data->dig_P2) {
		FREE(bme280data->dig_P2);
	}

	if (bme280data->dig_P3) {
		FREE(bme280data->dig_P3);
	}

	if (bme280data->dig_P4) {
		FREE(bme280data->dig_P4);
	}

	if (bme280data->dig_P5) {
		FREE(bme280data->dig_P5);
	}

	if (bme280data->dig_P6) {
		FREE(bme280data->dig_P6);
	}

	if (bme280data->dig_P7) {
		FREE(bme280data->dig_P7);
	}

	if (bme280data->dig_P8) {
		FREE(bme280data->dig_P8);
	}

	if (bme280data->dig_P9) {
		FREE(bme280data->dig_P9);
	}

	if (bme280data->dig_H1) {
		FREE(bme280data->dig_H1);
	}

	if (bme280data->dig_H2) {
		FREE(bme280data->dig_H2);
	}

	if (bme280data->dig_H3) {
		FREE(bme280data->dig_H3);
	}

	if (bme280data->dig_H4) {
		FREE(bme280data->dig_H4);
	}

	if (bme280data->dig_H5) {
		FREE(bme280data->dig_H5);
	}

	if (bme280data->dig_H6) {
		FREE(bme280data->dig_H6);
	}
	
	if (bme280data->fd) {
		for (y = 0; y < bme280data->nrid; y++) {
			if (bme280data->fd[y] > 0) {
				close(bme280data->fd[y]);
			}
		}
		FREE(bme280data->fd);
	}
	FREE(bme280data);
	threads--;

	return (void *) NULL;
}

static struct threadqueue_t *initDev(JsonNode *jdevice) {
	if(wiringXSupported() == 0 && wiringXSetup() == 0) {
		loop = 1;
		char *output = json_stringify(jdevice, NULL);
		JsonNode *json = json_decode(output);
		json_free(output);

		struct protocol_threads_t *node = protocol_thread_init(bme280, json);
		return threads_register("bme280", &thread, (void *) node, 0);
	} else {
		return NULL;
	}
}

static void threadGC(void) {
	loop = 0;
	protocol_thread_stop(bme280);
	while (threads > 0) {
		usleep(10);
	}
	protocol_thread_free(bme280);
}
#endif

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif
void bme280Init(void) {
#if !defined(__FreeBSD__) && !defined(_WIN32)
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&lock, &attr);
#endif

	protocol_register(&bme280);
	protocol_set_id(bme280, "bme280");
	protocol_device_add(bme280, "bme280", "I2C Barometric Pressure,Humidy and Temperature Sensor");
//	protocol_device_add(bme280, "bmp280", "I2C Barometric Pressure and Temperature Sensor");
	bme280->devtype = WEATHER;
	bme280->hwtype = SENSOR;

	options_add(&bme280->options, 'i', "id", OPTION_HAS_VALUE, DEVICES_ID, JSON_STRING, NULL, "0x[0-9a-f]{2}");
	options_add(&bme280->options, 'o', "oversampling", OPTION_HAS_VALUE, DEVICES_SETTING, JSON_NUMBER, (void *) 1, "^[0123]$");
	options_add(&bme280->options, 'p', "pressure", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, (void *) 0, "^[0-9]{1,3}$");
	options_add(&bme280->options, 't', "temperature", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, (void *) 0, "^[0-9]{1,3}$");
	//options_add(&bme280->options, 'h', "humidity", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, (void *) 0, "^[0-9]{1,3}$");

	options_add(&bme280->options, 0, "poll-interval", OPTION_HAS_VALUE, DEVICES_SETTING, JSON_NUMBER, (void *) 10, "[0-9]");
	options_add(&bme280->options, 0, "pressure-offset", OPTION_HAS_VALUE, DEVICES_SETTING, JSON_NUMBER, (void *) 0, "[0-9]");
	options_add(&bme280->options, 0, "temperature-offset", OPTION_HAS_VALUE, DEVICES_SETTING, JSON_NUMBER, (void *) 0, "[0-9]");
	options_add(&bme280->options, 0, "temperature-decimals", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *) 1, "[0-9]");
	options_add(&bme280->options, 0, "humidity-decimals", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *) 1, "[0-9]");
	options_add(&bme280->options, 0, "pressure-decimals", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *) 1, "[0-9]");
	options_add(&bme280->options, 0, "show-pressure", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *) 1, "^[10]{1}$");
	options_add(&bme280->options, 0, "show-temperature", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *) 1, "^[10]{1}$");

#if !defined(__FreeBSD__) && !defined(_WIN32)
	bme280->initDev = &initDev;
	bme280->threadGC = &threadGC;
#endif
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module) {
	module->name = "bme280";
	module->version = "1.0";
	module->reqversion = "7.0";
	module->reqcommit = "84";
}

void init(void) {
	bme280Init();
}
#endif
