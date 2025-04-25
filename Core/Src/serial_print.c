/*
 * serial_print.c
 *
 *  Created on: Apr 26, 2025
 *      Author: c.lacas
 */

#include "serial_print.h"
#include "usbd_cdc_if.h"

static char serial_buffer[APP_RX_DATA_SIZE];

void serial_print(const char* msg) {
	memcpy(serial_buffer, msg, strlen(msg)+1);
	vcp_send(serial_buffer, strlen(serial_buffer));
}

void serial_println(const char* msg) {
	memcpy(serial_buffer, msg, strlen(msg)+1);
	strcat(serial_buffer, "\r\n");
	vcp_send(serial_buffer, strlen(serial_buffer));
}

void serial_print_n(const int val, const uint8_t base) {
	itoa(val, serial_buffer, base);
	vcp_send(serial_buffer, strlen(serial_buffer));
}

void serial_println_n(const int val, const uint8_t base) {
	itoa(val, serial_buffer, base);
	strcat(serial_buffer, "\r\n");
	vcp_send(serial_buffer, strlen(serial_buffer));
}


