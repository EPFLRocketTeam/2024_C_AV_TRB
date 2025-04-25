/*
 * serial_print.h
 *
 *  Created on: Apr 26, 2025
 *      Author: c.lacas
 */

#ifndef INC_SERIAL_PRINT_H_
#define INC_SERIAL_PRINT_H_

#include <stdint.h>

void serial_print(const char* msg);
void serial_println(const char* msg);
void serial_print_n(const int val, const uint8_t base);
void serial_println_n(const int val, const uint8_t base);

#endif /* INC_SERIAL_PRINT_H_ */
