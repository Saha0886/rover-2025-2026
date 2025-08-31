//
// Created by glebt on 17.05.2025.
//

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdint.h>

typedef struct
{
   uint8_t core_id;
   uint8_t cmd;
   uint8_t data[4];
} protocol ;

protocol create_data(const uint8_t core_id, uint8_t cmd, uint8_t data[]);

void create_msg(protocol data, uint8_t* tx_buffer);

void parse_msg(protocol *data, const uint8_t* rx_buffer);

void float_to_bytes(const float x, uint8_t* bytes);

float bytes_to_float(const uint8_t* bytes);

void int_32_to_bytes(const int32_t x, uint8_t* bytes);

int32_t bytes_to_int_32(const uint8_t* bytes);


#endif //UART_PROTOCOL_H
