#ifndef INIT_H__
#define INIT_H__

#include "serial.h"
#include "radio.h"

/**
 * Perform global initialization.
 *
 * The two handlers are used to receive data from
 * the serial and radio communications correspondingly.
 */
void global_init(serial_handler_t* serial_handler,
                 radio_handler_t* radio_handler);

#endif // INIT_H__
