#ifndef TYPES_H
#define TYPES_H

#include <time.h>
#include <stdint.h>

struct SerialPortData {
	int index;
	uint8_t sensor_cels;
	uint8_t sensor_raw;
	uint8_t servo;
	uint8_t selected_cels;

	SerialPortData(int fromIndex = 0) {
		index = fromIndex;
		sensor_cels = 0;
		sensor_raw = 0;
		servo = 0;
		selected_cels = 0;
	}
};

#endif // TYPES_H
