/*
 * FIFO.h
 *
 *  Created on: 2022年10月12日
 *      Author: fj
 */

#ifndef FIFO_H_
#define FIFO_H_

#include <stdint.h>

#define FIFO_LENGTH 512

class FIFO {
	public:
		bool isQueueEmpty();
		bool isQueueFull();
		void pushMsg(uint8_t *, uint16_t);
		bool pullMsg(uint8_t&);

	private:
		uint8_t queue[FIFO_LENGTH];
		uint16_t _fifo_len = 0;
		uint16_t _head_pos = 0;
		uint16_t _tail_pos = 0;
};

#endif /* FIFO_H_ */
