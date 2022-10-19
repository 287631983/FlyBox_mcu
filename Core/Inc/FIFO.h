/*
 * FIFO.h
 *
 *  Created on: 2022年10月12日
 *      Author: fj
 */

#ifndef FIFO_H_
#define FIFO_H_

#include <stdint.h>
#include <deque>

#ifdef __cplusplus
class FIFO {
	public:
		FIFO(uint16_t size);
		~FIFO();
		void clear();
		bool isQueueEmpty();
		bool isQueueFull();
		bool push(uint8_t);
		bool pull(uint8_t&);

	private:
		uint8_t* _queue = nullptr;
		uint16_t _capacity = 0;
		uint16_t _fifo_len = 0;
		uint16_t _head_pos = 0;
		uint16_t _tail_pos = 0;
};
#endif

#endif /* FIFO_H_ */
