#include "FIFO.h"

bool FIFO::isQueueFull() {
    if (((_tail_pos+1)%FIFO_LENGTH) == _head_pos) {
        return 0;
    } else {
        return 1;
    }
}


bool FIFO::isQueueEmpty() {
    if (_head_pos == _tail_pos) {
        return 1;
    } else {
        return 0;
    }
}


void FIFO::pushMsg(uint8_t *msg, uint16_t len) {
    if (len == 0) {
        return;
    }
    if (msg == nullptr) {
        return;
    }
    if (isQueueFull()) {
        return;
    }
    for (uint16_t i = 0; i < len; i++) {
        queue[_tail_pos ++] = msg[i];
        if (_tail_pos >= FIFO_LENGTH) {
            _tail_pos = 0;
        }
    }
}

bool FIFO::pullMsg(uint8_t& data) {
    if (isQueueFull()) {
        return false;
    }

	data = queue[_head_pos ++];
	if (_head_pos >= FIFO_LENGTH) {
		_head_pos = 0;
	}
	if (isQueueFull()) {
		
	}
}