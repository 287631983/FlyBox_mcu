#include "FIFO.h"

FIFO::FIFO(uint16_t size) {
    _queue = new uint8_t[size];
    _capacity = size;
}


FIFO::~FIFO() {
    delete[] _queue;
    _queue = nullptr;
}


bool FIFO::isQueueFull() {
    if (((_tail_pos+1)%_capacity) == _head_pos) {
        return 0;
    } else {
        return 1;
    }
}


void FIFO::clear() {
    _head_pos = 0;
    _tail_pos = 0;
    _fifo_len = 0;
}


bool FIFO::isQueueEmpty() {
    if (_head_pos == _tail_pos) {
        return 1;
    } else {
        return 0;
    }
}


bool FIFO::push(uint8_t elem) {
    if (isQueueFull()) {
        return false;
    }
    _queue[_tail_pos++] = elem;
    _tail_pos = _tail_pos % _capacity;
    _fifo_len ++;

    return true;
}


bool FIFO::pull(uint8_t& data) {
    if (isQueueFull()) {
        return false;
    }

	data = _queue[_head_pos ++];
    _head_pos = _head_pos % _capacity;
    _fifo_len --;
    return true;
}