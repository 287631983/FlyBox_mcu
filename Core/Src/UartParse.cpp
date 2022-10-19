/*
 * UartParse.cpp
 *
 *  Created on: Oct 14, 2022
 *      Author: fj
 */

#include "UartParse.h"
#include "main.h"
#include <iostream>

extern void prepareDshotPacket(const uint16_t, uint16_t *);
extern uint16_t ESC_CMD_MOTOR1[ESC_CMD_BUF_LEN];
extern uint16_t ESC_CMD_MOTOR2[ESC_CMD_BUF_LEN];
extern uint16_t ESC_CMD_MOTOR3[ESC_CMD_BUF_LEN];
extern uint16_t ESC_CMD_MOTOR4[ESC_CMD_BUF_LEN];

#define OFFSET_FRAME_HEADER	  	0
#define OFFSET_DATA_LEN 	    2
#define OFFSET_CMD_ID           3
#define OFFSET_DATA 			4

#define CMD_ID_THROTTLE  0x01

UartParse::UartParse() {
	// TODO Auto-generated constructor stub
	_process_data = {
		{CMD_ID_THROTTLE, std::bind(&UartParse::_update_throttle, this, std::placeholders::_1, std::placeholders::_2)},
	};
}


UartParse::~UartParse() {
	// TODO Auto-generated destructor stub
}

void UartParse::PushMsg(uint8_t* msg, uint16_t len) {
	if (msg == nullptr) {
		return;
	}
	std::vector<uint8_t> one_msg;
	for (uint16_t i = 0; i < len; i++) {
		one_msg.push_back(msg[i]);
	}

	_msg_queue.push_back(one_msg);
}


bool UartParse::IfHasNewMsg() {
	if (_msg_queue.empty()) { return false; } 
	else 					{ return true; }
}


void UartParse::ProcessOneMsg() {
	uint8_t cmd_id;
	uint8_t msg_data_len = 0;
	auto msg = _msg_queue.front();
	_msg_queue.pop_front();

	// 校验数据头
	if (msg.at(0) != 0x07 && msg.at(1) != 0x09) {
		return;
	}
	if (msg.size() == 0) { return; }

	// 校验checksum
	msg_data_len = msg.at(2);
	uint16_t checksum = 0;
	for (int i = OFFSET_DATA; i < OFFSET_DATA + msg_data_len; i++) {
		checksum += msg.at(i);
	}
	if (checksum != (static_cast<uint16_t>(msg.at(4+msg_data_len) << 8) + msg.at(5+msg_data_len))) {
		return;
	}

	// 校验CMD_ID
	cmd_id = msg.at(3);
	auto iter = _process_data.find(cmd_id);
	if (iter == _process_data.end()) {
        return;
	}

	iter->second(msg.data() + OFFSET_DATA, msg_data_len);
}


void UartParse::_update_throttle(const uint8_t* data, const uint8_t& len) {
	uint16_t t1, t2, t3, t4;
	t1 = (static_cast<uint16_t>(data[0]) << 8) + data[1];
	t2 = (static_cast<uint16_t>(data[2]) << 8) + data[3];
	t3 = (static_cast<uint16_t>(data[4]) << 8) + data[5];
	t4 = (static_cast<uint16_t>(data[6]) << 8) + data[7];

	prepareDshotPacket(t1, ESC_CMD_MOTOR1);
	prepareDshotPacket(t2, ESC_CMD_MOTOR2);
	prepareDshotPacket(t3, ESC_CMD_MOTOR3);
	prepareDshotPacket(t4, ESC_CMD_MOTOR4);
}
