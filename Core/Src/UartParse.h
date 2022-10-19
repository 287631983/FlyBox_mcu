/*
 * UartParse.h
 *
 *  Created on: Oct 14, 2022
 *      Author: fj
 */

#ifndef SRC_UARTPARSE_H_
#define SRC_UARTPARSE_H_

#include <stdint.h>
#include <deque>
#include <map>
#include <functional>
#include <vector>
class UartParse {
public:
	UartParse();
	virtual ~UartParse();

	void PushMsg(uint8_t* msg, uint16_t len);

	bool IfHasNewMsg();

	void ProcessOneMsg();

private:
	std::map<uint8_t, const std::function<void(const uint8_t*, const uint8_t&)>> _process_data;
	std::deque<std::vector<uint8_t>> _msg_queue; 

	void _update_throttle(const uint8_t*, const uint8_t&);
};

#endif /* SRC_UARTPARSE_H_ */
