#ifndef __CMD_H
#define __CMD_H


#include "mac_packet.h"
#include "cmd_const.h"

#define CMD_TEST_RADIO			0x00
#define CMD_SET_THRUST_OPENLOOP     0x80

void cmdSetup(void);
void cmdPushFunc(MacPacket rx_packet);


#endif // __CMD_H