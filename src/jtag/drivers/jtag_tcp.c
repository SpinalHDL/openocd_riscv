/***************************************************************************
 *   Copyright (C) 2008 by Øyvind Harboe                                   *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <stdio.h>
#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <netinet/tcp.h>
#endif
#include <string.h>
#include <fcntl.h>
#include "hello.h"

/* my private tap controller state, which tracks state for calling code */
static tap_state_t jtag_tcp_state;

int clientSocket;

#define TMS_SET 1
#define TDI_SET 2
#define TDO_READ 4
#define TCK_SET 8

static int jtag_tcp_khz(int khz, int *jtag_speed)
{
	if (khz == 0)
		*jtag_speed = 0;
	else
		*jtag_speed = 64000/khz;
	return ERROR_OK;
}

static int jtag_tcp_speed_div(int speed, int *khz)
{
	if (speed == 0)
		*khz = 0;
	else
		*khz = 64000/speed;

	return ERROR_OK;
}

static int jtag_tcp_speed(int speed)
{
	return ERROR_OK;
}

static int jtag_tcp_init(void)
{
	jtag_tcp_state = TAP_RESET;

	//---- Create the socket. The three arguments are: ----//
	// 1) Internet domain 2) Stream socket 3) Default protocol (TCP in this case) //
	clientSocket = socket(PF_INET, SOCK_STREAM, 0);

	int flag = 1;
	setsockopt(  clientSocket,            /* socket affected */
				 IPPROTO_TCP,     /* set option at TCP level */
				 TCP_NODELAY,     /* name of option */
				 (char *) &flag,  /* the cast is historical
										 cruft */
				 sizeof(int));    /* length of option value */


	/*int a = 0xFFFFFF;
	if (setsockopt(clientSocket, SOL_SOCKET, SO_RCVBUF, &a, sizeof(int)) == -1) {
	    fprintf(stderr, "Error setting socket opts: %s\n", strerror(errno));
	}
	a = 0xFFF;
	if (setsockopt(clientSocket, SOL_SOCKET, SO_SNDBUF, &a, sizeof(int)) == -1) {
	    fprintf(stderr, "Error setting socket opts: %s\n", strerror(errno));
	}*/


	//---- Configure settings of the server address struct ----//
	struct sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(7894);
	serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);

	//---- Connect the socket to the server using the address struct ----//
	socklen_t addr_size = sizeof serverAddr;
	if(connect(clientSocket, (struct sockaddr *) &serverAddr, addr_size)){
		LOG_ERROR("Can't connect to the TCP server");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int jtag_tcp_quit(void)
{
	return close(clientSocket);
}

/*
static int jtag_tcp_write(int tms, int tdi, int tdo, int tck){
	uint8_t buffer =

	return ERROR_OK;
}*/

static int jtag_tcp_reset(void)
{
	uint8_t buffer[10];
	for(uint32_t i = 0;i < 10;i+=2){
		buffer[i + 0] = TMS_SET;
		buffer[i + 1] = TMS_SET | TCK_SET;
	}

	if(send(clientSocket,(char*)buffer,10,0) <= 0)
		return ERROR_FAIL;

	tap_set_state(TAP_RESET);
    return ERROR_OK;
}

static int jtag_tcp_state_move(int skip)
{
	int i = 0, tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
	uint8_t buffer[tms_count*2 + 1];
	for (i = skip; i < tms_count; i++) {
		tms = (tms_scan >> i) & 1;
		buffer[i*2 + 0] =  tms ? TMS_SET : 0;
		buffer[i*2 + 1] = (tms ? TMS_SET : 0) | TCK_SET;
	}
	buffer[tms_count*2] = tms ? TMS_SET : 0;

	if(send(clientSocket,(char*)&buffer[skip*2],(tms_count - skip)*2 + 1, 0) <= 0)
		return ERROR_FAIL;

	tap_set_state(tap_get_end_state());
	return ERROR_OK;
}

static void jtag_tcp_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

static int jtag_tcp_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size)
{
	tap_state_t saved_end_state = tap_get_end_state();
	int bit_cnt;

	if (!((!ir_scan &&
			(tap_get_state() == TAP_DRSHIFT)) ||
			(ir_scan && (tap_get_state() == TAP_IRSHIFT)))) {
		if (ir_scan)
			jtag_tcp_end_state(TAP_IRSHIFT);
		else
			jtag_tcp_end_state(TAP_DRSHIFT);

		jtag_tcp_state_move(0);
		jtag_tcp_end_state(saved_end_state);
	}
	uint8_t txBuffer[scan_size*2];
	for (bit_cnt = 0; bit_cnt < scan_size; bit_cnt++) {
		int tms = (bit_cnt == scan_size-1 && tap_get_state() != tap_get_end_state()) ? 1 : 0;
		int tdi;
		int bytec = bit_cnt/8;
		int bcval = 1 << (bit_cnt % 8);

		/* if we're just reading the scan, but don't care about the output
		 * default to outputting 'low', this also makes valgrind traces more readable,
		 * as it removes the dependency on an uninitialised value
		 */
		tdi = 0;
		if ((type != SCAN_IN) && (buffer[bytec] & bcval))
			tdi = 1;

		txBuffer[bit_cnt*2 + 0] = (tms ? TMS_SET : 0) | (tdi ? TDI_SET : 0);
		txBuffer[bit_cnt*2 + 1] = (tms ? TMS_SET : 0) | (tdi ? TDI_SET : 0) | TCK_SET | (type != SCAN_OUT ? TDO_READ : 0);
	}

	if(send(clientSocket,(char*)txBuffer,scan_size*2, 0) <= 0)
		return ERROR_FAIL;


	if (tap_get_state() != tap_get_end_state()) {
		/* we *KNOW* the above loop transitioned out of
		 * the shift state, so we skip the first state
		 * and move directly to the end state.
		 */
		jtag_tcp_state_move(1);
	}

	return ERROR_OK;
}

static int jtag_tcp_scan_rsp(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size)
{
	int bit_cnt;

	if (type != SCAN_OUT) {
		for (bit_cnt = 0; bit_cnt < scan_size; bit_cnt++) {
			uint8_t rxBuffer;
			if(read(clientSocket,&rxBuffer,1) != 1)
				return ERROR_FAIL;
			int bytec = bit_cnt/8;
			int bcval = 1 << (bit_cnt % 8);
			if (rxBuffer & 1)
				buffer[bytec] |= bcval;
			else
				buffer[bytec] &= ~bcval;
		}
	}

	return ERROR_OK;
}

static int jtag_tcp_stableclocks(int num_cycles)
{
	int tms = (tap_get_state() == TAP_RESET ? 1 : 0);
	int i;
	uint8_t txBuffer[num_cycles*2];

	/* send num_cycles clocks onto the cable */
	for (i = 0; i < num_cycles; i++) {
		txBuffer[i*2 + 0] = (tms ? TMS_SET : 0);
		txBuffer[i*2 + 1] = (tms ? TMS_SET : 0) | TCK_SET;
	}

	if(send(clientSocket,(char*)txBuffer,num_cycles*2, 0) <= 0)
		return ERROR_FAIL;

	return ERROR_OK;

}

static int jtag_tcp_runtest(int num_cycles)
{

	int i;
	uint8_t txBuffer[num_cycles*2];

	/* Move to the run-test / idle state */
	jtag_tcp_end_state(TAP_IDLE);
	if (jtag_tcp_state_move(0) != ERROR_OK)
		return ERROR_FAIL;
	
	/* send num_cycles clocks onto the cable and stay in run-test */
	for (i = 0; i < num_cycles; i++) {
		txBuffer[i*2 + 0] = 0;
		txBuffer[i*2 + 1] = 0 | TCK_SET;
	}

	/* Send the cycles */
	if(send(clientSocket,(char*)txBuffer,num_cycles*2, 0) <= 0)
		return ERROR_FAIL;
  
	return ERROR_OK;
	
}

int jtag_tcp_execute_queue(struct jtag_command *cmd_queue)
{
	struct jtag_command *cmd;
	int retval = ERROR_OK;
	uint8_t *buffer;
	int scan_size;
	enum scan_type type;

	for (cmd = cmd_queue; retval == ERROR_OK && cmd != NULL;
	     cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RESET:
			retval = jtag_tcp_reset();
			break;
		case JTAG_TLR_RESET:
			jtag_tcp_end_state(cmd->cmd.statemove->end_state);
			jtag_tcp_state_move(0);
			break;
		case JTAG_SLEEP:
			//jtag_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			jtag_tcp_end_state(cmd->cmd.scan->end_state);
			scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
			type = jtag_scan_type(cmd->cmd.scan);
			if (jtag_tcp_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size) != ERROR_OK)
				retval = ERROR_JTAG_QUEUE_FAILED;
			if (buffer)
				free(buffer);
			break;
		case JTAG_STABLECLOCKS:
			retval = jtag_tcp_stableclocks(cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_RUNTEST:
		        retval = jtag_tcp_runtest(cmd->cmd.runtest->num_cycles);
		  
			break;
                        /*
		case JTAG_PATHMOVE:
			retval = jtag_vpi_path_move(cmd->cmd.pathmove);
			break;
		case JTAG_TMS:
			retval = jtag_vpi_tms(cmd->cmd.tms);
			break;*/
		default:
			LOG_ERROR("unknow cmd ???");
			retval = ERROR_FAIL;
			break;
		}
	}

	{
		uint8_t txBuffer = TDO_READ;
		if(send(clientSocket,(char*)&txBuffer,1, 0) <= 0)
			return ERROR_FAIL;
	}

	for (cmd = cmd_queue; retval == ERROR_OK && cmd != NULL;
	     cmd = cmd->next) {
		switch (cmd->type) {
			break;
		case JTAG_SCAN:
			jtag_tcp_end_state(cmd->cmd.scan->end_state);
			scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
			type = jtag_scan_type(cmd->cmd.scan);
			if (jtag_tcp_scan_rsp(cmd->cmd.scan->ir_scan, type, buffer, scan_size) != ERROR_OK)
				retval = ERROR_JTAG_QUEUE_FAILED;
			if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
				retval = ERROR_JTAG_QUEUE_FAILED;
			if (buffer)
				free(buffer);
			break;
		default:
			break;
		}
	}
	if(retval != 0)
		LOG_ERROR("jtag_tcp queue error\n");

	{
		uint8_t rxBuffer;
		if(read(clientSocket,&rxBuffer,1) != 1)
			return ERROR_FAIL;
	}
	return retval;
}


int jtag_tcp_streset(int srst, int trst){
	return ERROR_OK;
}


static const struct command_registration jtag_tcp_command_handlers[] = {
	{
		.name = "jtag_tcp",
		.usage = "",
		.mode = COMMAND_ANY,
		.help = "jtag_tcp interface driver commands",
		.chain = hello_command_handlers,
	},
	COMMAND_REGISTRATION_DONE,
};


/* The jtag_tcp driver is used to easily check the code path
 * where the target is unresponsive.
 */
static struct jtag_interface jtag_tcp_interface = {
        .supported = DEBUG_CAP_TMS_SEQ,
        .execute_queue = &jtag_tcp_execute_queue
};
struct adapter_driver jtag_tcp_adapter_driver = {
		.name = "jtag_tcp",

		.commands = jtag_tcp_command_handlers,
		.transports = jtag_only,
		
		.reset = jtag_tcp_streset,

		.speed = &jtag_tcp_speed,
		.khz = &jtag_tcp_khz,
		.speed_div = &jtag_tcp_speed_div,

		.init = &jtag_tcp_init,
		.quit = &jtag_tcp_quit,

	    .jtag_ops = &jtag_tcp_interface,
	    .swd_ops = NULL,
	};
