/* Copyright 2018 Paul Stoffregen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <Arduino.h>
#include "Ethernet.h"
#include "utility/w5100.h"

#if ARDUINO >= 156 && !defined(ARDUINO_ARCH_PIC32)
extern void yield(void);
#else
#define yield()
#endif

// TODO: randomize this when not using DHCP, but how?
static uint16_t local_port = 49152;  // 49152 to 65535

typedef struct {
	uint16_t RX_RSR; // Number of bytes received
	uint16_t RX_RD;  // Address to read
	uint16_t TX_FSR; // Free space ready for transmit
	uint8_t  RX_inc; // how much have we advanced RX_RD
} socketstate_t;

static socketstate_t state[MAX_SOCK_NUM];



/*****************************************/
/*          Socket management            */
/*****************************************/


void EthernetClass::socketPortRand(uint16_t n)
{
	n &= 0x3FFF;
	local_port ^= n;
	//Serial.printf("socketPortRand %d, srcport=%d\n", n, local_port);
}

uint8_t EthernetClass::socketBegin(uint8_t protocol, uint16_t port)
{
	uint8_t s, status[MAX_SOCK_NUM], chip, maxindex=MAX_SOCK_NUM;

	// first check hardware compatibility
	chip = _w5100.getChip();
	if (!chip) return MAX_SOCK_NUM; // immediate error if no hardware detected
#if MAX_SOCK_NUM > 4
	if (chip == 51) maxindex = 4; // W5100 chip never supports more than 4 sockets
#endif
	//Serial.printf("W5000socket begin, protocol=%d, port=%d\n", protocol, port);
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	// look at all the hardware sockets, use any that are closed (unused)
	for (s=0; s < maxindex; s++) {
		status[s] = _w5100.readSnSR(s);
		if (status[s] == SnSR::CLOSED) goto makesocket;
	}
	//Serial.printf("W5000socket step2\n");
	// as a last resort, forcibly close any already closing
	for (s=0; s < maxindex; s++) {
		uint8_t stat = status[s];
		if (stat == SnSR::LAST_ACK) goto closemakesocket;
		if (stat == SnSR::TIME_WAIT) goto closemakesocket;
		if (stat == SnSR::FIN_WAIT) goto closemakesocket;
		if (stat == SnSR::CLOSING) goto closemakesocket;
	}
#if 0
	Serial.printf("W5000socket step3\n");
	// next, use any that are effectively closed
	for (s=0; s < MAX_SOCK_NUM; s++) {
		uint8_t stat = status[s];
		// TODO: this also needs to check if no more data
		if (stat == SnSR::CLOSE_WAIT) goto closemakesocket;
	}
#endif
	_spibus.endTransaction();
	return MAX_SOCK_NUM; // all sockets are in use
closemakesocket:
	//Serial.printf("W5000socket close\n");
	_w5100.execCmdSn(s, Sock_CLOSE);
makesocket:
	//Serial.printf("W5000socket %d\n", s);
	EthernetServer::server_port[s] = 0;
	delayMicroseconds(250); // TODO: is this needed??
	_w5100.writeSnMR(s, protocol);
	_w5100.writeSnIR(s, 0xFF);
	if (port > 0) {
		_w5100.writeSnPORT(s, port);
	} else {
		// if don't set the source port, set local_port number.
		if (++local_port < 49152) local_port = 49152;
		_w5100.writeSnPORT(s, local_port);
	}
	_w5100.execCmdSn(s, Sock_OPEN);
	state[s].RX_RSR = 0;
	state[s].RX_RD  = _w5100.readSnRX_RD(s); // always zero?
	state[s].RX_inc = 0;
	state[s].TX_FSR = 0;
	//Serial.printf("W5000socket prot=%d, RX_RD=%d\n", _w5100.readSnMR(s), state[s].RX_RD);
	_spibus.endTransaction();
	return s;
}

// multicast version to set fields before open  thd
uint8_t EthernetClass::socketBeginMulticast(uint8_t protocol, IPAddress ip, uint16_t port)
{
	uint8_t s, status[MAX_SOCK_NUM], chip, maxindex=MAX_SOCK_NUM;

	// first check hardware compatibility
	chip = _w5100.getChip();
	if (!chip) return MAX_SOCK_NUM; // immediate error if no hardware detected
#if MAX_SOCK_NUM > 4
	if (chip == 51) maxindex = 4; // W5100 chip never supports more than 4 sockets
#endif
	//Serial.printf("W5000socket begin, protocol=%d, port=%d\n", protocol, port);
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	// look at all the hardware sockets, use any that are closed (unused)
	for (s=0; s < maxindex; s++) {
		status[s] = _w5100.readSnSR(s);
		if (status[s] == SnSR::CLOSED) goto makesocket;
	}
	//Serial.printf("W5000socket step2\n");
	// as a last resort, forcibly close any already closing
	for (s=0; s < maxindex; s++) {
		uint8_t stat = status[s];
		if (stat == SnSR::LAST_ACK) goto closemakesocket;
		if (stat == SnSR::TIME_WAIT) goto closemakesocket;
		if (stat == SnSR::FIN_WAIT) goto closemakesocket;
		if (stat == SnSR::CLOSING) goto closemakesocket;
	}
#if 0
	Serial.printf("W5000socket step3\n");
	// next, use any that are effectively closed
	for (s=0; s < MAX_SOCK_NUM; s++) {
		uint8_t stat = status[s];
		// TODO: this also needs to check if no more data
		if (stat == SnSR::CLOSE_WAIT) goto closemakesocket;
	}
#endif
	_spibus.endTransaction();
	return MAX_SOCK_NUM; // all sockets are in use
closemakesocket:
	//Serial.printf("W5000socket close\n");
	_w5100.execCmdSn(s, Sock_CLOSE);
makesocket:
	//Serial.printf("W5000socket %d\n", s);
	EthernetServer::server_port[s] = 0;
	delayMicroseconds(250); // TODO: is this needed??
	_w5100.writeSnMR(s, protocol);
	_w5100.writeSnIR(s, 0xFF);
	if (port > 0) {
		_w5100.writeSnPORT(s, port);
	} else {
		// if don't set the source port, set local_port number.
		if (++local_port < 49152) local_port = 49152;
		_w5100.writeSnPORT(s, local_port);
	}
	// Calculate MAC address from Multicast IP Address
    	byte mac[] = {  0x01, 0x00, 0x5E, 0x00, 0x00, 0x00 };
    	mac[3] = ip[1] & 0x7F;
    	mac[4] = ip[2];
    	mac[5] = ip[3];
    	_w5100.writeSnDIPR(s, ip.raw_address());   //239.255.0.1
    	_w5100.writeSnDPORT(s, port);
    	_w5100.writeSnDHAR(s, mac);
	_w5100.execCmdSn(s, Sock_OPEN);
	state[s].RX_RSR = 0;
	state[s].RX_RD  = _w5100.readSnRX_RD(s); // always zero?
	state[s].RX_inc = 0;
	state[s].TX_FSR = 0;
	//Serial.printf("W5000socket prot=%d, RX_RD=%d\n", _w5100.readSnMR(s), state[s].RX_RD);
	_spibus.endTransaction();
	return s;
}
// Return the socket's status
//
uint8_t EthernetClass::socketStatus(uint8_t s)
{
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	uint8_t status = _w5100.readSnSR(s);
	_spibus.endTransaction();
	return status;
}

// Immediately close.  If a TCP connection is established, the
// remote host is left unaware we closed.
//
void EthernetClass::socketClose(uint8_t s)
{
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	_w5100.execCmdSn(s, Sock_CLOSE);
	_spibus.endTransaction();
}


// Place the socket in listening (server) mode
//
uint8_t EthernetClass::socketListen(uint8_t s)
{
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	if (_w5100.readSnSR(s) != SnSR::INIT) {
		_spibus.endTransaction();
		return 0;
	}
	_w5100.execCmdSn(s, Sock_LISTEN);
	_spibus.endTransaction();
	return 1;
}


// establish a TCP connection in Active (client) mode.
//
void EthernetClass::socketConnect(uint8_t s, uint8_t * addr, uint16_t port)
{
	// set destination IP
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	_w5100.writeSnDIPR(s, addr);
	_w5100.writeSnDPORT(s, port);
	_w5100.execCmdSn(s, Sock_CONNECT);
	_spibus.endTransaction();
}



// Gracefully disconnect a TCP connection.
//
void EthernetClass::socketDisconnect(uint8_t s)
{
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	_w5100.execCmdSn(s, Sock_DISCON);
	_spibus.endTransaction();
}



/*****************************************/
/*    Socket Data Receive Functions      */
/*****************************************/


uint16_t EthernetClass::getSnRX_RSR(uint8_t s)
{
#if 1
        uint16_t val, prev;

        prev = _w5100.readSnRX_RSR(s);
        while (1) {
                val = _w5100.readSnRX_RSR(s);
                if (val == prev) {
			return val;
		}
                prev = val;
        }
#else
	uint16_t val = _w5100.readSnRX_RSR(s);
	return val;
#endif
}

void EthernetClass::read_data(uint8_t s, uint16_t src, uint8_t *dst, uint16_t len)
{
	uint16_t size;
	uint16_t src_mask;
	uint16_t src_ptr;

	//Serial.printf("read_data, len=%d, at:%d\n", len, src);
	src_mask = (uint16_t)src & _w5100.SMASK;
	src_ptr = _w5100.RBASE(s) + src_mask;

	if (_w5100.hasOffsetAddressMapping() || src_mask + len <= _w5100.SSIZE) {
		_w5100.read(src_ptr, dst, len);
	} else {
		size = _w5100.SSIZE - src_mask;
		_w5100.read(src_ptr, dst, size);
		dst += size;
		_w5100.read(_w5100.RBASE(s), dst, len - size);
	}
}

// Receive data.  Returns size, or -1 for no data, or 0 if connection closed
//
int EthernetClass::socketRecv(uint8_t s, uint8_t *buf, int16_t len)
{
	// Check how much data is available
	int ret = state[s].RX_RSR;
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	if (ret < len) {
		uint16_t rsr = getSnRX_RSR(s);
		ret = rsr - state[s].RX_inc;
		state[s].RX_RSR = ret;
		//Serial.printf("Sock_RECV, RX_RSR=%d, RX_inc=%d\n", ret, state[s].RX_inc);
	}
	if (ret == 0) {
		// No data available.
		uint8_t status = _w5100.readSnSR(s);
		if ( status == SnSR::LISTEN || status == SnSR::CLOSED ||
		  status == SnSR::CLOSE_WAIT ) {
			// The remote end has closed its side of the connection,
			// so this is the eof state
			ret = 0;
		} else {
			// The connection is still up, but there's no data waiting to be read
			ret = -1;
		}
	} else {
		if (ret > len) ret = len; // more data available than buffer length
		uint16_t ptr = state[s].RX_RD;
		if (buf) read_data(s, ptr, buf, ret);
		ptr += ret;
		state[s].RX_RD = ptr;
		state[s].RX_RSR -= ret;
		uint16_t inc = state[s].RX_inc + ret;
		if (inc >= 250 || state[s].RX_RSR == 0) {
			state[s].RX_inc = 0;
			_w5100.writeSnRX_RD(s, ptr);
			_w5100.execCmdSn(s, Sock_RECV);
			//Serial.printf("Sock_RECV cmd, RX_RD=%d, RX_RSR=%d\n",
			//  state[s].RX_RD, state[s].RX_RSR);
		} else {
			state[s].RX_inc = inc;
		}
	}
	_spibus.endTransaction();
	//Serial.printf("socketRecv, ret=%d\n", ret);
	return ret;
}

uint16_t EthernetClass::socketRecvAvailable(uint8_t s)
{
	uint16_t ret = state[s].RX_RSR;
	if (ret == 0) {
		_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
		uint16_t rsr = getSnRX_RSR(s);
		_spibus.endTransaction();
		ret = rsr - state[s].RX_inc;
		state[s].RX_RSR = ret;
		//Serial.printf("sockRecvAvailable s=%d, RX_RSR=%d\n", s, ret);
	}
	return ret;
}

// get the first byte in the receive queue (no checking)
//
uint8_t EthernetClass::socketPeek(uint8_t s)
{
	uint8_t b;
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	uint16_t ptr = state[s].RX_RD;
	_w5100.read((ptr & _w5100.SMASK) + _w5100.RBASE(s), &b, 1);
	_spibus.endTransaction();
	return b;
}



/*****************************************/
/*    Socket Data Transmit Functions     */
/*****************************************/

uint16_t EthernetClass::getSnTX_FSR(uint8_t s)
{
        uint16_t val, prev;

        prev = _w5100.readSnTX_FSR(s);
        while (1) {
                val = _w5100.readSnTX_FSR(s);
                if (val == prev) {
			state[s].TX_FSR = val;
			return val;
		}
                prev = val;
        }
}


void EthernetClass::write_data(uint8_t s, uint16_t data_offset, const uint8_t *data, uint16_t len)
{
	uint16_t ptr = _w5100.readSnTX_WR(s);
	ptr += data_offset;
	uint16_t offset = ptr & _w5100.SMASK;
	uint16_t dstAddr = offset + _w5100.SBASE(s);

	if (_w5100.hasOffsetAddressMapping() || offset + len <= _w5100.SSIZE) {
		_w5100.write(dstAddr, data, len);
	} else {
		// Wrap around circular buffer
		uint16_t size = _w5100.SSIZE - offset;
		_w5100.write(dstAddr, data, size);
		_w5100.write(_w5100.SBASE(s), data + size, len - size);
	}
	ptr += len;
	_w5100.writeSnTX_WR(s, ptr);
}


/**
 * @brief	This function used to send the data in TCP mode
 * @return	1 for success else 0.
 */
uint16_t EthernetClass::socketSend(uint8_t s, const uint8_t * buf, uint16_t len)
{
	uint8_t status=0;
	uint16_t ret=0;
	uint16_t freesize=0;

	if (len > _w5100.SSIZE) {
		ret = _w5100.SSIZE; // check size not to exceed MAX size.
	} else {
		ret = len;
	}

	// if freebuf is available, start.
	do {
		_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
		freesize = getSnTX_FSR(s);
		status = _w5100.readSnSR(s);
		_spibus.endTransaction();
		if ((status != SnSR::ESTABLISHED) && (status != SnSR::CLOSE_WAIT)) {
			ret = 0;
			break;
		}
		yield();
	} while (freesize < ret);

	// copy data
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	write_data(s, 0, (uint8_t *)buf, ret);
	_w5100.execCmdSn(s, Sock_SEND);

	/* +2008.01 bj */
	while ( (_w5100.readSnIR(s) & SnIR::SEND_OK) != SnIR::SEND_OK ) {
		/* m2008.01 [bj] : reduce code */
		if ( _w5100.readSnSR(s) == SnSR::CLOSED ) {
			_spibus.endTransaction();
			return 0;
		}
		_spibus.endTransaction();
		yield();
		_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	}
	/* +2008.01 bj */
	_w5100.writeSnIR(s, SnIR::SEND_OK);
	_spibus.endTransaction();
	return ret;
}

uint16_t EthernetClass::socketSendAvailable(uint8_t s)
{
	uint8_t status=0;
	uint16_t freesize=0;
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	freesize = getSnTX_FSR(s);
	status = _w5100.readSnSR(s);
	_spibus.endTransaction();
	if ((status == SnSR::ESTABLISHED) || (status == SnSR::CLOSE_WAIT)) {
		return freesize;
	}
	return 0;
}

uint16_t EthernetClass::socketBufferData(uint8_t s, uint16_t offset, const uint8_t* buf, uint16_t len)
{
	//Serial.printf("  bufferData, offset=%d, len=%d\n", offset, len);
	uint16_t ret =0;
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	uint16_t txfree = getSnTX_FSR(s);
	if (len > txfree) {
		ret = txfree; // check size not to exceed MAX size.
	} else {
		ret = len;
	}
	write_data(s, offset, buf, ret);
	_spibus.endTransaction();
	return ret;
}

bool EthernetClass::socketStartUDP(uint8_t s, uint8_t* addr, uint16_t port)
{
	if ( ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
	  ((port == 0x00)) ) {
		return false;
	}
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	_w5100.writeSnDIPR(s, addr);
	_w5100.writeSnDPORT(s, port);
	_spibus.endTransaction();
	return true;
}

bool EthernetClass::socketSendUDP(uint8_t s)
{
	_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	_w5100.execCmdSn(s, Sock_SEND);

	/* +2008.01 bj */
	while ( (_w5100.readSnIR(s) & SnIR::SEND_OK) != SnIR::SEND_OK ) {
		if (_w5100.readSnIR(s) & SnIR::TIMEOUT) {
			/* +2008.01 [bj]: clear interrupt */
			_w5100.writeSnIR(s, (SnIR::SEND_OK|SnIR::TIMEOUT));
			_spibus.endTransaction();
			//Serial.printf("sendUDP timeout\n");
			return false;
		}
		_spibus.endTransaction();
		yield();
		_spibus.beginTransaction(SPI_ETHERNET_SETTINGS);
	}

	/* +2008.01 bj */
	_w5100.writeSnIR(s, SnIR::SEND_OK);
	_spibus.endTransaction();

	//Serial.printf("sendUDP ok\n");
	/* Sent ok */
	return true;
}
/* vim: set noet sw=8: */
