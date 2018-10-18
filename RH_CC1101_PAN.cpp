// RH_CC1101_PAN.cpp
//
// Driver for Texas Instruments CC1101_PANL transceiver.
//
// Copyright (C) 2016 Mike McCauley
// $Id: RH_CC1101_PAN.cpp,v 1.9 2018/01/06 23:50:45 mikem Exp $

#include <RH_CC1101_PAN.h>

// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of RH_CC1101_PAN, allowing you to have
// 2 or more LORAs per Arduino
RH_CC1101_PAN* RH_CC1101_PAN::_deviceForInterrupt[RH_CC1101_PAN_NUM_INTERRUPTS] = {0, 0, 0};
// Index into _deviceForInterrupt for next device
uint8_t RH_CC1101_PAN::_interruptCount = 0;

// We need 2 tables of modem configuration registers, since some values change depending on the Xtal frequency
// These are indexed by the values of ModemConfigChoice
// Canned modem configurations generated with the TI SmartRF Studio v7 version 2.3.0 on boodgie
// based on the sample 'Typical settings'
// Stored in flash (program) memory to save SRAM
// For 26MHz crystals
PROGMEM static const RH_CC1101_PAN::ModemConfig MODEM_CONFIG_TABLE_26MHZ[] =
{
	// 0B    0C    10    11    12    15    19    1A    1B    1C    1D    21    22    23    24    25    26    2C    2D    2E
	{0x06, 0x00, 0xf5, 0x83, 0x13, 0x15, 0x16, 0x6c, 0x03, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb1_2Fd5_2
	{0x06, 0x00, 0xf6, 0x83, 0x13, 0x15, 0x16, 0x6c, 0x03, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb2_4Fd5_2
	{0x06, 0x00, 0xc7, 0x83, 0x13, 0x40, 0x16, 0x6c, 0x43, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb4_8Fd25_4
	{0x06, 0x00, 0xc8, 0x93, 0x13, 0x34, 0x16, 0x6c, 0x43, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb10Fd19
	{0x06, 0x00, 0xca, 0x83, 0x13, 0x35, 0x16, 0x6c, 0x43, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb38_4Fd20
	{0x08, 0x00, 0x7b, 0x83, 0x13, 0x42, 0x1d, 0x1c, 0xc7, 0x00, 0xb2, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb76_8Fd32
	{0x08, 0x00, 0x5b, 0xf8, 0x13, 0x47, 0x1d, 0x1c, 0xc7, 0x00, 0xb2, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x81, 0x31, 0x09}, // GFSK_Rb100Fd47
	{0x0c, 0x00, 0x2d, 0x3b, 0x13, 0x62, 0x1d, 0x1c, 0xc7, 0x00, 0xb0, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x88, 0x31, 0x09}, // GFSK_Rb250Fd127
};

// For 27MHz crystals
PROGMEM static const RH_CC1101_PAN::ModemConfig MODEM_CONFIG_TABLE_27MHZ[] =
{
	// 0B    0C    10    11    12    15    19    1A    1B    1C    1D    21    22    23    24    25    26    2C    2D    2E
	{0x06, 0x00, 0xf5, 0x75, 0x13, 0x14, 0x16, 0x6c, 0x03, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb1_2Fd5_2
	{0x06, 0x00, 0xf6, 0x75, 0x13, 0x14, 0x16, 0x6c, 0x03, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb2_4Fd5_2
	{0x06, 0x00, 0xc7, 0x75, 0x13, 0x37, 0x16, 0x6c, 0x43, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb4_8Fd25_4
	{0x06, 0x00, 0xc8, 0x84, 0x13, 0x33, 0x16, 0x6c, 0x43, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb10Fd19
	{0x06, 0x00, 0xca, 0x75, 0x13, 0x34, 0x16, 0x6c, 0x43, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb38_4Fd20
	{0x08, 0x00, 0x7b, 0x75, 0x13, 0x42, 0x1d, 0x1c, 0xc7, 0x00, 0xb2, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb76_8Fd32
	{0x08, 0x00, 0x5b, 0xf8, 0x13, 0x47, 0x1d, 0x1c, 0xc7, 0x00, 0xb2, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x81, 0x31, 0x09}, // GFSK_Rb100Fd47
	{0x0c, 0x00, 0x2d, 0x2f, 0x13, 0x62, 0x1d, 0x1c, 0xc7, 0x00, 0xb0, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x88, 0x31, 0x09}, // GFSK_Rb250Fd127
};

// These power outputs are based on the suggested optimum values for
// multilayer inductors in the 915MHz frequency band. Per table 5-15.
// Yes these are not linear.
// Caution: this table is indexed by the values of enum TransmitPower
// Do not change one without changing the other.
// If you do not like these values, use setPaTable() directly.
PROGMEM static const uint8_t paPowerValues[] =
{
	0x03, // -30dBm
	0x0e, // -20dBm
	0x1e, // -15dBm
	0x27, // -10dBm
	0x8e, // 0dBm
	0xcd, // 5dBm
	0xc7, // 7dBm
	0xc0, // 10dBm
};

RH_CC1101_PAN::RH_CC1101_PAN(uint8_t slaveSelectPin, uint8_t interruptPin, bool is27MHz, RHGenericSPI& spi)
:
RHNRFSPIDriver(slaveSelectPin, spi),
_rxBufValid(false),
_is27MHz(is27MHz)
{
	_interruptPin		= interruptPin;
	_myInterruptIndex	= 0xff; // Not allocated yet
}

bool RH_CC1101_PAN::init()
{
	if (!RHNRFSPIDriver::init()) return false;

	// Determine the interrupt number that corresponds to the interruptPin
	int interruptNumber = digitalPinToInterrupt(_interruptPin);
	
	if (interruptNumber == NOT_AN_INTERRUPT) return false;
	
	#ifdef RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
	interruptNumber = _interruptPin;
	#endif

	// Tell the low level SPI interface we will use SPI within this interrupt
	spiUsingInterrupt(interruptNumber);

	// Reset the chip
	// Strobe the reset
	uint8_t val = spiCommand(RH_CC1101_PAN_STROBE_30_SRES); // Reset

	delay(100);

	val = spiCommand(RH_CC1101_PAN_STROBE_36_SIDLE);	// IDLE

	if (val != 0x0f) return false;						// No chip there or reset failed.

	spiWriteRegister(RH_CC1101_PAN_REG_02_IOCFG0,		RH_CC1101_PAN_GDO_CFG_CRC_OK_AUTORESET);	// GDO0 to detect SYNC
	spiWriteRegister(RH_CC1101_PAN_REG_00_IOCFG2,		RH_CC1101_PAN_GDO_CFG_CCA);				// GDO2 to detect CCA
	spiWriteRegister(RH_CC1101_PAN_REG_06_PKTLEN,		RH_CC1101_PAN_MAX_PAYLOAD_LEN);			// max packet length
	spiWriteRegister(RH_CC1101_PAN_REG_07_PKTCTRL1,		RH_CC1101_PAN_CRC_AUTOFLUSH);				// Don't append CRC & RSSI + Perform No Address check
	spiWriteRegister(RH_CC1101_PAN_REG_08_PKTCTRL0,		RH_CC1101_PAN_PKT_FORMAT_NORMAL | RH_CC1101_PAN_CRC_EN | RH_CC1101_PAN_LENGTH_CONFIG_VARIABLE);
	spiWriteRegister(RH_CC1101_PAN_REG_13_MDMCFG1,		RH_CC1101_PAN_NUM_PREAMBLE_4);				// 4 preamble bytes, chan spacing not used
	spiWriteRegister(RH_CC1101_PAN_REG_17_MCSM1,		RH_CC1101_PAN_CCA_MODE_RSSI_PACKET | RH_CC1101_PAN_RXOFF_MODE_RX | RH_CC1101_PAN_TXOFF_MODE_IDLE);
	spiWriteRegister(RH_CC1101_PAN_REG_18_MCSM0,		RH_CC1101_PAN_FS_AUTOCAL_FROM_IDLE | RH_CC1101_PAN_PO_TIMEOUT_64);
	spiWriteRegister(RH_CC1101_PAN_REG_20_WORCTRL,		0xfb);										// from smartrf
	spiWriteRegister(RH_CC1101_PAN_REG_29_FSTEST,		0x59);										// from smartrf
	spiWriteRegister(RH_CC1101_PAN_REG_2A_PTEST,		0x7f);										// from smartrf
	spiWriteRegister(RH_CC1101_PAN_REG_2B_AGCTEST,		0x3f);										// from smartrf

	// Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
	// ARM M4 requires the below. else pin interrupt doesn't work properly.
	// On all other platforms, its innocuous, belt and braces
	pinMode(_interruptPin, INPUT);

	// Set up interrupt handler
	// Since there are a limited number of interrupt glue functions isr*() available,
	// we can only support a limited number of devices simultaneously
	// ON some devices, notably most Arduinos, the interrupt pin passed in is actuallt the
	// interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
	// yourself based on knowledge of what Arduino board you are running on.
	if (_myInterruptIndex == 0xff)
	{
		// First run, no interrupt allocated yet
		if (_interruptCount <= RH_CC1101_PAN_NUM_INTERRUPTS)
		{
			_myInterruptIndex = _interruptCount++;
		}
		else
		{
			// Too many devices, not enough interrupt vectors
			return false; 
		}
	}

	_deviceForInterrupt[_myInterruptIndex] = this;

	// Set some reasonable default values
	uint8_t syncWords[] = { 0x27, 0x18 };
	setSyncWords(syncWords, sizeof(syncWords));
	setTxPower(TransmitPowerM30dBm);
	setFrequency(868.0);
	// setModemConfig(GFSK_Rb4_8Fd25_4);		// < GFSK, Data Rate:   4.8kBaud,	Dev:  25.4kHz,	RX BW 100kHz
	// setModemConfig(GFSK_Rb1_2Fd5_2);			// < GFSK, Data Rate:   4.8kBaud,	Dev:  25.4kHz,	RX BW 100kHz
	setModemConfig(GFSK_Rb38_4Fd20);
	
	if (_myInterruptIndex == 0)
	{
		attachInterrupt(INT0, isr0, RISING);
	}
	else if (_myInterruptIndex == 1)	attachInterrupt(interruptNumber, isr1, RISING);
	else if (_myInterruptIndex == 2)	attachInterrupt(interruptNumber, isr2, RISING);
	else								return false; // Too many devices, not enough interrupt vectors

	setCADTimeout(10000);
	// Move to Idle State in preparation of the Calibration
	setModeIdle();
	// Move to Rx State to trigger Calibration and be ready to receive Packets
	setModeRx();

	return true;
}

void RH_CC1101_PAN::setIs27MHz(bool is27MHz)
{
	_is27MHz = is27MHz;
}

// C++ level interrupt handler for this instance
// We use this to get RxDone and TxDone interrupts
void RH_CC1101_PAN::RxInterruptHandler()
{
	byte	PacketBytes;
	byte	RxPacketCRC;
	byte	RxPacketLQI;
	
	if (_mode == RHModeRx)
	{
		PacketBytes = spiBurstReadRegister(RH_CC1101_PAN_REG_3B_RXBYTES);
		
		// Check if some bytes are available & if there is any FIFO overflow
		if ((PacketBytes & 0x7F) && !(PacketBytes & 0x80))
		{
			_bufLen		= spiReadRegister(RH_CC1101_PAN_REG_3F_FIFO);

			RxPacketLQI = spiBurstReadRegister(RH_CC1101_PAN_REG_33_CRC_REG);
			RxPacketCRC = RxPacketLQI & 0x80;
			RxPacketLQI = RxPacketLQI & 0x7F;
			// To Do : Exploit LQI

			// Check the number of bytes in Rx FIFO, FIFO overflow and Packet CRC
			if ((_bufLen < 4) || (_bufLen > RH_CC1101_PAN_FIFO_SIZE))
			{
				// Something wrong -> Clear Rx FIFO
				spiCommand(RH_CC1101_PAN_STROBE_36_SIDLE);
				spiCommand(RH_CC1101_PAN_STROBE_3A_SFRX);
				spiCommand(RH_CC1101_PAN_STROBE_34_SRX);

				clearRxBuf();
				return;
			}
			else
			{
				spiBurstRead(RH_CC1101_PAN_REG_3F_FIFO | RH_CC1101_PAN_SPI_BURST_MASK | RH_CC1101_PAN_SPI_READ_MASK, _buf, _bufLen);

				uint8_t raw_rssi = spiBurstReadRegister(RH_CC1101_PAN_REG_34_RSSI);
				_lastRssi = (raw_rssi >= 128) ? (((int16_t)raw_rssi-256)/2)-74:((int16_t)raw_rssi/2)-74;
				// TODO : Add LQI to public variable/properties

				validateRxBuf();

				if (_rxBufValid) setModeIdle();

				return;
			}
		}
	}
}

// These are low level functions that call the interrupt handler for the correct
// instance of RH_CC1101_PAN.
// 3 interrupts allows us to have 3 different devices
void RH_CC1101_PAN::isr0()
{
	if (_deviceForInterrupt[0]) _deviceForInterrupt[0]->RxInterruptHandler();
}

void RH_CC1101_PAN::isr1()
{
	if (_deviceForInterrupt[1]) _deviceForInterrupt[1]->RxInterruptHandler();
}

void RH_CC1101_PAN::isr2()
{
	if (_deviceForInterrupt[2]) _deviceForInterrupt[2]->RxInterruptHandler();
}

uint8_t RH_CC1101_PAN::spiReadRegister(uint8_t reg)
{
	return spiRead((reg & 0x3f) | RH_CC1101_PAN_SPI_READ_MASK);
}

uint8_t RH_CC1101_PAN::spiBurstReadRegister(uint8_t reg)
{
	return spiRead((reg & 0x3f) | RH_CC1101_PAN_SPI_READ_MASK | RH_CC1101_PAN_SPI_BURST_MASK);
}

uint8_t RH_CC1101_PAN::spiWriteRegister(uint8_t reg, uint8_t val)
{
	return spiWrite((reg & 0x3f), val);
}

uint8_t  RH_CC1101_PAN::spiBurstWriteRegister(uint8_t reg, const uint8_t* src, uint8_t len)
{
	return spiBurstWrite((reg & 0x3f) | RH_CC1101_PAN_SPI_BURST_MASK, src, len);
}

bool RH_CC1101_PAN::printRegisters()
{
	//	#ifdef RH_HAVE_SERIAL
	uint8_t i;
	char _buffer[32];

	for (i = 0; i <= 0x2f; i++)
	{
		sprintf(_buffer, "%2X : %2X ", i, spiReadRegister(i));
		Serial.print  (_buffer);
		Serial.println(spiReadRegister(i), BIN);
	}

	// Burst registers
	for (i = 0x30; i <= 0x3e; i++)
	{
		sprintf(_buffer, "%2X : %2X ", i, spiBurstReadRegister(i));
		Serial.print  (_buffer);
		Serial.println(spiBurstReadRegister(i), BIN);
	}
	//	#endif
	return true;
}

// Check whether the latest received message is complete and uncorrupted
void RH_CC1101_PAN::validateRxBuf()
{
	// Extract the 4 headers
	_rxHeaderTo    = _buf[0];
	_rxHeaderFrom  = _buf[1];
	_rxHeaderId    = _buf[2];
	_rxHeaderFlags = _buf[3];

	// TODO : Check if this is still needed as address check is performed by the CC1101
	if (_promiscuous || (_rxHeaderTo == _thisAddress) || (_rxHeaderTo == RH_BROADCAST_ADDRESS))
	{
		_rxGood++;
		_rxBufValid = true;
	}
	else
	{
		Serial.println(F("Packet not for this node"));
		_rxBufValid = false;
	}
}

bool RH_CC1101_PAN::available()
{
	if (_rxBufValid) // Will be set by the interrupt handler when a good message is received
	{
		return true;
	}
	
	setModeRx(); // Make sure we are receiving

	return false; // Nothing yet
}

void RH_CC1101_PAN::clearRxBuf()
{
	ATOMIC_BLOCK_START;
	_rxBufValid = false;
	_bufLen = 0;
	ATOMIC_BLOCK_END;
	// TO CHECK
}

bool RH_CC1101_PAN::recv(uint8_t* buf, uint8_t* len)
{
	if (!available()) return false;

	if (buf && len)
	{
		ATOMIC_BLOCK_START;
		// Skip the 4 headers that are at the beginning of the rxBuf
		if (*len > _bufLen - RH_CC1101_PAN_HEADER_LEN) *len = _bufLen - RH_CC1101_PAN_HEADER_LEN;
		memcpy(buf, _buf + RH_CC1101_PAN_HEADER_LEN, *len);
		ATOMIC_BLOCK_END;
	}

	clearRxBuf(); // This message accepted and cleared

	return true;
}

bool RH_CC1101_PAN::send(const uint8_t* data, uint8_t len)
{
	// Check length of message to send
	if (len > RH_CC1101_PAN_MAX_MESSAGE_LEN)
	{
		return false;
	}

	// Enter Idle State and clear Tx FIFO
	setModeIdle();

	// Flush Tx FIFO (just in case)
	spiCommand(RH_CC1101_PAN_STROBE_3B_SFTX);
	
	// Setup Packet content
	spiWriteRegister     (RH_CC1101_PAN_REG_3F_FIFO, len + RH_CC1101_PAN_HEADER_LEN);
	spiWriteRegister     (RH_CC1101_PAN_REG_3F_FIFO, _txHeaderTo);
	spiWriteRegister     (RH_CC1101_PAN_REG_3F_FIFO, _txHeaderFrom);
	spiWriteRegister     (RH_CC1101_PAN_REG_3F_FIFO, _txHeaderId);
	spiWriteRegister     (RH_CC1101_PAN_REG_3F_FIFO, _txHeaderFlags);
	spiBurstWriteRegister(RH_CC1101_PAN_REG_3F_FIFO, data, len);

	byte counter;

	_mode = RHModeTx;

	for (counter = 0; counter < 10; counter++)
	{
		if (setModeTx()) break;
		delay(10);
	}

	if (counter == 10)
	{
		Serial.println(F("Channel busy"));

		setModeIdle();
		setModeRx();

		return false;
	}

	// Wait Tx FIFO to be empty
	while (spiBurstReadRegister(RH_CC1101_PAN_REG_3A_TXBYTES) > 0) delay(1);

	// Wait for Idle State to be set (
	while ((statusRead() & 0b01110000) != 0b00000000) delay(1);

	setModeRx();

	return true;
}

uint8_t RH_CC1101_PAN::maxMessageLength()
{
	return RH_CC1101_PAN_MAX_MESSAGE_LEN;
}

// Wait until no channel activity detected or timeout
bool RH_CC1101_PAN::waitCAD()
{
	if (!_cad_timeout) return true;

	// Wait for any channel activity to finish or timeout
	// Sophisticated DCF function...
	// DCF : BackoffTime = random() x aSlotTime
	// 100 - 1000 ms
	// 10 sec timeout
	unsigned long t = millis();
	
	while (isChannelActive())
	{
		if (millis() - t > _cad_timeout) return false;
		
		#if (RH_PLATFORM == RH_PLATFORM_STM32) // stdlib on STMF103 gets confused if random is redefined
		delay(_random(1, 10) * 100);
		#else
		delay(random(1, 10) * 100); // Should these values be configurable? Macros?
		#endif
	}

	return true;
}

// subclasses are expected to override if CAD is available for that radio
bool RH_CC1101_PAN::isChannelActive()
{
	return (spiBurstReadRegister(RH_CC1101_PAN_REG_38_PKTSTATUS) & RH_CC1101_PAN_PKTSTATUS_CCA);
}

void RH_CC1101_PAN::setThisAddress(uint8_t address)
{
	_thisAddress = address;
	setCheckAddress(true);
	spiWriteRegister(RH_CC1101_PAN_REG_09_ADDR, address);
}

void RH_CC1101_PAN::setCheckAddress(bool checkAddress)
{
	if (checkAddress == true)
	{
		spiWriteRegister(RH_CC1101_PAN_REG_07_PKTCTRL1, spiReadRegister(RH_CC1101_PAN_REG_07_PKTCTRL1) | RH_CC1101_PAN_ADDR_CHK);
	}
	else
	{
		spiWriteRegister(RH_CC1101_PAN_REG_07_PKTCTRL1, spiReadRegister(RH_CC1101_PAN_REG_07_PKTCTRL1) & (~RH_CC1101_PAN_ADDR_CHK));
	}
}

bool RH_CC1101_PAN::setModeIdle()
{
	// Move to Idle State and wait until done
	spiCommand(RH_CC1101_PAN_STROBE_36_SIDLE);
	while ((statusRead() & 0b01110000) != 0b00000000) delay(1); // Serial.write('I');

	_mode = RHModeIdle;
	
	return true;
}

bool RH_CC1101_PAN::sleep()
{
	// Move to Idle State
	setModeIdle();
	
	// Move to Powerdown State
	spiCommand(RH_CC1101_PAN_STROBE_39_SPWD);
	
	_mode = RHModeSleep;
	
	return true;
}

bool RH_CC1101_PAN::wakeup()
{
	// Wake up CC1101
	statusRead();
	delay(1);

	return (setModeRx());
}

bool RH_CC1101_PAN::setModeRx()
{
	if (_mode != RHModeRx)
	{
		// Move to Idle State
		setModeIdle();

		// Make sure the mode is updated before next interrupt
		_mode = RHModeRx;

		// Move to Rx State and wait until done
		spiCommand(RH_CC1101_PAN_STROBE_34_SRX);
		// Wait for eventual calibration to be done
		while ((statusRead() & 0b01110000) == 0b01000000) delay(1); //Serial.write('K');
		// Wait for Rx State to be reached
		while ((statusRead() & 0b01110000) != 0b00010000) delay(1); //Serial.write('Y');
	}
	return true;
}

bool RH_CC1101_PAN::setModeTx()
{
	if ((_mode != RHModeTx) || ((statusRead() & 0b01110000) != 0b001000000))
	{
		if ((statusRead() & 0b01110000) == 0b01110000) spiCommand(RH_CC1101_PAN_STROBE_3B_SFTX); 

		// Make sure the mode is updated before next interrupt
		_mode = RHModeTx;

		// Move to Rx State and wait until done
		spiCommand(RH_CC1101_PAN_STROBE_35_STX);
		// Wait for eventual calibration to be done
		while ((statusRead() & 0b01110000) == 0b01000000) delay(1); //Serial.write('K');
		// Wait for Tx State to be reached
		while ((statusRead() & 0b01110000) != 0b00100000) delay(1); //Serial.write('Z');
	}
	return true;
}

uint8_t RH_CC1101_PAN::statusRead()
{
	return spiCommand(RH_CC1101_PAN_STROBE_3D_SNOP);
}

// Sigh, this chip has no TXDONE type interrupt, so we have to poll
bool RH_CC1101_PAN::waitPacketSent()
{
	// TODO : Use interrupt for updating the Packet Sent Status
	return true;
}

bool RH_CC1101_PAN::setTxPower(TransmitPower power)
{
	uint8_t patable[2];
	
	if (power > sizeof(paPowerValues))
	{
		power = (TransmitPower) sizeof(paPowerValues);
	}

	memcpy_P(&patable[0], (void*)&paPowerValues[power], sizeof(uint8_t));
	patable[1] = 0x00;
	setPaTable(patable, sizeof(patable));

	return true;
}

void RH_CC1101_PAN::setPaTable(uint8_t* patable, uint8_t patablesize)
{
	spiBurstWriteRegister(RH_CC1101_PAN_REG_3E_PATABLE, patable, patablesize);
}

bool RH_CC1101_PAN::setFrequency(float centre)
{
	// From section 5.21: fcarrier = fxosc / 2^16 * FREQ
	uint32_t FREQ;
	float fxosc = _is27MHz ? 27.0 : 26.0;
	FREQ = (uint32_t)(centre * 65536 / fxosc);

	// Some trivial checks
	if (FREQ & 0xff000000) return false;
	
	spiWriteRegister(RH_CC1101_PAN_REG_0D_FREQ2, (FREQ >> 16) & 0xff);
	spiWriteRegister(RH_CC1101_PAN_REG_0E_FREQ1, (FREQ >>  8) & 0xff);
	spiWriteRegister(RH_CC1101_PAN_REG_0F_FREQ0, (FREQ >>  0) & 0xff);

	// Radio is configured to calibrate automatically whenever it enters RX or TX mode
	// so no need to check for PLL lock here
	return true;
}

// Sets registers from a canned modem configuration structure
void RH_CC1101_PAN::setModemRegisters(const ModemConfig* config)
{
	spiWriteRegister(RH_CC1101_PAN_REG_0B_FSCTRL1,  config->reg_0b);
	spiWriteRegister(RH_CC1101_PAN_REG_0C_FSCTRL0,  config->reg_0c);
	spiWriteRegister(RH_CC1101_PAN_REG_10_MDMCFG4,  config->reg_10);
	spiWriteRegister(RH_CC1101_PAN_REG_11_MDMCFG3,  config->reg_11);
	spiWriteRegister(RH_CC1101_PAN_REG_12_MDMCFG2,  config->reg_12);
	spiWriteRegister(RH_CC1101_PAN_REG_15_DEVIATN,  config->reg_15);
	spiWriteRegister(RH_CC1101_PAN_REG_19_FOCCFG,   config->reg_19);
	spiWriteRegister(RH_CC1101_PAN_REG_1A_BSCFG,    config->reg_1a);
	spiWriteRegister(RH_CC1101_PAN_REG_1B_AGCCTRL2, config->reg_1b);
	spiWriteRegister(RH_CC1101_PAN_REG_1C_AGCCTRL1, config->reg_1c);
	spiWriteRegister(RH_CC1101_PAN_REG_1D_AGCCTRL0, config->reg_1d);
	spiWriteRegister(RH_CC1101_PAN_REG_21_FREND1,   config->reg_21);
	spiWriteRegister(RH_CC1101_PAN_REG_22_FREND0,   config->reg_22);
	spiWriteRegister(RH_CC1101_PAN_REG_23_FSCAL3,   config->reg_23);
	spiWriteRegister(RH_CC1101_PAN_REG_24_FSCAL2,   config->reg_24);
	spiWriteRegister(RH_CC1101_PAN_REG_25_FSCAL1,   config->reg_25);
	spiWriteRegister(RH_CC1101_PAN_REG_26_FSCAL0,   config->reg_26);
	spiWriteRegister(RH_CC1101_PAN_REG_2C_TEST2,    config->reg_2c);
	spiWriteRegister(RH_CC1101_PAN_REG_2D_TEST1,    config->reg_2d);
	spiWriteRegister(RH_CC1101_PAN_REG_2E_TEST0,    config->reg_2e);
}

// Set one of the canned Modem configs
// Returns true if its a valid choice
bool RH_CC1101_PAN::setModemConfig(ModemConfigChoice index)
{
	if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE_27MHZ) / sizeof(ModemConfig))) return false;

	const RH_CC1101_PAN::ModemConfig *p = _is27MHz ? MODEM_CONFIG_TABLE_27MHZ : MODEM_CONFIG_TABLE_26MHZ ;
	RH_CC1101_PAN::ModemConfig cfg;
	memcpy_P(&cfg, p + index, sizeof(RH_CC1101_PAN::ModemConfig));
	setModemRegisters(&cfg);

	return true;
}

void RH_CC1101_PAN::setSyncWords(const uint8_t* syncWords, uint8_t len)
{
	if (!syncWords || len != 2)
	return; // Only 2 byte sync words are supported

	spiWriteRegister(RH_CC1101_PAN_REG_04_SYNC1, syncWords[0]);
	spiWriteRegister(RH_CC1101_PAN_REG_05_SYNC0, syncWords[1]);
}
