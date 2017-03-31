#include "ENC28J60.h"
#include "Definitions.h"


void enc28j60_init_tx_buffer0(byte identifierLow, byte identifierHigh, byte messageSize) {

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	execute_write_command(REGISTER_TXB0SIDL, identifierLow, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(REGISTER_TXB0SIDH, identifierHigh, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(REGISTER_TXB0DLC, messageSize, do_csEnc28j60, SPI_INSTRUCTION_WRITE);
}

void enc28j60_init_tx_buffer1(byte identifierLow, byte identifierHigh, byte messageSize) {

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	execute_write_command(REGISTER_TXB1SIDL, identifierLow, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(REGISTER_TXB1SIDH, identifierHigh, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(REGISTER_TXB1DLC, messageSize, do_csEnc28j60, SPI_INSTRUCTION_WRITE);
}

void enc28j60_init_tx_buffer2(byte identifierLow, byte identifierHigh, byte messageSize) {

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	execute_write_command(REGISTER_TXB2SIDL, identifierLow, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(REGISTER_TXB2SIDH, identifierHigh, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(REGISTER_TXB2DLC, messageSize, do_csEnc28j60, SPI_INSTRUCTION_WRITE);
}

void initEnc28J60() {
	// Reset chip to set in operation mode
	enc28j60_execute_reset_command();

	// Configure bit timing
	enc28j60_configureCanBus();

	// Configure interrupts
	enc28j60_configureInterrupts();

	// Set device to normal mode
	enc28j60_switchMode(REGISTER_CANSTAT_NORMAL_MODE, REGISTER_CANCTRL_NORMAL_MODE);
}

void enc28j60_execute_reset_command() {
	// Reset chip to get initial condition and wait for operation mode state bit
	byte returnMessage;

	writeSimpleCommandSpi(SPI_INSTRUCTION_RESET, do_csEnc28j60);

	// Read the register value
	byte actualMode = execute_read_command(REGISTER_CANSTAT, do_csEnc28j60, SPI_INSTRUCTION_READ);
	Serial.print("Try to reset enc28j60");
	while (REGISTER_CANSTAT_CONFIGURATION_MODE != (REGISTER_CANSTAT_CONFIGURATION_MODE & actualMode))
	{
		actualMode = execute_read_command(REGISTER_CANSTAT, do_csEnc28j60, SPI_INSTRUCTION_READ);
		Serial.print(actualMode);
	}

	if (debugMode) Serial.print("enc28j60 reset succesfully and switch do mode ");
	if (debugMode) Serial.println(actualMode);
}

void enc28j60_configureCanBus() {
	// Configure bit timing

	execute_write_command(REGISTER_CNF1, REGISTER_CNF1_VALUE, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	execute_write_command(REGISTER_CNF2, REGISTER_CNF2_VALUE, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	execute_write_command(REGISTER_CNF3, REGISTER_CNF3_VALUE, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	if (debugMode) Serial.println("enc28j60 configure bus succesfully");
}

void enc28j60_configureInterrupts() {
	execute_write_command(REGISTER_CANINTE, REGISTER_CANINTE_VALUE, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	if (debugMode) Serial.println("enc28j60 configure interrupts succesfully");
}

void enc28j60_configureMasksFilters(byte registerAddress, byte registerValue) {

	// Set parameters for rx buffer 0
	execute_write_command(REGISTER_RXB0CTRL, REGISTER_RXB0CTRL_VALUE, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	// Set parameters for rx buffer 1
	execute_write_command(REGISTER_RXB1CTRL, REGISTER_RXB1CTRL_VALUE, do_csEnc28j60, SPI_INSTRUCTION_WRITE);
}

void enc28j60_configureRxBuffer() {

	// Set parameters for rx buffer 0
	execute_write_command(0x60, 0x04, do_csEnc28j60, SPI_INSTRUCTION_WRITE); // as dec 39: Filter 1 is enabled (rollover is active / bukt is set)

	execute_write_command(0x70, 0x00, do_csEnc28j60, SPI_INSTRUCTION_WRITE); // as dec 33: Filter 1 is enabled (if bukt is set)

	if (debugMode) Serial.println("enc28j60 configure rx buffer succesfully");
}

void enc28j60_switchMode(byte modeToCheck, byte modeToSwitch) {
	// Reset chip to get initial condition and wait for operation mode state bit
	byte returnMessage[1];
	byte actualMode;

	// Repeat mode switching for a specific time when the first trial is without success
	for (int i = 0; i < MAX_ERR_CNTR_MDSWCH_ENC28J60; i++)
	{
		execute_write_command(REGISTER_CANCTRL, modeToSwitch, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

		// Read the register value
		actualMode = execute_read_command(REGISTER_CANSTAT, do_csEnc28j60, SPI_INSTRUCTION_READ);
		long elapsedTime = 0;
		long errorTimerValue = millis();
		while ((actualMode != modeToSwitch) && (elapsedTime <= MAX_WAIT_TIME_ENC28J60))
		{
			actualMode = execute_read_command(REGISTER_CANSTAT, do_csEnc28j60, SPI_INSTRUCTION_READ);
			elapsedTime = millis() - errorTimerValue; // Stop time to break loop when mode isnt switching
		}
		if (elapsedTime > MAX_WAIT_TIME_ENC28J60)
		{
			if (debugMode) Serial.println("Abort waiting. Max. waiting time reached.");
			if (i == MAX_ERR_CNTR_MDSWCH_ENC28J60)
			{
				if (debugMode) Serial.println("ERROR MODE SWITCH - STOP ALL OPERATIONS");
				stopAllOperations = true;
			}
		}
		else break;
	}

	if (debugMode) Serial.print("enc28j60 switch to mode ");
	if (debugMode) Serial.print(actualMode);
	if (debugMode) Serial.println(" succesfully");
}

void enc28j60_load_tx_buffer0(byte messageData, int byteNumber, int messageSize) {

	execute_write_command(REGISTER_TXB0Dx[byteNumber], messageData, do_csEnc28j60, SPI_INSTRUCTION_WRITE);
}

void enc28j60_load_tx_buffer1(byte messageData, int byteNumber, int messageSize) {

	execute_write_command(REGISTER_TXB1Dx[byteNumber], messageData, do_csEnc28j60, SPI_INSTRUCTION_WRITE);
}

void enc28j60_load_tx_buffer2(byte messageData, int byteNumber, int messageSize) {

	execute_write_command(REGISTER_TXB2Dx[byteNumber], messageData, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

}

void enc28j60_init_tx_buffer(byte identifierLow, byte identifierHigh, byte messageSize, int regAddr) {

	byte regAddr_sidl;
	byte regAddr_sidh;
	byte regAddr_dlc;

	switch (regAddr)
	{
	case 0:
		regAddr_sidl = REGISTER_TXB0SIDL;
		regAddr_sidh = REGISTER_TXB0SIDH;
		regAddr_dlc = REGISTER_TXB0DLC;
		break;
	case 1:
		regAddr_sidl = REGISTER_TXB1SIDL;
		regAddr_sidh = REGISTER_TXB1SIDH;
		regAddr_dlc = REGISTER_TXB1DLC;
		break;
	case 2:
		regAddr_sidl = REGISTER_TXB2SIDL;
		regAddr_sidh = REGISTER_TXB2SIDH;
		regAddr_dlc = REGISTER_TXB2DLC;
		break;
	default:
		break;
	}

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	execute_write_command(regAddr_sidl, identifierLow, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(regAddr_sidh, identifierHigh, do_csEnc28j60, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(regAddr_dlc, messageSize, do_csEnc28j60, SPI_INSTRUCTION_WRITE);
}

void enc28j60_execute_rts_command(int bufferId)
{
	byte spiMessage[1];

	switch (bufferId)
	{
	case 0:
		spiMessage[0] = SPI_INSTRUCTION_RTS_BUFFER0;
		break;
	case 1:
		spiMessage[0] = SPI_INSTRUCTION_RTS_BUFFER1;
		break;
	case 2:
		spiMessage[0] = SPI_INSTRUCTION_RTS_BUFFER2;
		break;
	default:
		break;
	}
	writeSimpleCommandSpi(spiMessage[0], do_csEnc28j60);
}

