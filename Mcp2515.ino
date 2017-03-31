#include "MCP2515.h"
#include "Definitions.h"


void mcp2515_init_tx_buffer0(byte identifierLow, byte identifierHigh, byte messageSize) {

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	execute_write_command(REGISTER_TXB0SIDL, identifierLow, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(REGISTER_TXB0SIDH, identifierHigh, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(REGISTER_TXB0DLC, messageSize, do_csMcp2515, SPI_INSTRUCTION_WRITE);
}

void mcp2515_init_tx_buffer1(byte identifierLow, byte identifierHigh, byte messageSize) {

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	execute_write_command(REGISTER_TXB1SIDL, identifierLow, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(REGISTER_TXB1SIDH, identifierHigh, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(REGISTER_TXB1DLC, messageSize, do_csMcp2515, SPI_INSTRUCTION_WRITE);
}

void mcp2515_init_tx_buffer2(byte identifierLow, byte identifierHigh, byte messageSize) {

	// Set the message identifier to 10000000000 and extended identifier bit to 0
	execute_write_command(REGISTER_TXB2SIDL, identifierLow, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(REGISTER_TXB2SIDH, identifierHigh, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(REGISTER_TXB2DLC, messageSize, do_csMcp2515, SPI_INSTRUCTION_WRITE);
}

void initMcp2515() {
	// Reset chip to set in operation mode
	mcp2515_execute_reset_command();

	// Configure bit timing
	mcp2515_configureCanBus();

	// Configure interrupts
	mcp2515_configureInterrupts();

	// Set device to normal mode
	mcp2515_switchMode(REGISTER_CANSTAT_NORMAL_MODE, REGISTER_CANCTRL_NORMAL_MODE);
}

void mcp2515_execute_reset_command() {
	// Reset chip to get initial condition and wait for operation mode state bit
	byte returnMessage;

	writeSimpleCommandSpi(SPI_INSTRUCTION_RESET, do_csMcp2515);

	// Read the register value
	byte actualMode = execute_read_command(REGISTER_CANSTAT, do_csMcp2515, SPI_INSTRUCTION_READ);
	Serial.print("Try to reset Mcp2515");
	while (REGISTER_CANSTAT_CONFIGURATION_MODE != (REGISTER_CANSTAT_CONFIGURATION_MODE & actualMode))
	{
		actualMode = execute_read_command(REGISTER_CANSTAT, do_csMcp2515, SPI_INSTRUCTION_READ);
		Serial.print(actualMode);
	}

	if (debugMode) Serial.print("Mcp2515 reset succesfully and switch do mode ");
	if (debugMode) Serial.println(actualMode);
}

void mcp2515_configureCanBus() {
	// Configure bit timing

	execute_write_command(REGISTER_CNF1, REGISTER_CNF1_VALUE, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	execute_write_command(REGISTER_CNF2, REGISTER_CNF2_VALUE, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	execute_write_command(REGISTER_CNF3, REGISTER_CNF3_VALUE, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	if (debugMode) Serial.println("Mcp2515 configure bus succesfully");
}

void mcp2515_configureInterrupts() {
	execute_write_command(REGISTER_CANINTE, REGISTER_CANINTE_VALUE, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	if (debugMode) Serial.println("Mcp2515 configure interrupts succesfully");
}

void mcp2515_configureMasksFilters(byte registerAddress, byte registerValue) {

	// Set parameters for rx buffer 0
	execute_write_command(REGISTER_RXB0CTRL, REGISTER_RXB0CTRL_VALUE, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	// Set parameters for rx buffer 1
	execute_write_command(REGISTER_RXB1CTRL, REGISTER_RXB1CTRL_VALUE, do_csMcp2515, SPI_INSTRUCTION_WRITE);
}

void mcp2515_configureRxBuffer() {

	// Set parameters for rx buffer 0
	execute_write_command(0x60, 0x04, do_csMcp2515, SPI_INSTRUCTION_WRITE); // as dec 39: Filter 1 is enabled (rollover is active / bukt is set)

	execute_write_command(0x70, 0x00, do_csMcp2515, SPI_INSTRUCTION_WRITE); // as dec 33: Filter 1 is enabled (if bukt is set)

	if (debugMode) Serial.println("Mcp2515 configure rx buffer succesfully");
}

void mcp2515_switchMode(byte modeToCheck, byte modeToSwitch) {
	// Reset chip to get initial condition and wait for operation mode state bit
	byte returnMessage[1];
	byte actualMode;

	// Repeat mode switching for a specific time when the first trial is without success
	for (int i = 0; i < MAX_ERR_CNTR_MDSWCH_MCP2515; i++)
	{
		execute_write_command(REGISTER_CANCTRL, modeToSwitch, do_csMcp2515, SPI_INSTRUCTION_WRITE);

		// Read the register value
		actualMode = execute_read_command(REGISTER_CANSTAT, do_csMcp2515, SPI_INSTRUCTION_READ);
		long elapsedTime = 0;
		long errorTimerValue = millis();
		while ((actualMode != modeToSwitch) && (elapsedTime <= MAX_WAIT_TIME_MCP2515))
		{
			actualMode = execute_read_command(REGISTER_CANSTAT, do_csMcp2515, SPI_INSTRUCTION_READ);
			elapsedTime = millis() - errorTimerValue; // Stop time to break loop when mode isnt switching
		}
		if (elapsedTime > MAX_WAIT_TIME_MCP2515)
		{
			if (debugMode) Serial.println("Abort waiting. Max. waiting time reached.");
			if (i == MAX_ERR_CNTR_MDSWCH_MCP2515)
			{
				if (debugMode) Serial.println("ERROR MODE SWITCH - STOP ALL OPERATIONS");
				stopAllOperations = true;
			}
		}
		else break;
	}

	if (debugMode) Serial.print("Mcp2515 switch to mode ");
	if (debugMode) Serial.print(actualMode);
	if (debugMode) Serial.println(" succesfully");
}


void mcp2515_load_tx_buffer0(byte messageData, int byteNumber, int messageSize) {

	execute_write_command(REGISTER_TXB0Dx[byteNumber], messageData, do_csMcp2515, SPI_INSTRUCTION_WRITE);
}

void mcp2515_load_tx_buffer1(byte messageData, int byteNumber, int messageSize) {

	execute_write_command(REGISTER_TXB1Dx[byteNumber], messageData, do_csMcp2515, SPI_INSTRUCTION_WRITE);
}

void mcp2515_load_tx_buffer2(byte messageData, int byteNumber, int messageSize) {

	execute_write_command(REGISTER_TXB2Dx[byteNumber], messageData, do_csMcp2515, SPI_INSTRUCTION_WRITE);

}

void mcp2515_init_tx_buffer(byte identifierLow, byte identifierHigh, byte messageSize, int regAddr) {

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
	execute_write_command(regAddr_sidl, identifierLow, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(regAddr_sidh, identifierHigh, do_csMcp2515, SPI_INSTRUCTION_WRITE);

	// Set data length and set rtr bit to zero (no remote request)
	execute_write_command(regAddr_dlc, messageSize, do_csMcp2515, SPI_INSTRUCTION_WRITE);
}


void mcp2515_execute_rts_command(int bufferId)
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
	writeSimpleCommandSpi(spiMessage[0], do_csMcp2515);
}



bool readFromCanBus(byte bufferId, int cs_pin)
{
	byte returnMessage[BYTES_TO_READ_MCP2515];

	// Read pwm value as byte from can bus
	setCsPin(cs_pin, LOW);
	SPI.transfer(bufferId);
	for (int i = 0; i < BYTES_TO_READ_MCP2515; i++) {
		incoming_data[i] = SPI.transfer(0x00);
	}
	setCsPin(cs_pin, HIGH);
	//delay(SLOW_DOWN_CODE);

	return true;
}
