void writeToSpi(byte address, byte data, int cs_pin) {
	byte spiMessage[] = { address, data };

	setCsPin(cs_pin, LOW);
	for (int i = 0; i < 2; i++) SPI.transfer(spiMessage[i]);
	setCsPin(cs_pin, HIGH);
	delay(SLOW_DOWN_CODE);
}

void setCsPin(int cs_pin, uint8_t value) {
	digitalWrite(10, value);
	digitalWrite(cs_pin, value);
}

void execute_write_command(byte address, byte data, int cs_pin, byte spi_instruction)
{
	setCsPin(cs_pin, LOW); // Enable device
	SPI.transfer(spi_instruction); // Write spi instruction write  
	SPI.transfer(address);
	SPI.transfer(data);
	setCsPin(cs_pin, HIGH); // Disable device
	delay(SLOW_DOWN_CODE);
}

void writeSimpleCommandSpi(byte command, int cs_pin)
{
	setCsPin(cs_pin, LOW);
	SPI.transfer(command);
	setCsPin(cs_pin, HIGH);
	delay(SLOW_DOWN_CODE);
}

byte execute_read_command(byte registerToRead, int cs_pin, byte spi_instruction)
{
	byte returnMessage;

	// Enable device
	setCsPin(cs_pin, LOW);

	// Write spi instruction read  
	SPI.transfer(spi_instruction);

	// Write the address of the register to read
	SPI.transfer(registerToRead);
	returnMessage = SPI.transfer(0x00);

	// Disable device
	setCsPin(cs_pin, HIGH);

	delay(SLOW_DOWN_CODE);

	execute_write_command(REGISTER_CANINTF, REGISTER_CANINTF_VALUE_RESET_ALL_IF, do_csMcp2515);

	return returnMessage;
}

byte execute_read_state_command(int cs_pin, byte spi_instruction)
{
	byte returnMessage;

	//SPI_INSTRUCTION_READ_STATUS

	// Enable device
	setCsPin(cs_pin, LOW);

	// Write spi instruction read  
	SPI.transfer(SPI_INSTRUCTION_READ_STATUS);
	returnMessage = SPI.transfer(0x00);

	// Disable device
	setCsPin(cs_pin, HIGH);

	delay(SLOW_DOWN_CODE);

	return returnMessage;
}
