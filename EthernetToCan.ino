#include <SPI.h>
#include "ENC28J60.h"
#include "DEFINITIONS.h"

void setup()
{
	// Configure serial interface
	Serial.begin(9600);

	// USER CONFIGURATION
	debugMode = true;

	pinMode(do_csEnc28j60, OUTPUT);
	pinMode(do_csMcp2515, OUTPUT);
	pinMode(di_enc28J60_int_rec, INPUT);

	// Configure SPI
	SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
	SPI.begin();

	// Configure MCP2515
	initEnc28J60();

	// Give time to set up
	delay(100);
}

void loop()
{

  /* add main program code here */

}
