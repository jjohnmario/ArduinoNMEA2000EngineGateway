/*
 Name:		ArduinoNMEA2000EngineGateway.ino
 Created:	10/19/2022 1:04:19 PM
 Author:	jjohn
*/

#include <iostream>
#include <map>
#include <cstddef>
#include <cstring>
#include "CAN.h"
#include "LiquidCrystal_I2C.h"
#include <avr/pgmspace.h> 

LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
//VDO,12V,EU,0-10 Bar
double oilPressSensorResistance[] = { 10.00,31.00,52.00,71.00,90.00,107.00,124.00,140.00,156.00,170.00,184.00 };
double oilPressBar[] = { 0.00,1.00,2.00,3.00,4.00,5.00,6.00,7.00,8.00,9.00,10.00 };
//VDO,12V,US,100 to 250F
double engineTempSensorResistance[] = { 9.78,14.15,20.46,29.60,35.48,50.61,71.55,99.00,138.22,199.00,295.68,447.15,676.84,1012.74,1487.36,2137.79,3005.63 };
double engineTempF[] = { 302.00,284.00,286.00,248.00,230.00,212.00,194.00,176.00,158.00,140.00,122.00,104.00,86.00,68.00,50.00,32.00,14.00 };

bool isFirstScan = true;
char savedAddr;

// note: the _in array should have increasing values
double multiMap(double val, double* _in, double* _out, uint8_t size)
{
	// take care the value is within range
	// val = constrain(val, _in[0], _in[size-1]);
	if (val <= _in[0]) return _out[0];
	if (val >= _in[size - 1]) return _out[size - 1];

	// search right interval
	uint8_t pos = 1;  // _in[0] allready tested
	while (val > _in[pos]) pos++;

	// this will handle all exact "points" in the _in array
	if (val == _in[pos]) return _out[pos];

	// interpolate in the right segment for the rest
	return (val - _in[pos - 1]) * (_out[pos] - _out[pos - 1]) / (_in[pos] - _in[pos - 1]) + _out[pos - 1];
}

void onCanRecieved(int size) {

}

void sendAddressClaim(int addr) {

}

void sendAddressClaimRequest(uint8_t sourceAddr)
{
	CAN.beginExtendedPacket(id);
	CAN.endPacket();
	delay(1000);//delay while address claims are returned


}

int getSavedAddr()
{
	//get address from flash memory
	savedAddr = pgm_read_byte(0x0);
	return savedAddr;
}

void firstScan() {
	sendAddressClaimRequest();
	savedAddr = getSavedAddr();
	if(savedAddr == 0)
}

// the setup function runs once when you press reset or power the board
void setup() 
{
	lcd.init();                      // initialize the lcd 
	// Print a message to the LCD.
	lcd.backlight();
	lcd.setCursor(1, 0);
	lcd.print("MARIOWARE v0.1");
	lcd.setCursor(1, 1);
	lcd.print("BOOTING...");
	delay(3000);

	// start the CAN bus at 500 kbps
	if (!CAN.begin(500E3)) {
		lcd.clear();
		lcd.setCursor(1, 0);
		lcd.print("CAN BUS INIT FAILED!");
		while (1);
	}
	else
	{
		lcd.clear();
		lcd.setCursor(1, 0);
		lcd.print("CAN BUS ONLINE!");
	}
	CAN.onReceive(onCanRecieved);
	lcd.clear();

}

// the loop function runs over and over again until power down or reset
void loop()
{
	//First pass
	if (isFirstScan) {
		firstScan();
		//sendAddressClaimRequest();
		//CAN.available
		//sendAddressClaim(getSavedAddr());

		firstScan = false;
	}


	// send packet: id is 11 bits, packet can contain up to 8 bytes of data
	Serial.print("Sending packet ... ");

	CAN.beginPacket(0x12);
	CAN.write('h');
	CAN.write('e');
	CAN.write('l');
	CAN.write('l');
	CAN.write('o');
	CAN.endPacket();

	Serial.println("done");

	// send extended packet: id is 29 bits, packet can contain up to 8 bytes of data
	Serial.print("Sending extended packet ... ");

	CAN.beginExtendedPacket(0xabcdef);
	CAN.write('w');
	CAN.write('o');
	CAN.write('r');
	CAN.write('l');
	CAN.write('d');
	CAN.endPacket();

	Serial.println("done");


	double vIn = 3.30;
	double r1 = 1000.00;
	double r2 = 0.00;
	double val = 0.00;

	//Coolant temp
	Serial.println(analogRead(A0));
	r2 = voltageToResistance(analogRead(A0), vIn, r1);
	val = multiMap(r2, engineTempSensorResistance, engineTempF,11);
	//Serial.print("The engine temp is:");
	//Serial.print(val);
	//Serial.println("F");
	lcd.setCursor(1, 0);
	lcd.print("WATER F:");
	lcd.print(val);

	//Oil pressure
	r2 = voltageToResistance(analogRead(A1), vIn, r1);
	val = multiMap(r2, oilPressSensorResistance, oilPressBar, 11);
	//Serial.print("The engine oil pressure is:");
	//Serial.print(val);
	//Serial.println("Bar");
	lcd.setCursor(1, 1);
	lcd.print("OIL BAR:");
	lcd.print(val);
}

double voltageToResistance(int countsIn, double vIn, double r1)
{
	// Convert the analog reading counts (which goes from 0 - 1023) to a voltage (0 - vIn):
	float vOut = countsIn * (vIn / 1023.0);
	// Find the resistance of R2. R2 = Vout*R1/Vin-Vout
	float r2 = vOut * r1 / (vIn - vOut);
	return r2;
}

