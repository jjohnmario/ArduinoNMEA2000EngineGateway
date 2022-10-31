/*
 Name:		ArduinoNMEA2000EngineGateway.ino
 Created:	10/19/2022 1:04:19 PM
 Author:	jjohn
*/

#include "CanMessage.h"
#include <iostream>
#include <map>
#include <cstddef>
#include <cstring>
#include "CAN.h"
#include "LiquidCrystal_I2C.h"
#include <avr/pgmspace.h> 
unsigned long lastMillis;
uint32_t recievePgns[]{ 59392 ,59904, 60928 };
uint32_t transmitPgns[]{0};
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
//VDO,12V,EU,0-10 Bar
double oilPressSensorResistance[] = { 10.00,31.00,52.00,71.00,90.00,107.00,124.00,140.00,156.00,170.00,184.00 };
double oilPressBar[] = { 0.00,1.00,2.00,3.00,4.00,5.00,6.00,7.00,8.00,9.00,10.00 };
//VDO,12V,US,100 to 250F
double engineTempSensorResistance[] = {
	9.78,
	14.15,
	20.46,
	29.60,
	35.48,
	50.61,
	71.55,
	99.00,
	138.22,
	199.00,
	295.68,
	447.15,
	676.84,
	1012.74,
	1487.36,
	2137.79,
	3005.63 
};
double engineTempF[] = {
	302.00,
	284.00,
	286.00,
	248.00,
	230.00,
	212.00,
	194.00,
	176.00,
	158.00,
	140.00,
	122.00,
	104.00,
	86.00,
	68.00,
	50.00,
	32.00,
	14.00 
};
byte NAME[4] = { 0x0,0x0,0x0,0x0 };
bool isFirstScan = true;
char savedAddr;
union {
	long val;
	uint8_t bytes[4];
} extendedPacketId;
typedef struct {
	byte priority;//0=Highest,7=Lowest
	bool msgType;//0=ISO,1=NMEA2000
	unsigned pgn;
	byte sourceAddr;//Adress of producing CA
}Extended_Id;

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
	//Extended packet
	if (CAN.packetExtended()) {
		extendedPacketId.val = CAN.packetId();
		if (sizeof(extendedPacketId.bytes) == 4) {
			Serial.print("Extended packet recieved from address:");
			Serial.println(extendedPacketId.bytes[0]);
			Extended_Id extId = parseCanExtendedId(extendedPacketId.bytes);
			int dataLen = CAN.available();
			uint8_t data[8];
			CAN.readBytes(data, dataLen);
			for (byte b : data)
				Serial.print((char)b);
			bool isRecievePgn = false;//check if pgn is supported
			for (int x = 0; x < sizeof(recievePgns); x++) {
				if (extId.pgn == recievePgns[x]){				
					isRecievePgn = true;
					break;
				}
			}
			if (isRecievePgn) {
				processRecievedMsg(extId, data);//process supported pgn
			}
		}
	}
}
void processRecievedMsg(Extended_Id extId, byte* data) {
	Serial.println("PGN is valid.");
	//Allowed pgns
	switch (extId.pgn) {
	case 59392://ISO Acknowledge
		break;
	case 59904: {//ISO Request

		break;
	}

	case 60928://ISO Address Claim
		break;
	}
}

void processPgn59904(Extended_Id extId, byte* data) {

}

Extended_Id parseCanExtendedId(byte* idFrame)
{
	//idFrame[0] = byte[3] of CAN ID frame
	//idFrame[1] = byte[2] of CAN ID frame
	//idFrame[2] = byte[1] of CAN ID frame
	//idFrame[3] = byte[0] of CAN ID frame
	Serial.print("Pgn:");
	Extended_Id extId;
	if (sizeof(idFrame) == 4) {
		Extended_Id extId;
		extId.msgType = (0x1 & idFrame[3]);//msg type
		extId.pgn = extId.msgType;
		extId.pgn = (extId.pgn << 8) + idFrame[2];//pf
		byte ps;
		if (idFrame[1] > 239)
			ps = 0x0;
		else
			ps = idFrame[1];
		extId.pgn = (extId.pgn << 8) + ps;//ps
		Serial.println(extId.pgn);
		extId.sourceAddr = idFrame[0];//source address
	}
	return extId;
}

byte* makeAddressClaim(int addr) {
	//For now set NAME to zero
	byte bytes[4];
	bytes[0] = 0x0;
	bytes[1] = 0x0;
	bytes[2] = 0x0;
	bytes[3] = 0x0;
}

void sendAddressClaimRequest(uint8_t sourceAddr)
{
	CAN.beginExtendedPacket(0x0);
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
	//sendAddressClaimRequest();
	//savedAddr = getSavedAddr();
	//if(savedAddr == 0)
}

void updateLcd(double waterTemp, double oilPress) {
	lcd.setCursor(1, 0);
	lcd.print("WATER F:");
	lcd.print(waterTemp, 0);
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
	CAN.loopback();
	CAN.onReceive(onCanRecieved);
	lcd.clear();
	analogReadResolution(12);
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

		isFirstScan = false;
	}


	// send packet: id is 11 bits, packet can contain up to 8 bytes of data
	//Serial.print("Sending packet ... ");

	//CAN.beginPacket(0x12);
	//CAN.write('h');
	//CAN.write('e');
	//CAN.write('l');
	//CAN.write('l');
	//CAN.write('o');
	//CAN.endPacket();

	//Serial.println("done");

	// send extended packet: id is 29 bits, packet can contain up to 8 bytes of data
	//Serial.print("Sending extended packet ... ");

	//CAN.beginExtendedPacket(0x18DA6005);
	//CAN.write('w');
	//CAN.write('o');
	//CAN.write('r');
	//CAN.write('l');
	//CAN.write('d');
	//CAN.endPacket();

	//Serial.println("done");

	double vInMin = 0.0;
	double vInMax = 3.3;
	double vIn = 5.0;
	double r1 = 1000.00;
	double r2 = 0.00;
	double val = 0.00;

	//Coolant temp
	//Serial.println(analogRead(A0));
	double ref = analogInputToVolts(0, 4096, analogRead(A1), 0, 3.3);
	vIn = ref * 2;
	Serial.println(ref);
	double vOut = analogInputToVolts(0, 4096, analogRead(A0), vInMin, vInMax);
	r2 = voltageToR2(vIn, vOut, r1);
	Serial.println(r2);
	val = multiMap(r2, engineTempSensorResistance, engineTempF,17);
	Serial.println(val);
	//Serial.println(r2);
	//Serial.println(vOut);
	//Serial.print("The engine temp is:");
	//Serial.print(val,0);
	//Serial.println("F");

	if (millis() - lastMillis >= 2 * 1000UL)
	{
		lastMillis = millis();  //get ready for the next iteration
		updateLcd(val,0);
	}

	//Oil pressure
	//r2 = voltageToResistance(analogRead(A1), vIn, r1);
	//val = multiMap(r2, oilPressSensorResistance, oilPressBar, 11);
	//Serial.print("The engine oil pressure is:");
	//Serial.print(val);
	//Serial.println("Bar");
	//lcd.setCursor(1, 1);
	//lcd.print("OIL BAR:");
	//lcd.print(val);

	delay(1000);
}

double analogInputToVolts(int countsMin, int countsMax, int countsIn, double vMin, double vMax) {
	double vOut = countsIn * ((vMax - vMin) / (countsMax - countsMin));
	return vOut;
}

double voltageToR2(double vIn, double vOut, double r1) {
	double r2 = (vOut * r1) / (vIn - vOut);
	return r2;
}

double voltageToResistance(int countsMin, int countsMax, int countsIn, double vMin, double vMax, double r1)
{
	//Get volts per input count


	//Calculate resistance from voltage in
	//vOut = R2/(R1+R2) * Vin
	//R2 = (vOut*R1)/(vIn-vOut)
	//R2 = (countsIn

	//0-1024 counts = 0-3.3V
	// 1 count = 0.00322266V
	//0.0024390 volts per ohm of resistance
	// 3.3 volts/
	// Convert the analog reading counts (which goes from minCounts to maxCounts) to a voltage (vMin to vMax):
	float vOut = countsIn * ((vMax-vMin) / (countsMax - countsMin));
	Serial.println(vOut);
	// Find the resistance of R2. R2 = Vout*R1/Vin-Vout
	float r2 = vOut * r1 / ((vMax-vMin) - vOut);
	return r2;
}

