/*
 Name:		ArduinoNMEA2000EngineGateway.ino
 Created:	10/19/2022 1:04:19 PM
 Author:	jjohn
*/

#include "DeviceInformation.h"
#include "CanMessage.h"
#include <iostream>
#include <map>
#include <cstddef>
#include <cstring>
#include "CAN.h"
#include "LiquidCrystal_I2C.h"
#include <avr/pgmspace.h> 

uint8_t pgn126996FastPacketSeq = 0x0;
uint8_t pgn126464FastPacketSeq = 0x64;
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
uint64_t thisCanNAME;
bool isFirstScan = true;
byte claimedAddr = 0x2;
union {
	long val;
	uint8_t bytes[4];
} extendedPacketId;
typedef struct {
	byte priority;//0=Highest,7=Lowest
	bool msgType;//0=ISO,1=NMEA2000
	unsigned pgn;
	byte sourceAddr;//Address of producing CA
	byte desinationAddr;//Address of target CA
	byte pf;//PF (byte 1)
	byte ps;//PS (byte 2)
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

//New message from CAN bus.
void onCanRecieved(int size) {
	//Extended packet
	if (CAN.packetExtended()) {
		extendedPacketId.val = CAN.packetId();
		if (sizeof(extendedPacketId.bytes) == 4) {
			Extended_Id extId;
			extId = parseCanExtendedId(extendedPacketId.bytes);

			//Read up to 8 bytes of data 
			int dataLen = CAN.available();
			uint8_t data[8];
			CAN.readBytes(data, dataLen);

			//Process message
			processN2kMsg(extId, data);
		}
	}
}

void processN2kMsg(Extended_Id extId, byte* data)
{
	//ISO PGN Request 59904
	if (extId.pgn == 59904)
	{
		//Decode requested PGN
		Serial.print("ISO Request (59904) for PGN: ");
		uint32_t reqPgn = (0x1 & data[2]);//msg type 0=ISO, 1=NMEA2000
		reqPgn = (reqPgn << 8) + data[1];
		reqPgn = (reqPgn << 8) + data[0];
		Serial.println(reqPgn);

		//Requested PGNs
		switch (reqPgn) {
			
			case 60928: { //ISO Address Claim
				writePgn60928(255);
				break;
			}
			case 126996: {//Product Information
				writePgn126996(255);
				break;
			}
			case 126464: { //PGN List
				writePgn126464(255);
				break;
			}
		}
	}
	//ISO Address Claim 60928
	else if (extId.pgn == 60928)
	{
		//Parse 8 byte array to NAME
		uint64_t name = data[0];
		name = (name << 8) + data[1];
		name = (name << 8) + data[2];
		name = (name << 8) + data[3];
		name = (name << 8) + data[4];
		name = (name << 8) + data[5];
		name = (name << 8) + data[6];
		name = (name << 8) + data[7];
		negotiateAddressClaim(extId.sourceAddr, extId.desinationAddr, name);
	}
}

//Parses a byte array to an extended packet ID.
Extended_Id parseCanExtendedId(byte* idFrame)
{
	//idFrame[0] = byte[3] of CAN ID frame
	//idFrame[1] = byte[2] of CAN ID frame
	//idFrame[2] = byte[1] of CAN ID frame
	//idFrame[3] = byte[0] of CAN ID frame
	Extended_Id extId;
	if (sizeof(idFrame) == 4) {
		extId.sourceAddr = idFrame[0];//source address
		extId.ps = idFrame[1];//ps
		extId.pf = idFrame[2];//pf
		extId.msgType = (0x1 & idFrame[3]);//msg type
		extId.pgn = extId.msgType;
		extId.pgn = (extId.pgn << 8) + extId.pf;//pf
		byte ps;
		//if pf is 0 - 239, ps is the address of the destination CA
		//if pf is > 239, ps is meaningless relative to the destination CA
		if (extId.pf < 240)
		{
			extId.desinationAddr = ps;
			extId.ps = 0x0;
		}
		extId.pgn = (extId.pgn << 8) + extId.ps;//ps
	}
	return extId;
}

//Sends an address claim to the desired destination.
void sendAddressClaim(byte srcAddr, byte destAddr, uint64_t name) {

	if (thisCanNAME != 0)
	{
		uint32_t id = 0x18;
		id = (id << 8) + 0xEE;
		id = (id << 8) + (destAddr);
		id = (id << 8) + (srcAddr);
		CAN.beginExtendedPacket(id);
		byte* nameBytes = (byte*)&thisCanNAME;
		CAN.write(nameBytes[0]);
		CAN.write(nameBytes[1]);
		CAN.write(nameBytes[2]);
		CAN.write(nameBytes[3]);
		CAN.write(nameBytes[4]);
		CAN.write(nameBytes[5]);
		CAN.write(nameBytes[6]);
		CAN.write(nameBytes[7]);
		CAN.endPacket();
	}
}

//Negotiates an address claim for a network CAN device.
void negotiateAddressClaim(byte srcAddr, byte destAddr, uint64_t name) {
	if (thisCanNAME != 0)
	{
		/*
		* If the address of the calling device is equal to this device then
		* compare NAME of this device and the calling device.
		* If this devices NAME is greater than the calling device's NAME
		* then this device must surrender its address and claim a new address
		* using the address claim process.
		*/
		//Compare NAME of this device and the calling device.
		if (srcAddr == claimedAddr) {
			if (thisCanNAME > name) {
				Serial.println("Address conflict detected. Claiming new address...");
				claimedAddr++;
				sendAddressClaim(claimedAddr, 255, thisCanNAME);
			}
		}
	}
}

//PGN 60928: Address Claim
void writePgn60928(byte destAddr) {
	sendAddressClaim(claimedAddr, destAddr, thisCanNAME);
}

//PGN 126464: PGN List Request (transmit/recieve)
void writePgn126464(byte destAddr) {

	//Reset sequencer
	pgn126464FastPacketSeq = 0;

	//Create packet ID
	uint32_t id = 0x19;
	id = (id << 8) + 0xEE;
	id = (id << 8) + 0x00;
	id = (id << 8) + (claimedAddr);
	
	//Transmit PGNs
	//Fast packet 1
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126464FastPacketSeq);
	CAN.write(0xD);//13 bytes to data to write
	CAN.write(0x0); //0 = Transmit pgn
	CAN.write(0x0); //PGN 59904
	CAN.write(0xEA);
	CAN.write(0x0);
	CAN.write(0x0);//PGN 60928 (bytes 1-2)
	CAN.write(0xEE);
	CAN.endPacket();
	pgn126464FastPacketSeq++;

	//Fast packet 2
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126464FastPacketSeq);
	CAN.write(0x0);//PGN 60928 (byte 3)
	CAN.write(0x0D);//PGN 127245
	CAN.write(0xF1);
	CAN.write(0x1);
	CAN.write(0x10);//PGN 130576
	CAN.write(0xFE);
	CAN.write(0x1);
	CAN.endPacket();

	//Reset sequencer
	pgn126464FastPacketSeq = 0;

	//Recieve PGNs
	//Fast packet 1
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126464FastPacketSeq);
	CAN.write(0x7);//7 bytes to data to write
	CAN.write(0x1); //1 = Recieve pgn
	CAN.write(0x0); //PGN 59904
	CAN.write(0xEA);
	CAN.write(0x0);
	CAN.write(0x0);//PGN 60928 (bytes 1-2)
	CAN.write(0xEE);
	CAN.endPacket();
	pgn126464FastPacketSeq++;

	//Fast packet 2
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126464FastPacketSeq);
	CAN.write(0x0);//PGN 60928 (byte 3)
	CAN.write(0xFF);
	CAN.write(0xFF);
	CAN.write(0xFF);
	CAN.write(0xFF);
	CAN.write(0xFF);
	CAN.write(0xFF);
	CAN.endPacket();
	pgn126464FastPacketSeq++;
}

//PGN 126996: Product Information Request
void writePgn126996(byte destAddr) {

	//Data
	uint8_t n2Kver[2];
	n2Kver[1] = 0x08;
	n2Kver[0] = 0x35;
	uint8_t prodCode[2];
	prodCode[1] = 0x04;
	prodCode[0] = 0xD2;
	char modId[32] = { '/0' };
	String modIdStr = "Marioware Rudder/Trim Module";
	modIdStr.toCharArray(modId, modIdStr.length()+1, 0);
	char swVer[32] = {'/0'};
	String swVerStr = "1.0.0.0 (2022-12-02)";
	swVerStr.toCharArray(swVer, swVerStr.length()+1, 0);
	char modVer[32] = { '/0' };
	String modVerStr = "1.0.0.0 (2022-12-02)";
	modVerStr.toCharArray(modVer, modVerStr.length()+1, 0);
	char modSer[32] = { '/0' };
	String modSerStr = "00000001";
	modSerStr.toCharArray(modSer, modSerStr.length()+1, 0);
	uint8_t certLvl = 0;
	uint8_t loadEqv = 2;

	//Reset sequencer
	pgn126996FastPacketSeq = 0;

	//Create packet ID
	uint32_t id = 0x19;
	id = (id << 8) + 0xF0;
	id = (id << 8) + 0x14;
	id = (id << 8) + (claimedAddr);

	//Fast packet 1
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(0x86);//134 bytes of data to write
	CAN.write(n2Kver[0]);
	CAN.write(n2Kver[1]);
	CAN.write(prodCode[0]);
	CAN.write(prodCode[1]);
	CAN.write(modId[0]);
	CAN.write(modId[1]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 2
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modId[2]);
	CAN.write(modId[3]);
	CAN.write(modId[4]);
	CAN.write(modId[5]);
	CAN.write(modId[6]);
	CAN.write(modId[7]);
	CAN.write(modId[8]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 3
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modId[9]);
	CAN.write(modId[10]);
	CAN.write(modId[11]);
	CAN.write(modId[12]);
	CAN.write(modId[13]);
	CAN.write(modId[14]);
	CAN.write(modId[15]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 4
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modId[16]);
	CAN.write(modId[17]);
	CAN.write(modId[18]);
	CAN.write(modId[19]);
	CAN.write(modId[20]);
	CAN.write(modId[21]);
	CAN.write(modId[22]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 5
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modId[23]);
	CAN.write(modId[24]);
	CAN.write(modId[25]);
	CAN.write(modId[26]);
	CAN.write(modId[27]);
	CAN.write(modId[28]);
	CAN.write(modId[29]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 6
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modId[30]);
	CAN.write(modId[31]);
	CAN.write(swVer[0]);
	CAN.write(swVer[1]);
	CAN.write(swVer[2]);
	CAN.write(swVer[3]);
	CAN.write(swVer[4]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 7
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(swVer[5]);
	CAN.write(swVer[6]);
	CAN.write(swVer[7]);
	CAN.write(swVer[8]);
	CAN.write(swVer[9]);
	CAN.write(swVer[10]);
	CAN.write(swVer[11]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 8
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(swVer[12]);
	CAN.write(swVer[13]);
	CAN.write(swVer[14]);
	CAN.write(swVer[15]);
	CAN.write(swVer[16]);
	CAN.write(swVer[17]);
	CAN.write(swVer[18]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 9
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(swVer[19]);
	CAN.write(swVer[20]);
	CAN.write(swVer[21]);
	CAN.write(swVer[22]);
	CAN.write(swVer[23]);
	CAN.write(swVer[24]);
	CAN.write(swVer[25]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 10
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(swVer[26]);
	CAN.write(swVer[27]);
	CAN.write(swVer[28]);
	CAN.write(swVer[29]);
	CAN.write(swVer[30]);
	CAN.write(swVer[31]);
	CAN.write(modVer[0]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 11
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modVer[1]);
	CAN.write(modVer[2]);
	CAN.write(modVer[3]);
	CAN.write(modVer[4]);
	CAN.write(modVer[5]);
	CAN.write(modVer[6]);
	CAN.write(modVer[7]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 12
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modVer[8]);
	CAN.write(modVer[9]);
	CAN.write(modVer[10]);
	CAN.write(modVer[11]);
	CAN.write(modVer[12]);
	CAN.write(modVer[13]);
	CAN.write(modVer[14]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 13
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modVer[15]);
	CAN.write(modVer[16]);
	CAN.write(modVer[17]);
	CAN.write(modVer[18]);
	CAN.write(modVer[19]);
	CAN.write(modVer[20]);
	CAN.write(modVer[21]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 14
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modVer[22]);
	CAN.write(modVer[23]);
	CAN.write(modVer[24]);
	CAN.write(modVer[25]);
	CAN.write(modVer[26]);
	CAN.write(modVer[27]);
	CAN.write(modVer[28]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 15
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modVer[29]);
	CAN.write(modVer[30]);
	CAN.write(modSer[31]);
	CAN.write(modSer[0]);
	CAN.write(modSer[1]);
	CAN.write(modSer[2]);
	CAN.write(modSer[3]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 16
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modSer[4]);
	CAN.write(modSer[5]);
	CAN.write(modSer[6]);
	CAN.write(modSer[7]);
	CAN.write(modSer[8]);
	CAN.write(modSer[9]);
	CAN.write(modSer[10]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 17
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modSer[11]);
	CAN.write(modSer[12]);
	CAN.write(modSer[13]);
	CAN.write(modSer[14]);
	CAN.write(modSer[15]);
	CAN.write(modSer[16]);
	CAN.write(modSer[17]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 18
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modSer[18]);
	CAN.write(modSer[19]);
	CAN.write(modSer[20]);
	CAN.write(modSer[21]);
	CAN.write(modSer[22]);
	CAN.write(modSer[23]);
	CAN.write(modSer[24]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 19
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(modSer[25]);
	CAN.write(modSer[26]);
	CAN.write(modSer[27]);
	CAN.write(modSer[28]);
	CAN.write(modSer[29]);
	CAN.write(modSer[30]);
	CAN.write(modSer[31]);
	CAN.endPacket();
	pgn126996FastPacketSeq++;

	//Fast packet 20
	CAN.beginExtendedPacket(id);
	CAN.write(pgn126996FastPacketSeq);
	CAN.write(certLvl);
	CAN.write(loadEqv);
	CAN.endPacket();
}

//PGN 127345: Rudder
void writePgn127245(byte destAddr) {

	//1 radian = 0.0057295779513082332 degrees
	// 0.785398 radians = 45 degrees
	// 1.5708 radians = 90 degress
	//Create packet ID
	uint32_t id = 0xD;
	id = (id << 8) + 0xF1;
	id = (id << 8) + 0x0D;
	id = (id << 8) + (claimedAddr);
	CAN.beginExtendedPacket(id);
	CAN.write(0x0);//Instance
	CAN.write(0x0);//Direction order
	CAN.write(0x0);//Angle order
	CAN.write(0x0);
	CAN.write(0x5C);//Postion in radians/10000 (example 13/10000 = 0.0013)
	CAN.write(0x3D);
	CAN.write(0xFF);
	CAN.write(0xFF);
	CAN.endPacket();
}

//Small Craft Status (Trim Tabs)
void writePgn130576(byte destAddr){

	// 0-100%, 0% = tabs fully up, 100% = tabs fully down
	//Create packet ID
	uint32_t id = 0xD;
	id = (id << 8) + 0xFE;
	id = (id << 8) + 0x10;
	id = (id << 8) + (claimedAddr);
	CAN.beginExtendedPacket(id);
	CAN.write(0xB);//Port trim tab
	CAN.write(0xD);//Stbd trim tab
	CAN.write(0xFF);
	CAN.write(0xFF);
	CAN.write(0xFF);
	CAN.write(0xFF);
	CAN.write(0xFF);
	CAN.write(0xFF);
	CAN.endPacket();
}



//Creates CAN device NAME
uint64_t createDeviceName(uint32_t mfrCode,uint32_t serial,uint8_t funcInst,
	uint8_t ecuInst,uint8_t function, uint8_t vehicleSys,uint8_t indGroup, uint8_t vehicleSysInst)
{
	/*
		64-bit CAN NAME Fields
		**********************
		Bits 0-20: Identity number – This field is assigned by the manufacturer, similar to a serial number, i.e. the code must be uniquely assigned to the unit.
		Bits 21-31: Manufacturer code – The 11-Bit Manufacturer Code is assigned by the SAE.
		Bits 32-34: ECU Instance – A J1939 network may accommodate several ECUs of the same kind (i.e. same functionality). The Instance code separates them.
		Bits 35-39: Function Instance
		Bits 40-47: Function – This code, in a range between 128 and 255, is assigned according to the Industry Group. A value between 0 and 127 is not associated with any other parameter.
		Bit 48: Reserved – Always zero.
		Bits 49-55: Vehicle System – Vehicle systems are associated with the Industry Group and they can be, for instance, “tractor” in the “Common” industry or “trailer” in the “On-Highway” industry group.
		Bits 56-59: Vehicle System Instance – Assigns a number to each instance on the Vehicle System (in case you connect several networks – e.g. connecting cars on a train).
		Bits 60-62: Industry Group – These codes are associated with particular industries such as on-highway equipment, agricultural equipment, and more.
		Bit 63: Arbitrary Address Capable – Indicates whether or not the ECU/CA can negotiate an address (1 = yes; 0 = no). Some ECUs can only support one address; others support an address range.
	*/
	//Limits
	if (mfrCode > 0x1FFFFF)
		mfrCode = 0x1FFFFF;
	if (funcInst > 0x1F)
		funcInst = 0x1F;
	if (ecuInst > 0x7)
		ecuInst = 0x7;
	if (vehicleSys > 0x7F)
		vehicleSys = 0x7F;
	if (indGroup > 0x7)
		indGroup = 0x7;
	if (vehicleSysInst > 0xF)
		vehicleSysInst = 0xF;

	//11 bits mfr code, 21 bits serial number
	uint32_t mfrser = (mfrCode << 21);
	mfrser = mfrser | serial;

	//Function instance 5 bits, ecu instance 3 bits
	uint8_t func_and_ecu_inst;
	func_and_ecu_inst = (funcInst << 3);
	func_and_ecu_inst = func_and_ecu_inst | ecuInst;

	//Function - no formatting required.

	//vehicle system 7 bits, bit 0 reserved always 0
	vehicleSys = vehicleSys<< 1;

	//Arbitrary addr capable 1 bit, industry group 3 bits, vehicle system instance 4 bits
	uint8_t arb_addr_cap = 1;
	uint8_t combined = arb_addr_cap << 3;
	combined = combined | indGroup;
	combined = combined << 4;
	combined = combined | vehicleSysInst;

	//package it up MSB to LSB
	uint64_t name = 0;
	name = combined;
	name = name << 8;

	name = name | vehicleSys;
	name = name << 8;

	name = name | function;
	name = name << 8;

	name = name | func_and_ecu_inst;
	name = name << 32;

	name = name | mfrser;
	return name;
}

void updateLcd(double waterTemp, double oilPress) {
	lcd.setCursor(1, 0);
	lcd.print("WATER F:");
	lcd.print(waterTemp, 0);
}

// the setup function runs once when you press reset or power the board
void setup(){
	// initialize the lcd 
	lcd.init();                      
	// Print a message to the LCD.
	lcd.backlight();
	lcd.setCursor(1, 0);
	lcd.print("MARIOWARE v0.1");
	lcd.setCursor(1, 1);
	lcd.print("BOOTING...");
	//delay(3000);

	//Create CAN device NAME
	thisCanNAME = createDeviceName(2046, 1, 0, 0, 130, 25, 4, 0);

	//start the CAN bus at 250 kbps
	if (!CAN.begin(250E3)) {
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

	//Subscribe to new CAN messages
	CAN.onReceive(onCanRecieved);

	//lcd.clear();
	//analogReadResolution(12);
	
	//Send address claim
	sendAddressClaim(claimedAddr, 255, thisCanNAME);
}
// the loop function runs over and over again until power down or reset
void loop()
{
	writePgn127245(255);
	writePgn130576(255);
	delay(500);
	//double vInMin = 0.0;
	//double vInMax = 3.3;
	//double vIn = 5.0;
	//double r1 = 1000.00;
	//double r2 = 0.00;
	//double val = 0.00;

	////Coolant temp
	////Serial.println(analogRead(A0));
	//double ref = analogInputToVolts(0, 4096, analogRead(A1), 0, 3.3);
	//vIn = ref * 2;
	//Serial.println(ref);
	//double vOut = analogInputToVolts(0, 4096, analogRead(A0), vInMin, vInMax);
	//r2 = voltageToR2(vIn, vOut, r1);
	//Serial.println(r2);
	//val = multiMap(r2, engineTempSensorResistance, engineTempF,17);
	//Serial.println(val);
	//Serial.println(r2);
	//Serial.println(vOut);
	//Serial.print("The engine temp is:");
	//Serial.print(val,0);
	//Serial.println("F");

	//if (millis() - lastMillis >= 2 * 1000UL)
	//{
	//	lastMillis = millis();  //get ready for the next iteration
	//	updateLcd(val,0);
	//}

	//Oil pressure
	//r2 = voltageToResistance(analogRead(A1), vIn, r1);
	//val = multiMap(r2, oilPressSensorResistance, oilPressBar, 11);
	//Serial.print("The engine oil pressure is:");
	//Serial.print(val);
	//Serial.println("Bar");
	//lcd.setCursor(1, 1);
	//lcd.print("OIL BAR:");
	//lcd.print(val);

	//delay(1000);
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

