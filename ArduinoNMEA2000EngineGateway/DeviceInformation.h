// DeviceInformation.h

#ifndef _DEVICEINFORMATION_h
#define _DEVICEINFORMATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif
class tDeviceInformation {
protected:
    typedef union {
        uint64_t Name;
        struct {
            uint32_t UnicNumberAndManCode; // ManufacturerCode 11 bits , UniqueNumber 21 bits
            unsigned char DeviceInstance;
            unsigned char DeviceFunction;
            unsigned char DeviceClass;
            // I found document: http://www.novatel.com/assets/Documents/Bulletins/apn050.pdf it says about next fields:
            // The System Instance Field can be utilized to facilitate multiple NMEA 2000 networks on these larger marine platforms.
            // NMEA 2000 devices behind a bridge, router, gateway, or as part of some network segment could all indicate this by use
            // and application of the System Instance Field.
            // DeviceInstance and SystemInstance fields can be now changed by function SetDeviceInformationInstances or
            // by NMEA 2000 group function. Group function handling is build in the library.
            unsigned char IndustryGroupAndSystemInstance; // 4 bits each
        };
    } tUnionDeviceInformation;

    tUnionDeviceInformation DeviceInformation;

public:
    tDeviceInformation() { DeviceInformation.Name = 0; }
    void SetUniqueNumber(uint32_t _UniqueNumber) { DeviceInformation.UnicNumberAndManCode = (DeviceInformation.UnicNumberAndManCode & 0xffe00000) | (_UniqueNumber & 0x1fffff); }
    uint32_t GetUniqueNumber() const { return DeviceInformation.UnicNumberAndManCode & 0x1fffff; }
    void SetManufacturerCode(uint16_t _ManufacturerCode) { DeviceInformation.UnicNumberAndManCode = (DeviceInformation.UnicNumberAndManCode & 0x1fffff) | (((unsigned long)(_ManufacturerCode & 0x7ff)) << 21); }
    uint16_t GetManufacturerCode() const { return DeviceInformation.UnicNumberAndManCode >> 21; }
    void SetDeviceInstance(unsigned char _DeviceInstance) { DeviceInformation.DeviceInstance = _DeviceInstance; }
    unsigned char GetDeviceInstance() const { return DeviceInformation.DeviceInstance; }
    unsigned char GetDeviceInstanceLower() const { return DeviceInformation.DeviceInstance & 0x07; }
    unsigned char GetDeviceInstanceUpper() const { return (DeviceInformation.DeviceInstance >> 3) & 0x1f; }
    void SetDeviceFunction(unsigned char _DeviceFunction) { DeviceInformation.DeviceFunction = _DeviceFunction; }
    unsigned char GetDeviceFunction() const { return DeviceInformation.DeviceFunction; }
    void SetDeviceClass(unsigned char _DeviceClass) { DeviceInformation.DeviceClass = ((_DeviceClass & 0x7f) << 1); }
    unsigned char GetDeviceClass() const { return DeviceInformation.DeviceClass >> 1; }
    void SetIndustryGroup(unsigned char _IndustryGroup) { DeviceInformation.IndustryGroupAndSystemInstance = (DeviceInformation.IndustryGroupAndSystemInstance & 0x0f) | (_IndustryGroup << 4) | 0x80; }
    unsigned char GetIndustryGroup() const { return (DeviceInformation.IndustryGroupAndSystemInstance >> 4) & 0x07; }
    void SetSystemInstance(unsigned char _SystemInstance) { DeviceInformation.IndustryGroupAndSystemInstance = (DeviceInformation.IndustryGroupAndSystemInstance & 0xf0) | (_SystemInstance & 0x0f); }
    unsigned char GetSystemInstance() const { return DeviceInformation.IndustryGroupAndSystemInstance & 0x0f; }

    uint64_t GetName() const { return DeviceInformation.Name; }
    void SetName(uint64_t _Name) { DeviceInformation.Name = _Name; }
    inline bool IsSame(uint64_t Other) { return GetName() == Other; }
};

