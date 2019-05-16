#ifndef DS2482_H
#define DS2482_H

#include "i2c.h"

#define DS2482_DEFAULT_ADDRESS 0x18

#define DS2482_ERROR_TIMEOUT		(1<<0)
#define DS2482_ERROR_SHORT			(1<<1)
#define DS2482_ERROR_CONFIG			(1<<2)

class DS2482
{
private:
	uint8_t address_;
	bool present_;
	I2C *i2c_; // The i2c object used for communication
	uint8_t buf_[256];
  int idx_;
	uint8_t error_;
	uint8_t searchAddress[8];
  uint8_t searchLastDiscrepancy;
  uint8_t searchLastDeviceFlag;
	
public:
	bool init(I2C *_i2c, uint8_t address = DS2482_DEFAULT_ADDRESS);
	inline bool present() {return present_;}
	inline uint8_t getError() {return error_;}
	
	void deviceReset();
	void setReadPointer(uint8_t readPointer);
  uint8_t readStatus();
  uint8_t readData();
  uint8_t waitOnBusy();
  uint8_t readConfig();
  void writeConfig(uint8_t config);
  void setStrongPullup();
  void clearStrongPullup();
  uint8_t wireReset();
  void wireWriteByte(uint8_t data, uint8_t power = 0);
  uint8_t wireReadByte();
  void wireWriteBit(uint8_t data, uint8_t power = 0);
  uint8_t wireReadBit();
  void wireSkip();
  void wireSelect(const uint8_t rom[8]);
  void wireResetSearch();
  uint8_t wireSearch(uint8_t *address);
};

#endif // DS2482_H
