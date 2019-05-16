#include "ds2482.h"

#include "printf.h"

#define DS2482_COMMAND_RESET		0xF0	// Device reset

#define DS2482_COMMAND_SRP			0xE1 	// Set read pointer
	#define DS2482_POINTER_STATUS		0xF0
		#define DS2482_STATUS_BUSY			(1<<0)
		#define DS2482_STATUS_PPD			(1<<1)
		#define DS2482_STATUS_SD			(1<<2)
		#define DS2482_STATUS_LL			(1<<3)
		#define DS2482_STATUS_RST 			(1<<4)
		#define DS2482_STATUS_SBR			(1<<5)
		#define DS2482_STATUS_TSB 			(1<<6)
		#define DS2482_STATUS_DIR 			(1<<7)
	#define DS2482_POINTER_DATA			0xE1
	#define DS2482_POINTER_CONFIG		0xC3
		#define DS2482_CONFIG_APU			(1<<0)
		#define DS2482_CONFIG_SPU			(1<<2)
		#define DS2482_CONFIG_1WS			(1<<3)


#define DS2482_COMMAND_WRITECONFIG	0xD2
#define DS2482_COMMAND_RESETWIRE	0xB4
#define DS2482_COMMAND_WRITEBYTE	0xA5
#define DS2482_COMMAND_READBYTE		0x96
#define DS2482_COMMAND_SINGLEBIT	0x87
#define DS2482_COMMAND_TRIPLET		0x78

#define WIRE_COMMAND_SKIP			0xCC
#define WIRE_COMMAND_SELECT			0x55
#define WIRE_COMMAND_SEARCH			0xF0


bool DS2482::init(I2C *_i2c, uint8_t address)
{
	i2c_ = _i2c;
	address_ = address;
	
	if(i2c_->write(address, 0xFF, DS2482_COMMAND_RESET) == I2C::RESULT_SUCCESS)
  {
    present_ = true;
	}else{
		present_ = false;
	}
	
	error_ = 0;
	return present_;
}

void DS2482::deviceReset()
{
	i2c_->write(address_, 0xFF, DS2482_COMMAND_RESET);
}


void DS2482::setReadPointer(uint8_t readPointer)
{
	i2c_->write(address_, DS2482_COMMAND_SRP, readPointer);
}

uint8_t DS2482::readStatus()
{
	setReadPointer(DS2482_POINTER_STATUS);
	uint8_t status = 0xFF;

	i2c_->read(address_, 0xFF, &status);
	return status;
}

uint8_t DS2482::readData()
{
	setReadPointer(DS2482_POINTER_DATA);
	uint8_t data;
	i2c_->read(address_, 0xFF, &data);
	return data;
}

uint8_t DS2482::readConfig()
{
	setReadPointer(DS2482_POINTER_CONFIG);
	uint8_t config;
	i2c_->read(address_, 0xFF, &config);
	return config;
}

void DS2482::setStrongPullup()
{
	writeConfig(readConfig() | DS2482_CONFIG_SPU);
}

void DS2482::clearStrongPullup()
{
	writeConfig(readConfig() & !DS2482_CONFIG_SPU);
}

uint8_t DS2482::waitOnBusy()
{
	uint8_t status;

	for(int i=1000; i>0; i--)
	{
		status = readStatus();
		if (!(status & DS2482_STATUS_BUSY))
			break;
		delay(20);
	}

	// if we have reached this point and we are still busy, there is an error
	if (status & DS2482_STATUS_BUSY)
		error_ = DS2482_ERROR_TIMEOUT;

	// Return the status so we don't need to explicitly do it again
	return status;
}

void DS2482::writeConfig(uint8_t config)
{
	waitOnBusy();
	i2c_->write(address_, DS2482_COMMAND_WRITECONFIG, config | (~config)<<4);
	uint8_t read_config;
	i2c_->read(address_, 0xFF, &read_config);
	if( read_config != config)
	{
		error_ = DS2482_ERROR_CONFIG;
	}
}

uint8_t DS2482::wireReset()
{
	waitOnBusy();
	// Datasheet warns that reset with SPU set can exceed max ratings
	clearStrongPullup();
	
	waitOnBusy();
	
	i2c_->write(address_, 0xFF, DS2482_COMMAND_RESETWIRE);

	uint8_t status = waitOnBusy();
	
	if (status & DS2482_STATUS_SD)
	{
		error_ = DS2482_ERROR_SHORT;
	}
							
	return (status & DS2482_STATUS_PPD) ? true : false;
}

void DS2482::wireWriteByte(uint8_t data, uint8_t power)
{
	waitOnBusy();
	if (power)
		setStrongPullup();
	i2c_->write(address_, DS2482_COMMAND_WRITEBYTE, data);
}

uint8_t DS2482::wireReadByte()
{
	waitOnBusy();
	i2c_->write(address_, 0xFF, DS2482_COMMAND_READBYTE);
	waitOnBusy();
	return readData();
}

void DS2482::wireWriteBit(uint8_t data, uint8_t power)
{
	waitOnBusy();
	if (power)
		setStrongPullup();
	
	i2c_->write(address_, DS2482_COMMAND_SINGLEBIT, data ? 0x80 : 0x00);
}

uint8_t DS2482::wireReadBit()
{
	wireWriteBit(1);
	uint8_t status = waitOnBusy();
	return status & DS2482_STATUS_SBR ? 1 : 0;
}

void DS2482::wireSkip()
{
	wireWriteByte(WIRE_COMMAND_SKIP);
}

void DS2482::wireSelect(const uint8_t rom[8])
{
	wireWriteByte(WIRE_COMMAND_SELECT);
	for (int i=0;i<8;i++)
		wireWriteByte(rom[i]);
}

void DS2482::wireResetSearch()
{
	searchLastDiscrepancy = 0;
	searchLastDeviceFlag = 0;

	for (int i = 0; i < 8; i++)
	{
		searchAddress[i] = 0;
	}
}

uint8_t DS2482::wireSearch(uint8_t *address)
{
	uint8_t direction;
	uint8_t last_zero=0;
	
	if (searchLastDeviceFlag)
		return 0;
	
	if (!wireReset())
		return 0;
	
	waitOnBusy();
	
	wireWriteByte(WIRE_COMMAND_SEARCH);
	
	for(uint8_t i=0;i<64;i++)
	{
		int searchByte = i / 8; 
		int searchBit = 1 << i % 8;
		
		if (i < searchLastDiscrepancy)
			direction = searchAddress[searchByte] & searchBit;
		else
			direction = i == searchLastDiscrepancy;
		
		waitOnBusy();
		i2c_->write(address_, DS2482_COMMAND_TRIPLET, direction ? 0x80 : 0x00);
		uint8_t status = waitOnBusy();

		uint8_t id = status & DS2482_STATUS_SBR;
		uint8_t comp_id = status & DS2482_STATUS_TSB;
		direction = status & DS2482_STATUS_DIR;

		if (id && comp_id)
		{
			return 0;
		}
		else
		{
			if (!id && !comp_id && !direction)
			{
				last_zero = i;
			}
		}

		if (direction)
			searchAddress[searchByte] |= searchBit;
		else
			searchAddress[searchByte] &= ~searchBit;

	}

	searchLastDiscrepancy = last_zero;

	if (!last_zero)
		searchLastDeviceFlag = 1;

	for (uint8_t i=0; i<8; i++)
		address[i] = searchAddress[i];
	
	return 1;
}
	
