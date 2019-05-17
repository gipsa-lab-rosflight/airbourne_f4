#ifndef MULTI_RANGE_H
#define MULTI_RANGE_H

#include "vl53l0x.h"
#include "ds2482.h"
#include "i2c.h"

#define MULTI_RANGE_MAX_SENSORS 16
#define MULTIRANGE_UPDATE_WAIT_MILLIS 20

class VL53L0X_Ext : public VL53L0X
{
public:
	uint8_t rom[8]; // 1W address
};

class MultiRange
{
private:
	VL53L0X_Ext sensors_[MULTI_RANGE_MAX_SENSORS];
	uint16_t ranges_[MULTI_RANGE_MAX_SENSORS]; // range data (mm)
	
	DS2482 oneWire_;
	uint8_t nbSensors_;
	bool sensor_present_;
	uint32_t last_update_ms_;
	uint8_t cur_sensor_;
	bool is_updating_;
	I2C *i2c_;
	bool new_data_;
	
public:
	MultiRange();
	bool init(I2C *i2c, uint8_t base_address=0x50);
	bool present();

	void update();

	inline bool has_new_data(){ return new_data_; }
	
	void range_callback(uint16_t result, bool success);

	inline uint8_t getNbSensors() {return nbSensors_;}
	inline uint16_t getRange(uint8_t k) {return ranges_[k];}
	
	void read(uint16_t *ranges);
};

#endif // MULTI_RANGE_H
