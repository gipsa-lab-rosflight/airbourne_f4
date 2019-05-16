#ifndef VL53L0X_H
#define VL53L0X_H

#include <cstdint>

#include "i2c.h"

#define VL53L0X_ADDRESS_DEFAULT 0x29

enum VL53L0X_REGISTER_ADDRESSES {
	SYSRANGE_START = 0x00,

	SYSTEM_THRESH_HIGH = 0x0C,
	SYSTEM_THRESH_LOW = 0x0E,

	SYSTEM_SEQUENCE_CONFIG = 0x01,
	SYSTEM_RANGE_CONFIG = 0x09,
	SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,

	SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,

	GPIO_HV_MUX_ACTIVE_HIGH = 0x84,

	SYSTEM_INTERRUPT_CLEAR = 0x0B,

	RESULT_INTERRUPT_STATUS = 0x13,
	RESULT_RANGE_STATUS = 0x14,

	RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
	RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0,
	RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
	RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4,
	RESULT_PEAK_SIGNAL_RATE_REF = 0xB6,

	ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,

	I2C_SLAVE_DEVICE_ADDRESS = 0x8A,

	MSRC_CONFIG_CONTROL = 0x60,

	PRE_RANGE_CONFIG_MIN_SNR = 0x27,
	PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
	PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
	PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,

	FINAL_RANGE_CONFIG_MIN_SNR = 0x67,
	FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
	FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
	FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

	PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
	PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,

	PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
	PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
	PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,

	SYSTEM_HISTOGRAM_BIN = 0x81,
	HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
	HISTOGRAM_CONFIG_READOUT_CTRL = 0x55,

	FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
	FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
	FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
	CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,

	MSRC_CONFIG_TIMEOUT_MACROP = 0x46,

	SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
	IDENTIFICATION_MODEL_ID = 0xC0,
	IDENTIFICATION_REVISION_ID = 0xC2,

	OSC_CALIBRATE_VAL = 0xF8,

	GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,

	GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
	DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
	DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
	POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,

	VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,

	ALGO_PHASECAL_LIM = 0x30,
	ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
};

enum vl53l0xVcselPeriodType {
	VcselPeriodPreRange,
	VcselPeriodFinalRange
};

struct VL53L0XSequenceStepEnables {
	// TCC: Target CentreCheck
	bool tcc;
	// MSRC: Minimum Signal Rate Check
	bool msrc;
	// DSS: Dynamic Spad Selection
	bool dss;
	bool preRange;
	bool finalRange;
};

struct VL53L0XSequenceStepTimeouts {
	uint16_t preRangeVCSELPeriodPCLKs;
	uint16_t finalRangeVCSELPeriodPCLKs;

	uint16_t msrcDssTccMCLKs;
	uint16_t preRangeMCLKs;
	uint16_t finalRangeMCLKs;

	uint32_t msrcDssTccMicroseconds;
	uint32_t preRangeMicroseconds;
	uint32_t finalRangeMicroseconds;
};

class VL53L0X
{
private:
	bool sensor_present_;
	I2C *i2c_;
	uint8_t address_;
	uint32_t measurementTimingBudgetMicroseconds_;
  uint32_t timeoutStartMilliseconds_;
  uint32_t ioTimeout_;
  bool didTimeout_;
	uint8_t stopVariable_;

	static uint8_t rangeBuf_[2];
	static void(*cb_)(uint16_t, bool);
	
public:
	VL53L0X();
	bool init(I2C *_i2c, uint8_t address=VL53L0X_ADDRESS_DEFAULT);
	
	void setAddress(uint8_t newAddress);

	inline uint8_t getAddress() {
    return address_;
  }
	
	bool setSignalRateLimit(float limitMCPS);
	float getSignalRateLimit();
	bool setMeasurementTimingBudget(uint32_t budgetMicroseconds);
	uint32_t getMeasurementTimingBudget();
	
	bool setVcselPulsePeriod(vl53l0xVcselPeriodType type, uint8_t periodPCLKs);
	uint8_t getVcselPulsePeriod(vl53l0xVcselPeriodType type);
	
	void startContinuous(uint32_t periodMilliseconds = 0);
	void stopContinuous();

	uint16_t readRangeContinuousMillimeters(bool blocking = true);
	uint16_t readRangeSingleMillimeters();
	
	void asyncReadRangeContinuous(void(*callback)(uint16_t, bool));
	
	inline void setTimeout(uint32_t timeout) {
		ioTimeout_ = timeout;
  }

	inline uint32_t getTimeout() {
    return ioTimeout_;
  }
	
	bool timeoutOccurred();
	
private:
	void initHardware();
	
	bool getSPADInfo(uint8_t* count, bool* typeIsAperture);
	void getSequenceStepEnables(VL53L0XSequenceStepEnables* enables);
	void getSequenceStepTimeouts(const VL53L0XSequenceStepEnables* enables, VL53L0XSequenceStepTimeouts* timeouts);
	static uint16_t decodeTimeout(uint16_t registerValue);
	static uint16_t encodeTimeout(uint16_t timeoutMCLKs);
	static uint32_t timeoutMclksToMicroseconds(uint16_t timeoutPeriodMCLKs, uint8_t vcselPeriodPCLKs);
	static uint32_t timeoutMicrosecondsToMclks(uint32_t timeoutPeriodMicroseconds, uint8_t vcselPeriodPCLKs);
	
	bool performSingleRefCalibration(uint8_t vhvInitByte);

	static void rangeCallback(uint8_t);
	
	/**
   * Write an 8-bit register.
   */
  void writeRegister(uint8_t register, uint8_t value);
  /**
   * Write a 16-bit register.
   */
  void writeRegister16Bit(uint8_t register, uint16_t value);
  /**
   * Write a 32-bit register.
   *
   * Based on VL53L0X_write_dword from VL53L0X kernel driver.
   */
  void writeRegister32Bit(uint8_t register, uint32_t value);
  /**
   * Write an arbitrary number of bytes from the given array to the sensor, starting at the given register.
   */
  void writeRegisterMultiple(uint8_t register, const uint8_t* source, uint8_t count);
  /**
   * Read an 8-bit register.
   */
  uint8_t readRegister(uint8_t register);
  /**
   * Read a 16-bit register.
   */
  uint16_t readRegister16Bit(uint8_t register);
  /**
   * Read a 32-bit register.
   */
  uint32_t readRegister32Bit(uint8_t register);
  /**
   * Read an arbitrary number of bytes from the sensor, starting at the given register, into the given array.
   */
  void readRegisterMultiple(uint8_t register, uint8_t* destination, uint8_t count);

};

#endif // VL53L0X_H
