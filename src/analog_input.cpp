/*
 * Copyright (c) 2019, Amaury Negre
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "analog_input.h"
#include "adc.h"

#include "printf.h"

static DRV_ADC DRV_ADC1;
static DRV_ADC DRV_ADC2;
static DRV_ADC DRV_ADC3;

void AnalogInput::init(ADC_TypeDef* ADCx, uint8_t channel, GPIO_TypeDef* gpio_port, uint16_t pin)
{
	adc_dev_ = ADCx;
	channel_ = channel;
	
	if(adc_dev_==ADC1)
	{
		DRV_ADC1.init(ADC1);
	}else if(adc_dev_==ADC2)
	{
		DRV_ADC2.init(ADC2);
	}else if(adc_dev_==ADC3)
	{
		DRV_ADC3.init(ADC3);
	}
	gpio_.init(gpio_port, pin, GPIO::ANALOG);

}

uint16_t AnalogInput::read()
{
	uint32_t timeout = 0xFFF;
	
	ADC_RegularChannelConfig(adc_dev_, channel_, 1, ADC_SampleTime_3Cycles);
	
	// Start the conversion
  ADC_SoftwareStartConv(adc_dev_);
		
  // Wait until conversion completion
  while(ADC_GetFlagStatus(adc_dev_, ADC_FLAG_EOC) == RESET)
	{
		if(timeout--==0x00){
			return 0;
		}
	}
  // Get the conversion value
  return ADC_GetConversionValue(adc_dev_);
}
