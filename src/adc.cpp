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

#include "adc.h"

DRV_ADC::DRV_ADC()
	: dev_(NULL)
{
}

void DRV_ADC::init( ADC_TypeDef* dev)
{
	if(dev == dev_)
	{
		return;
	}
	
	dev_ = dev;
	
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	ADC_InitTypeDef ADC_InitStructure;

	//ADC_DeInit();
	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ExternalTrigConv = DISABLE; //ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;

	if (dev_ == ADC1) {
		/* Enable ADC clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	}else if (dev_ == ADC2) {
		/* Enable ADC clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	}else if (dev_ == ADC3) {
		/* Enable ADC clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	}
	
  /* Set common ADC settings */
 	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
 	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
 	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
 	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
 	ADC_CommonInit(&ADC_CommonInitStruct);
	
	/* Now do the setup */
	ADC_Init(dev_, &ADC_InitStructure);
  /* Enable ADC */
  ADC_Cmd(dev_, ENABLE);
	
}
