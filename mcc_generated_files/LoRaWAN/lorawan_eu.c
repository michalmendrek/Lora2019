/********************************************************************
 * Copyright (C) 2016 Microchip Technology Inc. and its subsidiaries
 * (Microchip).  All rights reserved.
 *
 * You are permitted to use the software and its derivatives with Microchip
 * products. See the license agreement accompanying this software, if any, for
 * more info about your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
 * MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP, SMSC, OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH
 * OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY FOR ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT OR CONSEQUENTIAL DAMAGES, OR OTHER SIMILAR COSTS. To the fullest
 * extend allowed by law, Microchip and its licensors liability will not exceed
 * the amount of fees, if any, that you paid directly to Microchip to use this
 * software.
 *************************************************************************
 *
 *                           lorawan_eu.c
 *
 * LoRaWAN EU file
 *
 ******************************************************************************/

/****************************** INCLUDES **************************************/
#include <math.h>
#include <stdbool.h>
#include "lorawan.h"
#include "lorawan_aes.h"
#include "lorawan_aes_cmac.h"
#include "lorawan_private.h"
#include "lorawan_defs.h"
#include "AES.h"
#include "radio_interface.h"
#include "sw_timer.h"
#include "lorawan_eu.h"


/****************************** VARIABLES *************************************/
const uint8_t rxWindowSize[] = {8, 10, 14, 26, 49, 88, 60, 8};

// Max Payload Size 
const uint8_t LoRa_maxPayloadSize[8] = {51, 51, 51, 115, 242, 242, 242, 56}; // for FSK max message size should be 64 bytes

// Channels by ism band
ChannelParams_t LoRa_Channels[MAX_EU_SINGLE_BAND_CHANNELS];

static const int8_t rxWindowOffset[] = {-33, -50, -58, -62, -66, -68, -15, -2};

// Tx power possibilities by ism band
static const int8_t txPower868[] = {20, 14, 11, 8, 5, 2};

static const int8_t txPower433[] = {10, 7, 4, 1, -2, -5};

// Spreading factor possibilities 
static const uint8_t spreadingFactor[] = {12, 11, 10, 9, 8, 7, 7};

// Bandwidth possibilities 
static const uint8_t bandwidth[] = {BW_125KHZ, BW_125KHZ, BW_125KHZ, BW_125KHZ, BW_125KHZ, BW_125KHZ, BW_250KHZ};

// Modulation possibilities 
static const uint8_t modulation[] = {MODULATION_LORA, MODULATION_LORA, MODULATION_LORA, MODULATION_LORA, MODULATION_LORA, MODULATION_LORA, MODULATION_LORA, MODULATION_FSK};

static const ChannelParams_t DefaultChannels868[] = {
                                                     LC0_868,
                                                     LC1_868,
                                                     LC2_868,
};

static const ChannelParams_t DefaultChannels433[] = {
                                                     LC0_433,
                                                     LC1_433,
                                                     LC2_433,
};

static const uint8_t FskSyncWordBuff[3] = {0xC1, 0x94, 0xC1};

/************************ PRIVATE FUNCTION PROTOTYPES *************************/

static void LoRa_CreateSoftwareTimers(void);
static void LoRa_SetCallbackSoftwareTimers(void);
static void LoRa_StopAllSoftwareTimers(void);
void LoRa_Reset(IsmBand_t ismBandNew);
static void LoRa_InitDefault868Channels(void);
static void LoRa_InitDefault433Channels(void);

static void CreateAllSoftwareTimers(void);

static void SetCallbackSoftwareTimers(void);

static void StopAllSoftwareTimers(void);

static void InitDefault868Channels(void);

static void InitDefault433Channels(void);

static void LoRa_UpdateDataRange(uint8_t channelId, uint8_t dataRangeNew);

static void LoRa_UpdateChannelIdStatus(uint8_t channelId, bool statusNew);

static LorawanError_t ValidateRxOffset(uint8_t rxOffset);

static LorawanError_t LoRa_ValidateFrequency(uint32_t frequencyNew);

static LorawanError_t ValidateDataRange(uint8_t dataRangeNew);

static LorawanError_t ValidateChannelId(uint8_t channelId, bool allowedForDefaultChannels);

static LorawanError_t ValidateChannelMaskCntl(uint8_t channelMaskCntl);

static void EnableChannels(uint16_t channelMask, uint8_t channelMaskCntl);

static void UpdateFrequency(uint8_t channelId, uint32_t frequencyNew);

static void UpdateDutyCycle(uint8_t channelId, uint16_t dutyCycleNew);

static LorawanError_t ValidateChannelMask(uint16_t channelMask);

static void EnableChannels1(uint16_t channelMask, uint8_t channelMaskCntl, uint8_t channelIndexMin, uint8_t channelIndexMax);

void LoRa_ConfigureRadioTx(uint8_t dataRate, uint32_t freq);

/****************************** FUNCTIONS *************************************/

void LoRa_System_Init(void) // this function resets everything to the default values
{
  // Allocate software timers and their callbacks
  loRa.LoRa_transmitStatus = LoRa_Idle;
  loRa.LoRa_StatusDanych = LoRa_transmitIdle;
  loRa.LoRa_Counnter.value = 0;

  if(loRa.LoRa_initialised == DISABLED)
    {
      //      CreateAllSoftwareTimers();
      LoRa_CreateSoftwareTimers();
      LoRa_SetCallbackSoftwareTimers();
      loRa.LoRa_initialised = ENABLED;
    }
  else
    {
      LoRa_StopAllSoftwareTimers();
    }
  loRa.LoRa_Addres = LoRaDeviceAddress;
  loRa.LoRa_ch0_params.frequency = LoRa_CH0_frequency;
  loRa.LoRa_ch0_params.dataRate = LoRa_CH0_datarate;
  //  loRa.LoRa_maxChannels = MAX_EU_SINGLE_BAND_CHANNELS;

  RADIO_Init(LoRa_radioBuffer, EU868_CALIBRATION_FREQ);

  srand(RADIO_ReadRandom()); // for the loRa random function we need a seed that is obtained from the radio

  LoRa_Reset(ISM_EU868);

}

static void LoRa_CreateSoftwareTimers(void)
{
  loRa.LoRa_TimerHandshaking = SwTimerCreate();
  loRa.LoRa_TimerRetransmit = SwTimerCreate();
  loRa.LoRa_TimerWaitAck = SwTimerCreate();
}

static void LoRa_SetCallbackSoftwareTimers(void)
{
  SwTimerSetCallback(loRa.LoRa_TimerHandshaking, LoRa_TimerHandshakingCallback, 0);
  SwTimerSetCallback(loRa.LoRa_TimerRetransmit, LoRa_TimerRetransmitCallback, 0);
  SwTimerSetCallback(loRa.LoRa_TimerWaitAck, LoRa_TimerWaitAckCallback, 0);
}

static void LoRa_StopAllSoftwareTimers(void)
{
   SwTimerStop(loRa.LoRa_TimerHandshaking);
  SwTimerStop(loRa.LoRa_TimerRetransmit);
  SwTimerStop(loRa.LoRa_TimerWaitAck);
}

void LoRa_Reset(IsmBand_t ismBandNew)
{
  if(loRa.LoRa_initialised == ENABLED)
    {
      LoRa_StopAllSoftwareTimers();
    }

  loRa.LoRa_syncWord = 0x34;
  RADIO_SetLoRaSyncWord(loRa.LoRa_syncWord);

  loRa.LoRa_batteryLevel = BATTERY_LEVEL_INVALID; // the end device was not able to measure the battery level

  loRa.LoRa_ismBand = ismBandNew;

  // initialize default channels
  loRa.LoRa_maxChannels = MAX_EU_SINGLE_BAND_CHANNELS;
  if(ISM_EU868 == ismBandNew)
    {
      RADIO_Init(LoRa_radioBuffer, EU868_CALIBRATION_FREQ);

      LoRa_InitDefault868Channels();
    }
  else
    {
      RADIO_Init(LoRa_radioBuffer, EU433_CALIBRATION_FREQ);

      LoRa_InitDefault433Channels();
    }

  loRa.LoRa_txPower = 1;

  loRa.LoRa_currentDataRate = DR0;

  LoRa_UpdateMinMaxChDataRate();
}

static void LoRa_InitDefault868Channels(void)
{
  uint8_t i;

  memset(LoRa_Channels, 0, sizeof(LoRa_Channels));
  memcpy(LoRa_Channels, DefaultChannels868, sizeof(DefaultChannels868));
  for(i = 3; i < MAX_EU_SINGLE_BAND_CHANNELS; i++)
    {
      // for undefined channels the duty cycle should be a very big value, and the data range a not-valid value
      //duty cycle 0 means no duty cycle limitation, the bigger the duty cycle value, the greater the limitation
      LoRa_Channels[i].dutyCycle = UINT16_MAX;
      LoRa_Channels[i].dataRange.value = UINT8_MAX;
    }
}

static void LoRa_InitDefault433Channels(void)
{
  uint8_t i;

  memset(LoRa_Channels, 0, sizeof(LoRa_Channels));
  memcpy(LoRa_Channels, DefaultChannels433, sizeof(DefaultChannels433));
  for(i = 3; i < MAX_EU_SINGLE_BAND_CHANNELS; i++)
    {
      // for undefined channels the duty cycle should be a very big value, and the data range a not-valid value
      //duty cycle 0 means no duty cycle limitation, the bigger the duty cycle value, the greater the limitation
      LoRa_Channels[i].dutyCycle = UINT16_MAX;
      LoRa_Channels[i].dataRange.value = UINT8_MAX;
    }
}

void LoRa_TxDone(uint16_t timeOnAir)
{
  if(loRa.LoRa_transmitStatus == LoRa_Handshaking_TX)
    {
      loRa.LoRa_transmitStatus = LoRa_Handshaking_RX;
      LoRa_EnterReceive();
    }
  if(loRa.LoRa_transmitStatus == LoRa_SendData_TX)
    {
      loRa.LoRa_transmitStatus = LoRa_SendData_RX;
      LoRa_EnterReceive();
    }
}

void LoRa_RxTimeout(void)
{
  if(loRa.LoRa_transmitStatus == LoRa_Handshaking_RX)
    {
      SwTimerStop(loRa.LoRa_TimerHandshaking);
      loRa.LoRa_transmitStatus = LoRa_SendFailed;
    }
  if(loRa.LoRa_transmitStatus == LoRa_SendData_RX)
    {
      SwTimerStop(loRa.LoRa_TimerWaitAck);
      loRa.LoRa_transmitStatus = LoRa_SendFailed;
    }
  RADIO_clearReceiveFlag();
}

void LORAWAN_Init(RxAppDataCb_t RxPayload, RxJoinResponseCb_t RxJoinResponse) // this function resets everything to the default values
{
  // Allocate software timers and their callbacks
  if(loRa.macInitialized == DISABLED)
    {
      CreateAllSoftwareTimers();
      SetCallbackSoftwareTimers();
      loRa.macInitialized = ENABLED;
    }
  else
    {
      StopAllSoftwareTimers();
    }

  rxPayload.RxAppData = RxPayload;
  rxPayload.RxJoinResponse = RxJoinResponse;

  RADIO_Init(&LoRa_radioBuffer[16], EU868_CALIBRATION_FREQ);

  srand(RADIO_ReadRandom()); // for the loRa random function we need a seed that is obtained from the radio

  LORAWAN_Reset(ISM_EU868);

}

void LORAWAN_Reset(IsmBand_t ismBandNew)
{
  
}

LorawanError_t LORAWAN_SetReceiveWindow2Parameters(uint32_t frequency, uint8_t dataRate)
{
 
}

uint32_t LoRa_GetFrequency(uint8_t channelId)
{
  return LoRa_Channels[channelId].frequency;
}

LorawanError_t LoRa_SetDataRange(uint8_t channelId, uint8_t dataRangeNew)
{
  LorawanError_t result = OK;

  if((ValidateChannelId(channelId, ALL_CHANNELS) != OK) || (ValidateDataRange(dataRangeNew) != OK))
    {
      result = INVALID_PARAMETER;
    }
  else
    {
      LoRa_UpdateDataRange(channelId, dataRangeNew);
    }

  return result;
}

uint8_t LORAWAN_GetDataRange(uint8_t channelId)
{
  uint8_t result = 0xFF;

  if(ValidateChannelId(channelId, ALL_CHANNELS) == OK)
    {
      result = LoRa_Channels[channelId].dataRange.value;
    }
  return result;
}

LorawanError_t LORAWAN_SetChannelIdStatus(uint8_t channelId, bool statusNew)
{
  LorawanError_t result = OK;


  if(ValidateChannelId(channelId, ALL_CHANNELS) != OK)
    {
      result = INVALID_PARAMETER;
    }

  else
    {
      if((LoRa_Channels[channelId].parametersDefined & (FREQUENCY_DEFINED | DATA_RANGE_DEFINED | DUTY_CYCLE_DEFINED)) == (FREQUENCY_DEFINED | DATA_RANGE_DEFINED | DUTY_CYCLE_DEFINED))
        {
          LoRa_UpdateChannelIdStatus(channelId, statusNew);
        }
      else
        {
          result = INVALID_PARAMETER;
        }
    }

  return result;
}

bool LORAWAN_GetChannelIdStatus(uint8_t channelId)
{
  bool result = DISABLED;

  if(ValidateChannelId(channelId, ALL_CHANNELS) == OK)
    {
      result = LoRa_Channels[channelId].status;
    }
  return result;
}

LorawanError_t LORAWAN_SetFrequency(uint8_t channelId, uint32_t frequencyNew)
{
  LorawanError_t result = OK;

  if((ValidateChannelId(channelId, 0) != OK) || (LoRa_ValidateFrequency(frequencyNew) != OK))
    {
      return INVALID_PARAMETER;
    }

  UpdateFrequency(channelId, frequencyNew);

  return result;
}

LorawanError_t LORAWAN_SetDutyCycle(uint8_t channelId, uint16_t dutyCycleValue)
{
  LorawanError_t result = OK;

  if(ValidateChannelId(channelId, ALL_CHANNELS) == OK)
    {
      UpdateDutyCycle(channelId, dutyCycleValue);
    }
  else
    {
      result = INVALID_PARAMETER;
    }

  return result;
}

uint16_t LORAWAN_GetDutyCycle(uint8_t channelId)
{
  uint16_t result = UINT16_MAX;

  if(ValidateChannelId(channelId, ALL_CHANNELS) == OK)
    {
      result = LoRa_Channels[channelId].dutyCycle;
    }

  return result;
}

uint8_t LoRa_GetIsmBand(void) //returns the ISM band
{
  return loRa.LoRa_ismBand;
}

void LORAWAN_TxDone(uint16_t timeOnAir)
{
   
}

// this function is called by the radio when the first or the second receive window expired without receiving any message (either for join accept or for message)

void LORAWAN_RxTimeout(void)
{
  
}

LorawanError_t ValidateDataRate(uint8_t dataRate)
{
  LorawanError_t result = OK;

  if(dataRate > DR7)
    {
      result = INVALID_PARAMETER;
    }

  return result;
}

LorawanError_t LoRa_ValidateTxPower(uint8_t txPowerNew)
{
  LorawanError_t result = OK;

  if(((ISM_EU868 == loRa.LoRa_ismBand) && (0 == txPowerNew)) || (txPowerNew > 5))
    {
      result = INVALID_PARAMETER;
    }

  return result;
}

uint8_t* ExecuteDutyCycle(uint8_t *ptr)
{
  
}

uint8_t* ExecuteLinkAdr(uint8_t *ptr)
{
  
}

uint8_t* ExecuteDevStatus(uint8_t *ptr)
{
  return ptr;
}

uint8_t* ExecuteNewChannel(uint8_t *ptr)
{
 
}

uint8_t* ExecuteRxParamSetupReq(uint8_t *ptr)
{
  
}

LorawanError_t SearchAvailableChannel(uint8_t maxChannels, bool transmissionType, uint8_t* channelIndex)
{
    
}

void UpdateCfList(uint8_t bufferLength, JoinAccept_t *joinAccept)
{
  uint8_t i;
  uint32_t frequency;
  uint8_t channelIndex;

  if((bufferLength == SIZE_JOIN_ACCEPT_WITH_CFLIST))
    {
      // 3 is the minimum channel index for single band
      channelIndex = 3;

      for(i = 0; i < NUMBER_CFLIST_FREQUENCIES; i++)
        {
          frequency = 0;
          memcpy(&frequency, joinAccept->members.cfList + 3 * i, 3);
          frequency *= 100;
          if(frequency != 0)
            {
              if(LoRa_ValidateFrequency(frequency) == OK)
                {
                  LoRa_Channels[i + channelIndex].frequency = frequency;
                  LoRa_Channels[i + channelIndex].dataRange.max = DR5;
                  LoRa_Channels[i + channelIndex].dataRange.min = DR0;
                  LoRa_Channels[i + channelIndex].dutyCycle = DUTY_CYCLE_DEFAULT_NEW_CHANNEL;
                  LoRa_Channels[i + channelIndex].parametersDefined = 0xFF; //all parameters defined
                  LORAWAN_SetChannelIdStatus(i + channelIndex, ENABLED);
                  loRa.macStatus.channelsModified = ENABLED; // a new channel was added, so the flag is set to inform the user
                }
            }
          else
            {
              LORAWAN_SetChannelIdStatus(i + channelIndex, DISABLED);
            }
        }

      loRa.macStatus.channelsModified = ENABLED;
    }
}

void ConfigureRadio(uint8_t dataRate, uint32_t freq)
{
  RADIO_SetModulation(modulation[dataRate]);
  RADIO_SetChannelFrequency(freq);
  RADIO_SetFrequencyHopPeriod(DISABLED);

  if(dataRate <= DR6)
    {
      //LoRa modulation
      RADIO_SetSpreadingFactor(spreadingFactor[dataRate]);
      RADIO_SetBandwidth(bandwidth[dataRate]);
      RADIO_SetLoRaSyncWord(loRa.LoRa_syncWord);
    }
  else
    {
      //FSK modulation
      RADIO_SetFSKSyncWord(sizeof(FskSyncWordBuff) / sizeof(FskSyncWordBuff[0]), (uint8_t*) FskSyncWordBuff);
    }
}

uint32_t GetRx1Freq(void)
{
  return loRa.receiveWindow1Parameters.frequency;
}

void UpdateDLSettings(uint8_t dlRx2Dr, uint8_t dlRx1DrOffset)
{

}

void StartReTxTimer(void)
{
 
}

LorawanError_t LoRa_SelectChannelForTransmission(uint8_t channelTx, uint8_t channelRx) // 
{
  LorawanError_t result = OK;
  uint8_t channelTxIndex = LoRa_Chann_nr;
  uint8_t channelRxIndex = LoRa_Chann_nr;

  if(channelTx <= LoRa_Chann_nr)
    {
      channelTxIndex = channelTx;
    }
  if(channelRx <= LoRa_Chann_nr)
    {
      channelRxIndex = channelRx;
    }

  loRa.LoRa_lastUsedChannelIndex = channelRxIndex;
  loRa.LoRa_receiveChannelParameters.frequency = LoRa_Channels[channelRxIndex].frequency;
  loRa.LoRa_receiveChannelParameters.dataRate = loRa.LoRa_currentDataRate;
  loRa.LoRa_sendChannelParameters.frequency = LoRa_Channels[channelRxIndex].frequency;
  loRa.LoRa_sendChannelParameters.dataRate = loRa.LoRa_currentDataRate;

  return result;
}

LorawanError_t SelectChannelForTransmission(bool transmissionType) // transmission type is 0 means join request, transmission type is 1 means data message mode
{
  
}

static void CreateAllSoftwareTimers(void)
{ 
  loRa.LoRa_TimerHandshaking = SwTimerCreate();
  loRa.LoRa_TimerRetransmit = SwTimerCreate();
  loRa.LoRa_TimerWaitAck = SwTimerCreate();
}

static void SetCallbackSoftwareTimers(void)
{
  SwTimerSetCallback(loRa.LoRa_TimerHandshaking, LoRa_TimerHandshakingCallback, 0);
  SwTimerSetCallback(loRa.LoRa_TimerRetransmit, LoRa_TimerRetransmitCallback, 0);
  SwTimerSetCallback(loRa.LoRa_TimerWaitAck, LoRa_TimerWaitAckCallback, 0);
}

static void StopAllSoftwareTimers(void)
{
  SwTimerStop(loRa.LoRa_TimerHandshaking);
  SwTimerStop(loRa.LoRa_TimerRetransmit);
  SwTimerStop(loRa.LoRa_TimerWaitAck);
}

static void InitDefault868Channels(void)
{
  uint8_t i;

  memset(LoRa_Channels, 0, sizeof(LoRa_Channels));
  memcpy(LoRa_Channels, DefaultChannels868, sizeof(DefaultChannels868));
  for(i = 3; i < MAX_EU_SINGLE_BAND_CHANNELS; i++)
    {
      // for undefined channels the duty cycle should be a very big value, and the data range a not-valid value
      //duty cycle 0 means no duty cycle limitation, the bigger the duty cycle value, the greater the limitation
      LoRa_Channels[i].dutyCycle = UINT16_MAX;
      LoRa_Channels[i].dataRange.value = UINT8_MAX;
    }
}

static void InitDefault433Channels(void)
{
  uint8_t i;

  memset(LoRa_Channels, 0, sizeof(LoRa_Channels));
  memcpy(LoRa_Channels, DefaultChannels433, sizeof(DefaultChannels433));
  for(i = 3; i < MAX_EU_SINGLE_BAND_CHANNELS; i++)
    {
      // for undefined channels the duty cycle should be a very big value, and the data range a not-valid value
      //duty cycle 0 means no duty cycle limitation, the bigger the duty cycle value, the greater the limitation
      LoRa_Channels[i].dutyCycle = UINT16_MAX;
      LoRa_Channels[i].dataRange.value = UINT8_MAX;
    }
}

static void LoRa_UpdateDataRange(uint8_t channelId, uint8_t dataRangeNew)
{
  uint8_t i;
  // after updating the data range of a channel we need to check if the minimum dataRange has changed or not.
  // The user cannot set the current data rate outside the range of the data range
  loRa.LoRa_minDataRate = DR7;
  loRa.LoRa_maxDataRate = DR0;

  LoRa_Channels[channelId].dataRange.value = dataRangeNew;
  LoRa_Channels[channelId].parametersDefined |= DATA_RANGE_DEFINED;
  for(i = 0; i < loRa.LoRa_maxChannels; i++)
    {
      if((LoRa_Channels[i].dataRange.min < loRa.LoRa_minDataRate) && (LoRa_Channels[i].status == ENABLED))
        {
          loRa.LoRa_minDataRate = LoRa_Channels[i].dataRange.min;
        }
      if((LoRa_Channels[i].dataRange.max > loRa.LoRa_maxDataRate) && (LoRa_Channels[i].status == ENABLED))
        {
          loRa.LoRa_maxDataRate = LoRa_Channels[i].dataRange.max;
        }
    }

  if(loRa.LoRa_currentDataRate > loRa.LoRa_maxDataRate)
    {
      loRa.LoRa_currentDataRate = loRa.LoRa_maxDataRate;
    }

  if(loRa.LoRa_currentDataRate < loRa.LoRa_minDataRate)
    {
      loRa.LoRa_currentDataRate = loRa.LoRa_minDataRate;
    }
}

static void LoRa_UpdateChannelIdStatus(uint8_t channelId, bool statusNew)
{
  uint8_t i;

  LoRa_Channels[channelId].status = statusNew;
  if(LoRa_Channels[channelId].status == DISABLED)
    {
      //Clear the dutycycle timer of the channel
      LoRa_Channels[channelId].channelTimer = 0;
    }

  for(i = 0; i < loRa.LoRa_maxChannels; i++)
    {
      if((LoRa_Channels[i].dataRange.min < loRa.LoRa_minDataRate) && (LoRa_Channels[i].status == ENABLED))
        {
          loRa.LoRa_minDataRate = LoRa_Channels[i].dataRange.min;
        }
      if((LoRa_Channels[i].dataRange.max > loRa.LoRa_maxDataRate) && (LoRa_Channels[i].status == ENABLED))
        {
          loRa.LoRa_maxDataRate = LoRa_Channels[i].dataRange.max;
        }
    }
}

static LorawanError_t ValidateRxOffset(uint8_t rxOffset)
{
  LorawanError_t result = OK;

  if(rxOffset > 5)
    {
      result = INVALID_PARAMETER;
    }

  return result;
}

static LorawanError_t LoRa_ValidateFrequency(uint32_t frequencyNew)
{
  LorawanError_t result = OK;

  if(ISM_EU868 == loRa.LoRa_ismBand)
    {
      if((frequencyNew > FREQ_870000KHZ) || (frequencyNew < FREQ_863000KHZ))
        {
          result = INVALID_PARAMETER;
        }
    }
  else
    {
      if((frequencyNew > FREQ_434790KHZ) || (frequencyNew < FREQ_433050KHZ))
        {
          result = INVALID_PARAMETER;
        }
    }

  return result;
}

static LorawanError_t ValidateDataRange(uint8_t dataRangeNew)
{
  LorawanError_t result = OK;
  uint8_t dataRateMax, dataRateMin;

  dataRateMin = dataRangeNew & LAST_NIBBLE;
  dataRateMax = (dataRangeNew & FIRST_NIBBLE) >> SHIFT4;

  if((ValidateDataRate(dataRateMax) != OK) || (ValidateDataRate(dataRateMin) != OK) || (dataRateMax < dataRateMin))
    {
      result = INVALID_PARAMETER;
    }
  return result;
}

static LorawanError_t ValidateChannelId(uint8_t channelId, bool allowedForDefaultChannels) //if allowedForDefaultChannels is 1, all the channels can be modified, if it is 0 channels 0, 1, 2 and 16, 17, and 18 (dual band) cannot be modified
{
  LorawanError_t result = OK;

  if((channelId >= MAX_EU_SINGLE_BAND_CHANNELS) || ((allowedForDefaultChannels == WITHOUT_DEFAULT_CHANNELS) && (channelId < 3)))
    {
      result = INVALID_PARAMETER;
    }

  return result;
}

static LorawanError_t ValidateChannelMaskCntl(uint8_t channelMaskCntl)
{
  LorawanError_t result = OK;

  if((channelMaskCntl != 0) && (channelMaskCntl != 6))
    {
      result = INVALID_PARAMETER;
    }

  return result;
}

static void EnableChannels(uint16_t channelMask, uint8_t channelMaskCntl)
{
  EnableChannels1(channelMask, channelMaskCntl, 0, MAX_EU_SINGLE_BAND_CHANNELS);
}

static void UpdateFrequency(uint8_t channelId, uint32_t frequencyNew)
{
  LoRa_Channels[channelId].frequency = frequencyNew;
  LoRa_Channels[channelId].parametersDefined |= FREQUENCY_DEFINED;
}

static void UpdateDutyCycle(uint8_t channelId, uint16_t dutyCycleNew)
{
  LoRa_Channels[channelId].dutyCycle = dutyCycleNew;
  LoRa_Channels[channelId].parametersDefined |= DUTY_CYCLE_DEFINED;
}

static LorawanError_t ValidateChannelMask(uint16_t channelMask)
{
  uint8_t i = 0;

  if(channelMask != 0x0000U)
    {
      for(i = 0; i < MAX_EU_SINGLE_BAND_CHANNELS; i++)
        {
          if(((channelMask & BIT0) == BIT0) && ((LoRa_Channels[i].parametersDefined & (FREQUENCY_DEFINED | DATA_RANGE_DEFINED | DUTY_CYCLE_DEFINED)) != (FREQUENCY_DEFINED | DATA_RANGE_DEFINED | DUTY_CYCLE_DEFINED))) // if the channel mask sent enables a yet undefined channel, the command is discarded and the device state is not changed
            {
              return INVALID_PARAMETER;
            }
          else
            {
              channelMask = channelMask >> SHIFT1;
            }
        }

      return OK;
    }
  else
    {
      //ChMask set to 0x0000 in ADR may be used as a DoS attack so receiving this results in an error
      return INVALID_PARAMETER;
    }
}

static void EnableChannels1(uint16_t channelMask, uint8_t channelMaskCntl, uint8_t channelIndexMin, uint8_t channelIndexMax)
{
  uint8_t i;

  if(channelMaskCntl == 6)
    {
      for(i = channelIndexMin; i < channelIndexMax; i++)
        {
          LoRa_UpdateChannelIdStatus(i, ENABLED);
        }
    }
  else if(channelMaskCntl == 0)
    {
      for(i = channelIndexMin; i < channelIndexMax; i++)
        {
          if(channelMask & BIT0 == BIT0)
            {
              LoRa_UpdateChannelIdStatus(i, ENABLED);
            }
          else
            {
              LoRa_UpdateChannelIdStatus(i, DISABLED);
            }
          channelMask = channelMask >> SHIFT1;
        }
    }
}

static void LoRa_DutyCycleCallback(uint8_t param)
{
  uint32_t minim = UINT32_MAX;
  bool found = false;
  uint8_t i;

  for(i = 0; i < MAX_EU_SINGLE_BAND_CHANNELS; i++)
    {
      //Validate this only for enabled channels
      if((LoRa_Channels[i].status == ENABLED) && (LoRa_Channels[i].channelTimer != 0))
        {
          if(LoRa_Channels[i].channelTimer > loRa.XXX_lastTimerValue)
            {
              LoRa_Channels[i].channelTimer = LoRa_Channels[i].channelTimer - loRa.XXX_lastTimerValue;
            }
          else
            {
              LoRa_Channels[i].channelTimer = 0;
            }
          if((LoRa_Channels[i].channelTimer <= minim) && (LoRa_Channels[i].channelTimer != 0))
            {
              minim = LoRa_Channels[i].channelTimer;
              found = true;
            }
        }
    }
  if(found == true)
    {
      loRa.XXX_lastTimerValue = minim;    
    }
}

void LoRa_ConfigureRadioTx(uint8_t dataRate, uint32_t freq)
{
  int8_t txPower;

  ConfigureRadio(dataRate, freq);

  if(ISM_EU868 == loRa.LoRa_ismBand)
    {
      txPower = txPower868[loRa.LoRa_txPower];
    }
  else
    {
      txPower = txPower868[loRa.LoRa_txPower];
    }

  RADIO_SetOutputPower(txPower);

  RADIO_SetCRC(ENABLED);
  RADIO_SetIQInverted(DISABLED);
}

