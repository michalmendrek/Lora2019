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
 *                           lorawan.c
 *
 * LoRaWAN file
 *
 ******************************************************************************/

/****************************** INCLUDES **************************************/
#include "lorawan.h"
#include "lorawan_aes.h"
#include "lorawan_aes_cmac.h"
#include "lorawan_defs.h"
#include "lorawan_private.h"
#include "AES.h"
#include "radio_interface.h"
#include "sw_timer.h"
#include "interrupt_manager_lora_addons.h"
#include <math.h>

#include "lorawan_eu.h"

/****************************** VARIABLES *************************************/

//CID = LinkCheckReq     = 2
//CID = LinkADRAns       = 3
//CID = DutyCycleAns     = 4
//CID = RX2SetupAns      = 5
//CID = DevStatusAns     = 6
//CID = NewChannelAns    = 7
//CID = RXTimingSetupAns = 8
// Index in macEndDevCmdReplyLen = CID - 2
static const uint8_t macEndDevCmdReplyLen[] = {1, 2, 1, 2, 3, 2, 1};

LoRa_t loRa;

uint8_t LoRa_radioBuffer[MAXIMUM_BUFFER_LENGTH];
static uint8_t aesBuffer[AES_BLOCKSIZE];

extern const uint8_t LoRa_maxPayloadSize[];
extern ChannelParams_t LoRa_Channels[];
extern const uint8_t rxWindowSize[];

/************************ FUNCTION PROTOTYPES *************************/
static void LoRa_AssemblePacket_XYf(uint8_t *buffer, uint16_t bufferLength, uint8_t nxt_channel);
static uint8_t LoRa_GetMaxPayloadSize(void);
uint8_t LoRa_CRC(uint8_t *buf, uint8_t cntr);
/**********************************************************************/

static uint8_t LoRa_GetMaxPayloadSize(void);

static bool LoRa_FindSmallestDataRate(void);

static void LoRa_ConfigureRadioRx_XYfe(uint8_t dataRate, uint32_t freq);

uint8_t localDioStatus;

/****************************** PUBLIC FUNCTIONS ******************************/
LorawanError_t LoRa_Send_Data(void)
{
  LorawanError_t result;

  LoRa_ConfigureRadioTx_XYfe(loRa.LoRa_sendChannelParameters.dataRate, loRa.LoRa_sendChannelParameters.frequency);

  if(RADIO_Transmit_XYfe(loRa.LoRa_HeaderBufor, loRa.LoRa_HeaderLength) == OK)
    {
      loRa.LoRa_StatusDanych = LoRa_transmiting;

      //      loRa.lorawanMacStatus.synchronization = ENABLED; //set the synchronization flag because one packet was sent (this is a guard for the the RxAppData of the user)
      loRa.LoRa_transmitStatus = LoRa_SendData_TX; // set the state of MAC to transmission occurring. No other packets can be sent afterwards
      SwTimerSetTimeout_Yf(loRa.LoRa_TimerWaitAck, MS_TO_TICKS_SHORT(LoRa_ACK_timeout));
    }
  else
    {
      return LoRa_Send_problem;
    }
  return OK;
}

LorawanError_t LoRa_Send_Header_XYfl(void)
{
  LorawanError_t result;

  LoRa_ConfigureRadioTx_XYfe(loRa.LoRa_ch0_params.dataRate, loRa.LoRa_ch0_params.frequency);

  if(RADIO_Transmit_XYfe(loRa.LoRa_HeaderBufor, loRa.LoRa_HeaderLength) == OK)
    {
      loRa.LoRa_StatusDanych = LoRa_transmiting;
      loRa.LoRa_Counnter.value++; // the uplink frame counter increments for every new transmission (it does not increment for a retransmission)

      //      loRa.lorawanMacStatus.synchronization = ENABLED; //set the synchronization flag because one packet was sent (this is a guard for the the RxAppData of the user)
      loRa.LoRa_transmitStatus = LoRa_Handshaking_TX; // set the state of MAC to transmission occurring. No other packets can be sent afterwards
      SwTimerSetTimeout_Yf(loRa.LoRa_TimerHandshaking, MS_TO_TICKS_SHORT(LoRa_Handshaking_timeout));
    }
  else
    {
      return LoRa_Send_problem;
    }
  return OK;
}

LorawanError_t LoRa_Send_XYfl(void *buffer, uint8_t bufferLength)
{
  LorawanError_t result;

  loRa.LoRa_StatusDanych = LoRa_transmit_Fail;
  //validate date length using MaxPayloadSize
  if(bufferLength > LoRa_GetMaxPayloadSize())
    {
      return INVALID_BUFFER_LENGTH;
    }

  uint8_t channel = Random(LoRa_Chann_nr) + 1;

  result = LoRa_SelectChannelForTransmission_XYf(channel, channel);

  LoRa_AssemblePacket_XYf(buffer, bufferLength, channel);

  if(LoRa_Send_Header_XYfl() == OK)
    {
      return OK;
    }
  else
    {
      return LoRa_Send_problem;
    }
}

void LoRa_EnterReceive_XYfe_HD(void)
{
  RADIO_clearFlag_Yf();

  switch(loRa.LoRa_transmitStatus)
    {
      case LoRa_Handshaking_TX:
        {
          //          LoRa_EnterReceive_head
          loRa.LoRa_transmitStatus = LoRa_Handshaking_RX;
          break;
        }

      case LoRa_SendData_TX:
        {
          //          LoRa_EnterReceive_data
          loRa.LoRa_transmitStatus = LoRa_SendData_RX;
          break;
        }
      default:
        loRa.LoRa_transmitStatus = ToDo_LoRa_retransmit_radio_configured;
        return;
    }

  LoRa_ConfigureRadioRx_XYfe(loRa.LoRa_receiveChannelParameters.dataRate, loRa.LoRa_receiveChannelParameters.frequency);

  if(RADIO_ReceiveStart_XYfe(4) != OK)
    {
      loRa.LoRa_transmitStatus = ToDo_LoRa_retransmit;
    }
}

LorawanError_t LoRa_RxDone_OK_XYfe(uint8_t *buffer, uint8_t bufferLength);
LorawanError_t LoRa_RxDone_Fail(void);

LorawanError_t LoRa_RxDone_XYfe(uint8_t *buffer, uint8_t bufferLength, bool RX_success)
{
  RADIO_clearFlag_Yf();

  SwTimerStop_Yf(loRa.LoRa_TimerHandshaking);
  SwTimerStop_Yf(loRa.LoRa_TimerWaitAck);

  if(RX_success)
    {
      LoRa_RxDone_OK_XYfe(buffer, bufferLength);
    }
  else
    {
      LoRa_RxDone_Fail();
    }

}

LoRaResponseError_t LoRa_HandshakingAckVerify_XYfe(uint8_t *buffer, uint8_t bufferLength)
{
  if(bufferLength != 2)
    {
      return Response_LenghtError;
    }
  if(*buffer != loRa.LoRa_Addres)
    {
      return Response_AddresError;
    }
  //  if(buffer[2] != LoRa_CRC(buffer, 2))
  //    {
  //      return Response_CrcError;
  //    }
  switch(buffer[1]) //loRa.LoRa_Command
    {
      case 0:
        {
          loRa.LoRa_Command = buffer[1];
          LoRa_ConfigureRadioTx_XYfe(loRa.LoRa_sendChannelParameters.dataRate, loRa.LoRa_sendChannelParameters.frequency);

          if(RADIO_Transmit_XYfe(loRa.LoRa_Bufor, loRa.LoRa_BuforLength) == OK)
            {
              return Response_OK;
            }
          else
            {
              return Response_Transmit_problem;
            }
          break;
        }
      default:
        {
          return Response_UnknownCommandError;
        }
    }
}

LoRaResponseError_t LoRa_DataAckVerify_XYfe(uint8_t *buffer, uint8_t bufferLength)
{
  if(bufferLength != 2)
    {
      return Response_LenghtError;
    }
  if(*buffer != loRa.LoRa_Addres)
    {
      return Response_AddresError;
    }
  //  if(buffer[2] != LoRa_CRC(buffer, 2))
  //    {
  //      return Response_CrcError;
  //    }
  if(loRa.LoRa_nextUsedChannel != buffer[1])
    {
      return Response_ChannelError;
    }
  return Response_OK;
}

LorawanError_t LoRa_RxDone_OK_XYfe(uint8_t *buffer, uint8_t bufferLength)
{
  switch(loRa.LoRa_transmitStatus)
    {
      case LoRa_Handshaking_RX:
        {
          if(LoRa_HandshakingAckVerify_XYfe(buffer, bufferLength) == Response_OK)
            {
              loRa.LoRa_transmitStatus = LoRa_SendData_TX;
              SwTimerSetTimeout_Yf(loRa.LoRa_TimerWaitAck, MS_TO_TICKS_SHORT(LoRa_Transmit_timeout));
            }
          else
            {
              loRa.LoRa_transmitStatus = ToDo_LoRa_retransmit_radio_configured;
            }
          break;
        }
      case LoRa_SendData_RX:
        {
          if(LoRa_DataAckVerify_XYfe(buffer, bufferLength) == Response_OK)
            {
              loRa.LoRa_transmitStatus = LoRa_Sent;
              SwTimerStop_Yf(loRa.LoRa_TimerWaitAck);
              RADIO_SwTimers_stop_Yf();
//              if(loRa.LoRa_nextUsedChannel == 0)
                {
                  loRa.LoRa_StatusDanych = LoRa_transmit_OK;
                }
            }
          else
            {
              loRa.LoRa_transmitStatus = ToDo_LoRa_retransmit_radio_configured;
            }
          break;
        }
      default:
        {
          loRa.LoRa_transmitStatus = ToDo_LoRa_retransmit;
        }
    }
  return OK;
}

LorawanError_t LoRa_RxDone_Fail(void)
{

}

void LoRa_UpdateMinMaxChDataRate_Yf(void)
{
  uint8_t i;
  // after updating the data range of a channel we need to check if the minimum dataRange has changed or not.
  // The user cannot set the current data rate outside the range of the data range
  loRa.LoRa_minDataRate = DR7;
  loRa.LoRa_maxDataRate = DR0;

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

uint8_t LoRa_CRC(uint8_t *buf, uint8_t cntr)
{
  uint8_t CRC = 0;
  for(uint8_t i = 0; i < cntr; i++)
    {
      CRC ^= *(buf++);
    }
  return(CRC);
}

static void LoRa_AssemblePacket_XYf(uint8_t *buffer, uint16_t bufferLength, uint8_t nxt_channel)
{
  uint8_t bufferHeadIndex = 0;
  uint8_t bufferIndex = 0;

  memset(&loRa.LoRa_Bufor, 0, sizeof(loRa.LoRa_Bufor)); //clear bufor
  memset(&loRa.LoRa_HeaderBufor, 0, sizeof(loRa.LoRa_HeaderBufor)); //clear header

  loRa.LoRa_HeaderBufor[bufferHeadIndex++] = loRa.LoRa_Addres;
  loRa.LoRa_HeaderBufor[bufferHeadIndex++] = nxt_channel;
  //  loRa.LoRa_HeaderBufor[bufferHeadIndex] = LoRa_CRC(loRa.LoRa_HeaderBufor, bufferHeadIndex);
  loRa.LoRa_HeaderLength = bufferHeadIndex;

  loRa.LoRa_Bufor[1] = 0; //Random(LoRa_Chann_nr) + 1; //nxt channel
  memcpy(&loRa.LoRa_Bufor[2], buffer, bufferLength);
  bufferIndex = bufferLength + 3;
  loRa.LoRa_Bufor[0] = bufferIndex;
  loRa.LoRa_Bufor[bufferIndex] = LoRa_CRC(loRa.LoRa_Bufor, bufferIndex);
  loRa.LoRa_BuforLength = bufferIndex + 1;
}

/******************************************************************************/
//Set and get functions

LorawanError_t LoRa_SetCurrentDataRate(uint8_t valueNew)
{
  // the current data rate cannot be smaller than the minimum data rate defined for all the channels or bigger than the maximum data rate defined for all the channels

  if((valueNew < loRa.LoRa_minDataRate) || (valueNew > loRa.LoRa_maxDataRate) || (LoRa_ValidateDataRate(valueNew) != OK))
    {
      return INVALID_PARAMETER;
    }
  else
    {
      LoRa_UpdateCurrentDataRate(valueNew);
      return OK;
    }
}

uint8_t LoRa_GetCurrentDataRate(void)
{
  return loRa.LoRa_currentDataRate;
}

LorawanError_t LoRa_SetTxPower(uint8_t txPowerNew)
{
  LorawanError_t result = OK;

  if(LoRa_ValidateTxPower(txPowerNew) == OK)
    {
      LoRa_UpdateTxPower(txPowerNew);
    }
  else
    {
      result = INVALID_PARAMETER;
    }
  return result;
}

uint8_t LoRa_GetTxPower(void)
{
  return loRa.LoRa_txPower;
}

uint8_t LoRa_GetSyncWord(void)
{
  return loRa.LoRa_syncWord;
}

void LoRa_SetSyncWord(uint8_t syncWord)
{
  loRa.LoRa_syncWord = syncWord;
}

void LoRa_SetCounter(uint32_t ctr)
{
  loRa.LoRa_Counnter.value = ctr;
}

uint32_t LoRa_GetCounter(void)
{
  return loRa.LoRa_Counnter.value;
}

// Set and get functions for protocol parameters

// battery Level: 0 - external power source,  1-254 level, 255: the end device was not able to measure the battery level
// default value for battery is 255 - the end device was not able to measure the battery level

void LoRa_SetBattery(uint8_t batteryLevelNew)
{
  loRa.LoRa_batteryLevel = batteryLevelNew;
}

void LoRa_StopTransmisions_XYfl(void)
{
  SwTimerStop_Yf(loRa.LoRa_TimerHandshaking);
  SwTimerStop_Yf(loRa.LoRa_TimerRetransmit);
  SwTimerStop_Yf(loRa.LoRa_TimerWaitAck);
  
  RADIO_SwTimers_stop_Yf();
  
  RADIO_clearFlag_Yf();
  
  RADIO_standby_XYfl();
}

void LoRa_TimerHandshakingCallback_XYfe(uint8_t param) //  timeout handshaking - nie nawi?zano polaczenia
{
  loRa.LoRa_transmitStatus = LoRa_transmit_Error;
  
  LoRa_StopTransmisions_XYfl();
  
  LoRa_PrepareRetransmit_XYfl();
}

void LoRa_PrepareRetransmit_XYfl(void)
{
  
  if(loRa.LoRa_Counnter.value < LoRa_Retransmit_trials)
    {
      SwTimerSetTimeout_Yf(loRa.LoRa_TimerRetransmit, MS_TO_TICKS_LONG(LoRa_Retransmit_timeout));
      SwTimerStart_Yf(loRa.LoRa_TimerRetransmit);
      loRa.LoRa_transmitStatus =LoRa_Wait_retransmit;
      loRa.LoRa_StatusDanych = LoRa_transmiting;
    }
  else
    {
      loRa.LoRa_StatusDanych = LoRa_transmit_Fail;
    }
}

void LoRa_TimerWaitAckCallback(uint8_t param)
{
  loRa.LoRa_transmitStatus = LoRa_transmit_Error;

  SwTimerStop_Yf(loRa.LoRa_TimerWaitAck);
  RADIO_standby_XYfl();
  RADIO_clearTransmitFlag();
  RADIO_clearReceiveFlag();
}

void LoRa_TimerRetransmitCallback(uint8_t param)
{
  SwTimerStop_Yf(loRa.LoRa_TimerRetransmit);
  LoRa_Send_Header_XYfl();
}

void LoRa_TxWdtTimeout(void)
{
  LoRa_PrepareRetransmit_XYfl();
}

void LoRa_RxWdtTimeout(void)
{
  LoRa_PrepareRetransmit_XYfl();
}

void LoRa_UpdateCurrentDataRate(uint8_t valueNew)
{
  loRa.LoRa_currentDataRate = valueNew;
}

void LoRa_UpdateTxPower(uint8_t txPowerNew)
{
  loRa.LoRa_txPower = txPowerNew;
}

//Generates a 16-bit random number based on some measurements made by the radio (seed is initialized inside mac Reset)

uint16_t Random(uint16_t max)
{
  return(rand() % max);
}

static uint8_t LoRa_GetMaxPayloadSize(void)
{
  uint8_t result;
  result = LoRa_maxPayloadSize[loRa.LoRa_currentDataRate];

  return result;
}

// This function checks which value can be assigned to the current data rate.

static bool LoRa_FindSmallestDataRate(void)
{
  uint8_t i = 0, dataRate;
  bool found = false;

  if(loRa.LoRa_currentDataRate > loRa.LoRa_minDataRate)
    {
      dataRate = loRa.LoRa_currentDataRate - 1;

      while((found == false) && (dataRate >= loRa.LoRa_minDataRate))
        {
          for(i = 0; i < loRa.LoRa_maxChannels; i++)
            {
              if((dataRate >= LoRa_Channels[i].dataRange.min) && (dataRate <= LoRa_Channels[i].dataRange.max) && (LoRa_Channels[i].status == ENABLED))
                {
                  found = true;
                  break;
                }
            }
          if((found == false) && (dataRate > loRa.LoRa_minDataRate)) // if no channels were found after one search, then the device will switch to the next lower data rate possible
            {
              dataRate = dataRate - 1;
            }
        }

      if(found == true)
        {
          loRa.LoRa_currentDataRate = dataRate;
        }
    }

  return found;
}

static void LoRa_ConfigureRadioRx_XYfe(uint8_t dataRate, uint32_t freq) //OK
{
  LoRa_ConfigureRadio_XYfe(dataRate, freq);
  RADIO_SetCRC_Yf(DISABLED);
  RADIO_SetIQInverted_Yf(ENABLED);
}

//Based on the last packet received, this function checks the flags and updates the state accordingly

void LoRa_Mainloop(void)
{
  SwTimersExecute();

  //Execute radio interrupts
  localDioStatus = INTERRUPT_GetDioStatus();

  if((localDioStatus & DIO0) != 0)
    {
      RADIO_DIO0();
    }
  if((localDioStatus & DIO1) != 0)
    {
      RADIO_DIO1();
    }
  if((localDioStatus & DIO2) != 0)
    {
      RADIO_DIO2();
    }
  if((localDioStatus & DIO3) != 0)
    {
      RADIO_DIO3();
    }
  if((localDioStatus & DIO4) != 0)
    {
      RADIO_DIO4();
    }
  if((localDioStatus & DIO5) != 0)
    {
      RADIO_DIO5();
    }
}
