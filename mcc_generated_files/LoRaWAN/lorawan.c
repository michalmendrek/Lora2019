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

uint8_t macBuffer[MAXIMUM_BUFFER_LENGTH];
uint8_t LoRa_radioBuffer[MAXIMUM_BUFFER_LENGTH];
static uint8_t aesBuffer[AES_BLOCKSIZE];
RxAppData_t rxPayload;

extern const uint8_t LoRa_maxPayloadSize[];
extern ChannelParams_t LoRa_Channels[];
extern const uint8_t rxWindowSize[];

/************************ FUNCTION PROTOTYPES *************************/
static void LoRa_AssemblePacket(uint8_t *buffer, uint16_t bufferLength, uint8_t nxt_channel);
static uint8_t LoRa_GetMaxPayloadSize(void);
uint8_t LoRa_CRC(uint8_t *buf, uint8_t cntr);
/**********************************************************************/

static void UpdateReceiveDelays(uint8_t delay);

static uint8_t LoRa_GetMaxPayloadSize(void);

static bool FindSmallestDataRate(void);

static uint8_t* ExecuteRxTimingSetup(uint8_t *ptr);

static void PrepareSessionKeys(uint8_t* sessionKey, uint8_t* appNonce, uint8_t* networkId);

static void ComputeSessionKeys(JoinAccept_t *joinAcceptBuffer);

static uint8_t CountfOptsLength(void);

static void UpdateJoinInProgress(uint8_t state);

static void CheckFlags(Hdr_t* hdr);

static uint8_t CheckMcastFlags(Hdr_t* hdr);

static void AssembleEncryptionBlock(uint8_t dir, uint32_t frameCounter, uint8_t blockId, uint8_t firstByte, uint8_t multicastStatus);

static uint32_t ExtractMic(uint8_t *buffer, uint8_t bufferLength);

static uint32_t ComputeMic(uint8_t *key, uint8_t* buffer, uint8_t bufferLength);

static void EncryptFRMPayload(uint8_t* buffer, uint8_t bufferLength, uint8_t dir, uint32_t frameCounter, uint8_t* key, uint8_t macBufferIndex, uint8_t* bufferToBeEncrypted, uint8_t multicastStatus);

static uint8_t* MacExecuteCommands(uint8_t *buffer, uint8_t fOptsLen);

static void SetReceptionNotOkState(void);

static void ConfigureRadioRx(uint8_t dataRate, uint32_t freq);

extern void UpdateCfList(uint8_t bufferLength, JoinAccept_t *joinAccept);

uint8_t localDioStatus;

/****************************** PUBLIC FUNCTIONS ******************************/
LorawanError_t LoRa_Send_Data(void)
{
  LorawanError_t result;

  ConfigureRadioTx(loRa.LoRa_sendChannelParameters.dataRate, loRa.LoRa_sendChannelParameters.frequency);

  if(RADIO_Transmit(loRa.LoRa_HeaderBufor, loRa.LoRa_HeaderLength) == OK)
    {
      loRa.LoRa_StatusDanych = LoRa_transmiting;

      //      loRa.lorawanMacStatus.synchronization = ENABLED; //set the synchronization flag because one packet was sent (this is a guard for the the RxAppData of the user)
      loRa.LoRa_transmitStatus = LoRa_SendData_TX; // set the state of MAC to transmission occurring. No other packets can be sent afterwards
      SwTimerSetTimeout(loRa.LoRa_TimerWaitAck, MS_TO_TICKS_SHORT(LoRa_ACK_timeout));
    }
  else
    {
      return LoRa_Send_problem;
    }
  return OK;
}

LorawanError_t LoRa_Send_Header(void)
{
  LorawanError_t result;

  ConfigureRadioTx(loRa.LoRa_ch0_params.dataRate, loRa.LoRa_ch0_params.frequency);

  if(RADIO_Transmit(loRa.LoRa_HeaderBufor, loRa.LoRa_HeaderLength) == OK)
    {
      loRa.LoRa_StatusDanych = LoRa_transmiting;
      loRa.LoRa_Counnter.value++; // the uplink frame counter increments for every new transmission (it does not increment for a retransmission)

      //      loRa.lorawanMacStatus.synchronization = ENABLED; //set the synchronization flag because one packet was sent (this is a guard for the the RxAppData of the user)
      loRa.LoRa_transmitStatus = LoRa_Handshaking_TX; // set the state of MAC to transmission occurring. No other packets can be sent afterwards
      SwTimerSetTimeout(loRa.LoRa_TimerHandshaking, MS_TO_TICKS_SHORT(LoRa_Handshaking_timeout));
    }
  else
    {
      return LoRa_Send_problem;
    }
  return OK;
}

LorawanError_t LoRa_Send(void *buffer, uint8_t bufferLength)
{
  LorawanError_t result;

  loRa.LoRa_StatusDanych = LoRa_transmit_Fail;
  //validate date length using MaxPayloadSize
  if(bufferLength > LoRa_GetMaxPayloadSize())
    {
      return INVALID_BUFFER_LENGTH;
    }

  if((loRa.LoRa_transmitStatus != LoRa_Idle))
    {
      return MAC_STATE_NOT_READY_FOR_TRANSMISSION;
    }
  uint8_t channel = Random(LoRa_Chann_nr) + 1;

  result = LoRa_SelectChannelForTransmission(channel, channel);

  LoRa_AssemblePacket(buffer, bufferLength, channel);

  if(LoRa_Send_Header() == OK)
    {
      return OK;
    }
  else
    {
      return LoRa_Send_problem;
    }
}

void LoRa_EnterReceive(void)
{
  RADIO_clearFlag();

  ConfigureRadioRx(loRa.LoRa_receiveChannelParameters.dataRate, loRa.LoRa_receiveChannelParameters.frequency);

  if(RADIO_ReceiveStart(4) != OK)
    {

    }
}
LorawanError_t LoRa_RxDone_OK(uint8_t *buffer, uint8_t bufferLength);
LorawanError_t LoRa_RxDone_Fail(void);

LorawanError_t LoRa_RxDone(uint8_t *buffer, uint8_t bufferLength, bool RX_success)
{
  RADIO_clearFlag();

  SwTimerStop(loRa.LoRa_TimerHandshaking);
  SwTimerStop(loRa.LoRa_TimerWaitAck);

  if(RX_success)
    {
      LoRa_RxDone_OK(buffer, bufferLength);
    }
  else
    {
      LoRa_RxDone_Fail();
    }

}

LorawanError_t LoRa_RxDone_OK(uint8_t *buffer, uint8_t bufferLength)
{
  if(loRa.LoRa_transmitStatus == LoRa_Handshaking_RX)
    {
      if((*buffer == loRa.LoRa_Addres)&&(bufferLength == 3))
        {
          if(buffer[2] == LoRa_CRC(buffer, 2))
            {
              loRa.LoRa_Command = buffer[1];
              if(loRa.LoRa_Command == 0)
                {
                  ConfigureRadioTx(loRa.LoRa_sendChannelParameters.dataRate, loRa.LoRa_sendChannelParameters.frequency);

                  if(RADIO_Transmit(loRa.LoRa_Bufor, loRa.LoRa_BuforLength) == OK)
                    {
                      loRa.LoRa_transmitStatus = LoRa_SendData_TX;
                      SwTimerSetTimeout(loRa.LoRa_TimerWaitAck, MS_TO_TICKS_SHORT(LoRa_Transmit_timeout));
                    }
                }
            }
        }
    }
  else if(loRa.LoRa_transmitStatus == LoRa_SendData_RX)
    {
      if((*buffer == loRa.LoRa_Addres))
        {
          if(buffer[bufferLength - 1] == LoRa_CRC(buffer, bufferLength - 1))
            {

              if(loRa.LoRa_nextUsedChannel == buffer[1])
                {
                  SwTimerStop(loRa.LoRa_TimerWaitAck);
                  RADIO_SwTimers_stop();
                  loRa.LoRa_transmitStatus = LoRa_Sent;
                  loRa.LoRa_StatusDanych = LoRa_transmit_OK;
                }
            }
        }
    }

  return OK;
}

LorawanError_t LoRa_RxDone_Fail(void)
{

}

void LoRa_UpdateMinMaxChDataRate(void)
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

static void LoRa_AssemblePacket(uint8_t *buffer, uint16_t bufferLength, uint8_t nxt_channel)
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

LorawanError_t LORAWAN_Join(ActivationType_t activationTypeNew)
{
  uint8_t bufferIndex;
  LorawanError_t result;

  if(loRa.macStatus.macPause == ENABLED)
    {
      return MAC_PAUSED; // Any further transmissions or receptions cannot occur is macPaused is enabled.
    }

  if(loRa.macStatus.silentImmediately == ENABLED)
    {
      return SILENT_IMMEDIATELY_ACTIVE;
    }

  if(loRa.macStatus.macState != IDLE)
    {
      return MAC_STATE_NOT_READY_FOR_TRANSMISSION;
    }

  loRa.activationParameters.activationType = activationTypeNew;

  if(OTAA == activationTypeNew)
    {
      //OTAA
      if((loRa.macKeys.deviceEui == 0) || (loRa.macKeys.applicationEui == 0) || (loRa.macKeys.applicationKey == 0))
        {
          return KEYS_NOT_INITIALIZED;
        }
      else
        {
          result = SelectChannelForTransmission(0);

          if(result == OK)
            {
              if(RADIO_Transmit(macBuffer, bufferIndex) == OK)
                {
                  UpdateJoinInProgress(TRANSMISSION_OCCURRING);
                  return OK;
                }
              else
                {
                  return MAC_STATE_NOT_READY_FOR_TRANSMISSION;
                }
            }
          else
            {
              return result;
            }
        }
    }
  else
    {
      //ABP
      if((loRa.macKeys.applicationSessionKey == 0) || (loRa.macKeys.networkSessionKey == 0) || (loRa.macKeys.deviceAddress == 0))
        {
          return KEYS_NOT_INITIALIZED;
        }
      else
        {
          UpdateJoinInProgress(ABP_DELAY);

          return OK;
        }
    }
}

LorawanError_t LORAWAN_Send(TransmissionType_t confirmed, uint8_t port, void *buffer, uint8_t bufferLength)
{
  LorawanError_t result;

  if(loRa.macStatus.macPause == ENABLED)
    {
      return MAC_PAUSED; // Any further transmissions or receptions cannot occur is macPaused is enabled.
    }

  if(loRa.macStatus.silentImmediately == ENABLED) // The server decided that any further uplink transmission is not possible from this end device.
    {
      return SILENT_IMMEDIATELY_ACTIVE;
    }

  if(loRa.macStatus.networkJoined == DISABLED) //The network needs to be joined before sending
    {
      return NETWORK_NOT_JOINED;
    }

  if((port < FPORT_MIN) && (bufferLength != 0)) //Port number should be <= 1 if there is data to send. If port number is 0, it indicates only Mac commands are inside FRM Payload
    {
      return INVALID_PARAMETER;
    }

  //validate date length using MaxPayloadSize

  if(loRa.fCntUp.value == UINT32_MAX)
    {
      // Inform application about rejoin in status
      loRa.macStatus.rejoinNeeded = 1;
      return FRAME_COUNTER_ERROR_REJOIN_NEEDED;
    }

  if((loRa.macStatus.macState != IDLE) && (CLASS_A == loRa.deviceClass))
    {
      return MAC_STATE_NOT_READY_FOR_TRANSMISSION;
    }

  result = SelectChannelForTransmission(1);
  if(result != OK)
    {
      return result;
    }
  else
    {
      if(CLASS_C == loRa.deviceClass)
        {
          RADIO_ReceiveStop();
        }

      if(RADIO_Transmit(&macBuffer[16], loRa.lastPacketLength) == OK)
        {
          loRa.fCntUp.value++; // the uplink frame counter increments for every new transmission (it does not increment for a retransmission)

          if(CNF == confirmed)
            {
              loRa.lorawanMacStatus.ackRequiredFromNextDownlinkMessage = ENABLED;
            }
          loRa.lorawanMacStatus.synchronization = ENABLED; //set the synchronization flag because one packet was sent (this is a guard for the the RxAppData of the user)
          loRa.macStatus.macState = TRANSMISSION_OCCURRING; // set the state of MAC to transmission occurring. No other packets can be sent afterwards
        }
      else
        {
          return MAC_STATE_NOT_READY_FOR_TRANSMISSION;
        }
    }

  return OK;
}

//Set and get functions

LorawanError_t LORAWAN_SetMcast(bool status)
{
  if(CLASS_A == loRa.deviceClass)
    {
      return INVALID_CLASS; // it works only for Class B and Class C
    }

  // Only ABP shall be checked
  if(CLASS_C == loRa.deviceClass)
    {
      if(ENABLED == status)
        {
          if((0 == loRa.macKeys.mcastApplicationSessionKey) ||
             (0 == loRa.macKeys.mcastNetworkSessionKey) ||
             (0 == loRa.macKeys.mcastDeviceAddress))
            {
              return MCAST_PARAM_ERROR;
            }

          loRa.macStatus.mcastEnable = ENABLED;
        }
      else
        {
          loRa.macStatus.mcastEnable = DISABLED;
        }
    }

  return OK;
}

bool LORAWAN_GetMcast(void)
{
  return loRa.macStatus.mcastEnable;
}

void LORAWAN_SetMcastDeviceAddress(uint32_t mcastDeviceAddressNew)
{
  loRa.activationParameters.mcastDeviceAddress.value = mcastDeviceAddressNew;
  loRa.macKeys.mcastDeviceAddress = 1;
}

uint32_t LORAWAN_GetMcastDeviceAddress(void)
{
  return loRa.activationParameters.mcastDeviceAddress.value;
}

void LORAWAN_SetMcastNetworkSessionKey(uint8_t *mcastNetworkSessionKeyNew)
{
  memcpy(loRa.activationParameters.mcastNetworkSessionKey, mcastNetworkSessionKeyNew, 16);
  loRa.macKeys.mcastNetworkSessionKey = 1;
}

void LORAWAN_SetMcastApplicationSessionKey(uint8_t *mcastApplicationSessionKeyNew)
{
  memcpy(loRa.activationParameters.mcastApplicationSessionKey, mcastApplicationSessionKeyNew, 16);
  loRa.macKeys.mcastApplicationSessionKey = 1;
}

void LORAWAN_GetMcastApplicationSessionKey(uint8_t *mcastApplicationSessionKey)
{
  if(mcastApplicationSessionKey != NULL)
    {
      memcpy(mcastApplicationSessionKey, loRa.activationParameters.mcastApplicationSessionKey, 16);
    }
}

void LORAWAN_GetMcastNetworkSessionKey(uint8_t *mcastNetworkSessionKey)
{
  if(mcastNetworkSessionKey != NULL)
    {
      memcpy(mcastNetworkSessionKey, loRa.activationParameters.mcastNetworkSessionKey, sizeof(loRa.activationParameters.mcastNetworkSessionKey));
    }
}

void LORAWAN_SetDeviceEui(uint8_t *deviceEuiNew)
{
  if(deviceEuiNew != NULL)
    {
      memcpy(loRa.activationParameters.deviceEui.buffer, deviceEuiNew, sizeof(loRa.activationParameters.deviceEui));
      loRa.macKeys.deviceEui = 1;
      loRa.macStatus.networkJoined = DISABLED; // this is a guard against overwriting any of the addresses after one join was already done. If any of the addresses change, rejoin is needed
    }
}

void LORAWAN_GetDeviceEui(uint8_t *deviceEui)
{
  memcpy(deviceEui, loRa.activationParameters.deviceEui.buffer, sizeof(loRa.activationParameters.deviceEui));
}

void LORAWAN_SetApplicationEui(uint8_t *applicationEuiNew)
{
  if(applicationEuiNew != NULL)
    {
      memcpy(loRa.activationParameters.applicationEui.buffer, applicationEuiNew, 8);
      loRa.macKeys.applicationEui = 1;
      loRa.macStatus.networkJoined = DISABLED; // this is a guard against overwriting any of the addresses after one join was already done. If any of the addresses change, rejoin is needed
    }
}

void LORAWAN_GetApplicationEui(uint8_t *applicationEui)
{
  memcpy(applicationEui, loRa.activationParameters.applicationEui.buffer, sizeof(loRa.activationParameters.applicationEui));
}

void LORAWAN_SetDeviceAddress(uint32_t deviceAddressNew)
{
  loRa.activationParameters.deviceAddress.value = deviceAddressNew;
  loRa.macKeys.deviceAddress = 1;
  loRa.macStatus.networkJoined = DISABLED; // this is a guard against overwriting any of the addresses after one join was already done. If any of the addresses change, rejoin is needed
}

uint32_t LORAWAN_GetDeviceAddress(void)
{
  return loRa.activationParameters.deviceAddress.value;
}

void LORAWAN_SetNetworkSessionKey(uint8_t *networkSessionKeyNew)
{
  if(networkSessionKeyNew != NULL)
    {
      memcpy(loRa.activationParameters.networkSessionKey, networkSessionKeyNew, 16);
      loRa.macKeys.networkSessionKey = 1;
      loRa.macStatus.networkJoined = DISABLED; // this is a guard against overwriting any of the addresses after one join was already done. If any of the addresses change, rejoin is needed
    }
}

void LORAWAN_GetNetworkSessionKey(uint8_t *networkSessionKey)
{
  memcpy(networkSessionKey, loRa.activationParameters.networkSessionKey, sizeof(loRa.activationParameters.networkSessionKey));
}

void LORAWAN_SetApplicationSessionKey(uint8_t *applicationSessionKeyNew)
{
  if(applicationSessionKeyNew != NULL)
    {
      memcpy(loRa.activationParameters.applicationSessionKey, applicationSessionKeyNew, 16);
      loRa.macKeys.applicationSessionKey = 1;
      loRa.macStatus.networkJoined = DISABLED; // this is a guard against overwriting any of the addresses after one join was already done. If any of the addresses change, rejoin is needed
    }
}

void LORAWAN_GetApplicationSessionKey(uint8_t *applicationSessionKey)
{
  memcpy(applicationSessionKey, loRa.activationParameters.applicationSessionKey, sizeof(loRa.activationParameters.applicationSessionKey));
}

void LORAWAN_SetApplicationKey(uint8_t *applicationKeyNew)
{
  if(applicationKeyNew != NULL)
    {
      memcpy(loRa.activationParameters.applicationKey, applicationKeyNew, 16);
      loRa.macKeys.applicationKey = 1;
      loRa.macStatus.networkJoined = DISABLED; // this is a guard against overwriting any of the addresses after one join was already done. If any of the addresses change, rejoin is needed
    }
}

void LORAWAN_GetApplicationKey(uint8_t *applicationKey)
{
  memcpy(applicationKey, loRa.activationParameters.applicationKey, sizeof(loRa.activationParameters.applicationKey));
}

void LORAWAN_SetAdr(bool status)
{
  loRa.macStatus.adr = status;
  loRa.lorawanMacStatus.adrAckRequest = DISABLED; // this flag should only be on when ADR is set and the adr ack counter is bigger than adr ack limit
}

bool LORAWAN_GetAdr(void)
{
  return loRa.macStatus.adr;
}

LorawanError_t LoRa_SetCurrentDataRate(uint8_t valueNew)
{
  // the current data rate cannot be smaller than the minimum data rate defined for all the channels or bigger than the maximum data rate defined for all the channels

  if((valueNew < loRa.LoRa_minDataRate) || (valueNew > loRa.LoRa_maxDataRate) || (ValidateDataRate(valueNew) != OK))
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

LorawanError_t LORAWAN_SetTxPower(uint8_t txPowerNew)
{
  LorawanError_t result = OK;

  if(ValidateTxPower(txPowerNew) == OK)
    {
      UpdateTxPower(txPowerNew);
    }
  else
    {
      result = INVALID_PARAMETER;
    }
  return result;
}

uint8_t LORAWAN_GetTxPower(void)
{
  return loRa.txPower;
}

uint8_t LORAWAN_GetSyncWord(void)
{
  return loRa.syncWord;
}

void LORAWAN_SetSyncWord(uint8_t syncWord)
{
  loRa.syncWord = syncWord;
}

void LORAWAN_SetUplinkCounter(uint32_t ctr)
{
  loRa.fCntUp.value = ctr;
}

uint32_t LORAWAN_GetUplinkCounter(void)
{
  return loRa.fCntUp.value;
}

void LORAWAN_SetDownlinkCounter(uint32_t ctr)
{
  loRa.fCntDown.value = ctr;
}

uint32_t LORAWAN_GetDownlinkCounter(void)
{
  return loRa.fCntDown.value;
}

// Set and get functions for protocol parameters

void LORAWAN_SetReceiveDelay1(uint16_t receiveDelay1New)
{
  }

uint16_t LORAWAN_GetReceiveDelay1(void)
{
  }

uint16_t LORAWAN_GetReceiveDelay2(void)
{
  }

void LORAWAN_SetJoinAcceptDelay1(uint16_t joinAcceptDelay1New)
{
  }

uint16_t LORAWAN_GetJoinAcceptDelay1(void)
{
  }

void LORAWAN_SetJoinAcceptDelay2(uint16_t joinAcceptDelay2New)
{
  }

uint16_t LORAWAN_GetJoinAcceptDelay2(void)
{
  }

void LORAWAN_SetMaxFcntGap(uint16_t maxFcntGapNew)
{
  }

uint16_t LORAWAN_GetMaxFcntGap(void)
{
  }

void LORAWAN_SetAdrAckLimit(uint8_t adrAckLimitNew)
{
  }

uint8_t LORAWAN_GetAdrAckLimit(void)
{
  }

void LORAWAN_SetAdrAckDelay(uint8_t adrAckDelayNew)
{
  }

uint8_t LORAWAN_GetAdrAckDelay(void)
{
  }

void LORAWAN_SetAckTimeout(uint16_t ackTimeoutNew)
{
 }

uint16_t LORAWAN_GetAckTimeout(void)
{
  }

void LORAWAN_SetClass(LoRaClass_t deviceClass)
{
  loRa.deviceClass = deviceClass;
  loRa.macStatus.mcastEnable = 0;

  if(CLASS_C == deviceClass)
    {
      RADIO_SetWatchdogTimeout(0);
    }
  else if(deviceClass == CLASS_A)
    {
      loRa.macStatus.macState = IDLE;
      RADIO_SetWatchdogTimeout(WATCHDOG_DEFAULT_TIME);
      RADIO_ReceiveStop();
    }
}

LoRaClass_t LORAWAN_GetClass(void)
{
  return loRa.deviceClass;
}

void LORAWAN_SetMcastDownCounter(uint32_t newCnt)
{
  loRa.fMcastCntDown.value = newCnt;
}

uint32_t LORAWAN_GetMcastDownCounter()
{
  return loRa.fMcastCntDown.value;
}

void LORAWAN_SetNumberOfRetransmissions(uint8_t numberRetransmissions)
{
  loRa.maxRepetitionsConfirmedUplink = numberRetransmissions;
}

// for confirmed frames, default value is 8. The number of retransmissions includes also the first transmission

uint8_t LORAWAN_GetNumberOfRetransmissions(void)
{
  return loRa.maxRepetitionsConfirmedUplink;
}

void LORAWAN_GetReceiveWindow2Parameters(uint32_t* frequency, uint8_t* dataRate)
{
  *dataRate = loRa.receiveWindow2Parameters.dataRate;
  *frequency = loRa.receiveWindow2Parameters.frequency;
}

// battery Level: 0 - external power source,  1-254 level, 255: the end device was not able to measure the battery level
// default value for battery is 255 - the end device was not able to measure the battery level

void LoRa_SetBattery(uint8_t batteryLevelNew)
{
  loRa.LoRa_batteryLevel = batteryLevelNew;
}

// the LORAWAN_GetPrescaler function returns the prescaler value that is sent by the server to the end device via the Mac Command
// not user configurable

uint16_t LORAWAN_GetPrescaler(void)
{
  return loRa.prescaler;
}

// if status is enabled, responses to ACK and MAC commands will be sent immediately

void LORAWAN_SetAutomaticReply(bool status)
{
  loRa.macStatus.automaticReply = status;
}

bool LORAWAN_GetAutomaticReply(void)
{
  return loRa.macStatus.automaticReply;
}

// not user configurable

uint32_t LORAWAN_GetStatus(void)
{
  uint32_t status = loRa.macStatus.value;

  loRa.macStatus.channelsModified = DISABLED;
  loRa.macStatus.txPowerModified = DISABLED;
  loRa.macStatus.nbRepModified = DISABLED;
  loRa.macStatus.prescalerModified = DISABLED;
  loRa.macStatus.secondReceiveWindowModified = DISABLED;
  loRa.macStatus.rxTimingSetup = DISABLED;

  return status;
}

// not user configurable

/* This function is called when there is a need to send data outside the MAC layer
 It can be called when MAC is in idle, before RX1, between RX1 and RX2 or retransmission delay state
 It will return how much time other transmissions can occur*/
uint32_t LORAWAN_Pause(void)
{
  uint32_t timeToPause;

  switch(loRa.macStatus.macState)
    {
      case IDLE:
        {
          timeToPause = UINT32_MAX;
        }
        break;

      case BEFORE_RX1:
        {
          if(loRa.lorawanMacStatus.joining == ENABLED)
            {

            }
          else if(loRa.macStatus.networkJoined == ENABLED)
            {
              
            }
        }
        break;

      case BETWEEN_RX1_RX2:
        {
          if(loRa.lorawanMacStatus.joining == ENABLED)
            {

            }
          else if(loRa.macStatus.networkJoined == ENABLED)
            {

            }
          timeToPause = TICKS_TO_MS(timeToPause);
        }
        break;

      case RETRANSMISSION_DELAY:
        {
          timeToPause = TICKS_TO_MS(timeToPause);
        }
        break;

      default:
        {
          timeToPause = 0;
        }
        break;
    }

  if(timeToPause >= 200)
    {
      timeToPause = timeToPause - 50; //this is a guard in case of non-syncronization
      loRa.macStatus.macPause = ENABLED;
    }
  else
    {
      timeToPause = 0;
      loRa.macStatus.macPause = DISABLED;
    }

  return timeToPause;
}

void LORAWAN_Resume(void)
{
  loRa.macStatus.macPause = DISABLED;
}

// period will be in seconds

void LORAWAN_LinkCheckConfigure(uint16_t period)
{
  }

// if LORAWAN_ForceEnable is sent, the Silent Immediately bit sent by the end device is discarded and transmission is possible again

void LORAWAN_ForceEnable(void)
{
  loRa.macStatus.silentImmediately = DISABLED;
}

void LoRa_TimerHandshakingCallback(uint8_t param) //  timeout handshaking - nie nawi?zano polaczenia
{
  loRa.LoRa_transmitStatus = LoRa_transmit_Error;

  RADIO_clearFlag();
  SwTimerStop(loRa.LoRa_TimerHandshaking);
  SwTimerStop(loRa.LoRa_TimerWaitAck);
  SwTimerStop(loRa.LoRa_TimerRetransmit);
  RADIO_standby();
  RADIO_SwTimers_stop();

  SwTimerSetTimeout(loRa.LoRa_TimerRetransmit, MS_TO_TICKS_LONG(LoRa_Retransmit_timeout));
  if(loRa.LoRa_Counnter.value < LoRa_Retransmit_trials)
    {
      SwTimerStart(loRa.LoRa_TimerRetransmit);
    }
  else
    {
      loRa.LoRa_StatusDanych = LoRa_transmit_Fail;
    }
}

void LoRa_TimerRetransmitCallback(uint8_t param)
{
  SwTimerStop(loRa.LoRa_TimerRetransmit);
  LoRa_Send_Header();
}

void LoRa_TimerWaitAckCallback(uint8_t param)
{
  loRa.LoRa_transmitStatus = LoRa_transmit_Error;

  SwTimerStop(loRa.LoRa_TimerWaitAck);
  RADIO_standby();
  RADIO_clearTransmitFlag();
  RADIO_clearReceiveFlag();
}

void LORAWAN_EnterContinuousReceive(void)
{
  RADIO_ReceiveStop();
  RADIO_ReleaseData();

  ConfigureRadioRx(loRa.receiveWindow2Parameters.dataRate, loRa.receiveWindow2Parameters.frequency);

  if(RADIO_ReceiveStart(CLASS_C_RX_WINDOW_SIZE) != OK)
    {
      ResetParametersForConfirmedTransmission();
      ResetParametersForUnconfirmedTransmission();
      loRa.macStatus.macState = IDLE;
      if(rxPayload.RxAppData != NULL)
        {
          rxPayload.RxAppData(NULL, 0, MAC_NOT_OK);
        }
    }
}

void LoRa_UpdateCurrentDataRate(uint8_t valueNew)
{
  loRa.LoRa_currentDataRate = valueNew;
}

void UpdateTxPower(uint8_t txPowerNew)
{
  loRa.txPower = txPowerNew;
}

void UpdateRetransmissionAckTimeoutState(void)
{
  loRa.macStatus.macState = RETRANSMISSION_DELAY;
}

void UpdateReceiveWindow2Parameters(uint32_t frequency, uint8_t dataRate)
{
  loRa.receiveWindow2Parameters.dataRate = dataRate;
  loRa.receiveWindow2Parameters.frequency = frequency;
}

void ResetParametersForConfirmedTransmission(void)
{
  loRa.macStatus.macState = IDLE;
  loRa.counterRepetitionsConfirmedUplink = 1;
  loRa.lorawanMacStatus.ackRequiredFromNextDownlinkMessage = DISABLED;
}

void ResetParametersForUnconfirmedTransmission(void)
{
  loRa.macStatus.macState = IDLE;
  loRa.counterRepetitionsUnconfirmedUplink = 1;
  loRa.crtMacCmdIndex = 0;
}

void SetJoinFailState(void)
{
  loRa.macStatus.networkJoined = 0;
  loRa.lorawanMacStatus.joining = 0;
  loRa.macStatus.macState = IDLE;
  if(rxPayload.RxJoinResponse != NULL)
    {
      rxPayload.RxJoinResponse(REJECTED); // inform the application layer that join failed via callback
    }
}

//Generates a 16-bit random number based on some measurements made by the radio (seed is initialized inside mac Reset)

uint16_t Random(uint16_t max)
{
  return(rand() % max);
}

LorawanError_t LORAWAN_RxDone(uint8_t *buffer, uint8_t bufferLength)
{
}

static uint8_t LoRa_GetMaxPayloadSize(void)
{
  uint8_t result;
      result = LoRa_maxPayloadSize[loRa.LoRa_currentDataRate];
 
  return result;
}

static uint8_t* MacExecuteCommands(uint8_t *buffer, uint8_t fOptsLen)
{
  bool done = false;
  uint8_t *ptr;
  ptr = buffer;
  while((ptr < (buffer + fOptsLen)) && (done == false))
    {
      //Clean structure before using it         
      loRa.macCommands[loRa.crtMacCmdIndex].channelMaskAck = 0;
      loRa.macCommands[loRa.crtMacCmdIndex].dataRateAck = 0;
      loRa.macCommands[loRa.crtMacCmdIndex].powerAck = 0;
      loRa.macCommands[loRa.crtMacCmdIndex].channelAck = 0;
      loRa.macCommands[loRa.crtMacCmdIndex].dataRateReceiveWindowAck = 0;
      loRa.macCommands[loRa.crtMacCmdIndex].rx1DROffestAck = 0;
      loRa.macCommands[loRa.crtMacCmdIndex].dataRateRangeAck = 0;
      loRa.macCommands[loRa.crtMacCmdIndex].channelFrequencyAck = 0;

      //Reply has the same value as request
      loRa.macCommands[loRa.crtMacCmdIndex].receivedCid = *ptr;

      switch(*ptr++)
        {
          case LINK_CHECK_CID:
            {
              // No reply to server is needed 
              loRa.macCommands[loRa.crtMacCmdIndex].receivedCid = INVALID_VALUE;
            }
            break;

          case LINK_ADR_CID:
            {
              ptr = ExecuteLinkAdr(ptr);
            }
            break;

          case DUTY_CYCLE_CID:
            {
              ptr = ExecuteDutyCycle(ptr);
            }
            break;

          case RX2_SETUP_CID:
            {
              ptr = ExecuteRxParamSetupReq(ptr);
            }
            break;

          case DEV_STATUS_CID:
            {
              ptr = ExecuteDevStatus(ptr);
            }
            break;

          case NEW_CHANNEL_CID:
            {
              ptr = ExecuteNewChannel(ptr);

            }
            break;

          case RX_TIMING_SETUP_CID:
            {
              ptr = ExecuteRxTimingSetup(ptr);
            }
            break;

          default:
            {
              done = true; // Unknown MAC commands cannot be skipped and the first unknown MAC command terminates the processing of the MAC command sequence.
              ptr = buffer + fOptsLen;
              loRa.macCommands[loRa.crtMacCmdIndex].receivedCid = INVALID_VALUE;
            }
            break;
        }

      if((loRa.macCommands[loRa.crtMacCmdIndex].receivedCid != INVALID_VALUE) &&
         (loRa.crtMacCmdIndex < MAX_NB_CMD_TO_PROCESS))
        {
          loRa.crtMacCmdIndex++;
        }
    }
  return ptr;
}

static uint8_t* ExecuteRxTimingSetup(uint8_t *ptr)
{
  uint8_t delay;

  delay = (*ptr) & LAST_NIBBLE;
  ptr++;

  UpdateReceiveDelays(delay);
  loRa.macStatus.rxTimingSetup = ENABLED;

  return ptr;
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

static void ConfigureRadioRx(uint8_t dataRate, uint32_t freq)
{
  ConfigureRadio(dataRate, freq);
  RADIO_SetCRC(DISABLED);
  RADIO_SetIQInverted(ENABLED);
}

static void UpdateReceiveDelays(uint8_t delay)
{
 }

static void UpdateJoinInProgress(uint8_t state)
{
  // set the states and flags accordingly
  loRa.macStatus.networkJoined = 0; //last join (if any) is not considered any more, a new join is requested
  loRa.lorawanMacStatus.joining = ENABLED;
  loRa.macStatus.macState = state;
}

static void PrepareSessionKeys(uint8_t* sessionKey, uint8_t* appNonce, uint8_t* networkId)
{
  uint8_t index = 0;

  memset(&sessionKey[index], 0, sizeof(aesBuffer)); //appends 0-es to the buffer so that pad16 is done
  index++; // 1 byte for 0x01 or 0x02 depending on the key type

  memcpy(&sessionKey[index], appNonce, JA_APP_NONCE_SIZE);
  index = index + JA_APP_NONCE_SIZE;

  memcpy(&sessionKey[index], networkId, JA_NET_ID_SIZE);
  index = index + JA_NET_ID_SIZE;

  memcpy(&sessionKey[index], &loRa.devNonce, sizeof(loRa.devNonce));

}

static void ComputeSessionKeys(JoinAccept_t *joinAcceptBuffer)
{
  PrepareSessionKeys(loRa.activationParameters.applicationSessionKey, joinAcceptBuffer->members.appNonce, joinAcceptBuffer->members.networkId);
  loRa.activationParameters.applicationSessionKey[0] = 0x02; // used for Network Session Key
  AESEncodeLoRa(loRa.activationParameters.applicationSessionKey, loRa.activationParameters.applicationKey);

  PrepareSessionKeys(loRa.activationParameters.networkSessionKey, joinAcceptBuffer->members.appNonce, joinAcceptBuffer->members.networkId);
  loRa.activationParameters.networkSessionKey[0] = 0x01; // used for Network Session Key
  AESEncodeLoRa(loRa.activationParameters.networkSessionKey, loRa.activationParameters.applicationKey);
}

//Based on the last packet received, this function checks the flags and updates the state accordingly

static void CheckFlags(Hdr_t* hdr)
{
  if(hdr->members.fCtrl.adr == ENABLED)
    {
      loRa.macStatus.adr = ENABLED;
    }

  if(hdr->members.fCtrl.fPending == ENABLED)
    {
      loRa.lorawanMacStatus.fPending = ENABLED;
    }

  if(hdr->members.fCtrl.adrAckReq == ENABLED)
    {
      loRa.lorawanMacStatus.adrAckRequest = ENABLED;
    }

  if(hdr->members.mhdr.bits.mType == FRAME_TYPE_DATA_CONFIRMED_DOWN)
    {
      loRa.lorawanMacStatus.ackRequiredFromNextUplinkMessage = ENABLED; //next uplink packet should have the ACK bit set
    }
}

static uint8_t CheckMcastFlags(Hdr_t* hdr)
{

  /*
   * Multicast message conditions:
   * - ACK == 0
   * - ADRACKReq == 0
   * - FOpt == empty => FOptLen == 0 (no mac commands in fopt)
   * - FPort == 0 (no commands in payload)
   * - MType == UNCNF Data downlink
   */

  if((0 != hdr->members.fCtrl.ack) || (0 != hdr->members.fCtrl.adrAckReq) || (FRAME_TYPE_DATA_UNCONFIRMED_DOWN != hdr->members.mhdr.bits.mType))
    {
      return FLAG_ERROR;
    }

  if(0 != hdr->members.fCtrl.fOptsLen)
    {
      return FLAG_ERROR;
    }
  else
    {
      if(0 == *(((uint8_t *) hdr) + 8)) // Port vlaue in case of FOpts empty
        {
          return FLAG_ERROR;
        }
    }

  if(hdr->members.fCtrl.fPending == ENABLED)
    {
      loRa.lorawanMacStatus.fPending = ENABLED;
    }

  return FLAG_OK;
}

static uint8_t CountfOptsLength(void)
{
  uint8_t i, macCommandLength = 0;

  for(i = 0; i < loRa.crtMacCmdIndex; i++)
    {
      if(loRa.macCommands[i].receivedCid != INVALID_VALUE)
        {
          if((macCommandLength + macEndDevCmdReplyLen[loRa.macCommands[i].receivedCid - 2]) <= MAX_FOPTS_LEN)
            {
              macCommandLength += macEndDevCmdReplyLen[loRa.macCommands[i].receivedCid - 2];
            }
          else
            {
              break;
            }
        }
    }

  return macCommandLength;
}

static void AssembleEncryptionBlock(uint8_t dir, uint32_t frameCounter, uint8_t blockId, uint8_t firstByte, uint8_t multicastStatus)
{
  uint8_t bufferIndex = 0;

  memset(aesBuffer, 0, sizeof(aesBuffer)); //clear the aesBuffer

  aesBuffer[bufferIndex] = firstByte;

  bufferIndex = bufferIndex + 5; // 4 bytes of 0x00 (done with memset at the beginning of the function)

  aesBuffer[bufferIndex++] = dir;

  if(DISABLED == multicastStatus)
    {
      memcpy(&aesBuffer[bufferIndex], &loRa.activationParameters.deviceAddress, sizeof(loRa.activationParameters.deviceAddress));
      bufferIndex = bufferIndex + sizeof(loRa.activationParameters.deviceAddress);
    }
  else
    {
      memcpy(&aesBuffer[bufferIndex], &loRa.activationParameters.mcastDeviceAddress, sizeof(loRa.activationParameters.mcastDeviceAddress));
      bufferIndex = bufferIndex + sizeof(loRa.activationParameters.mcastDeviceAddress);
    }

  memcpy(&aesBuffer[bufferIndex], &frameCounter, sizeof(frameCounter));
  bufferIndex = bufferIndex + sizeof(frameCounter);

  bufferIndex++; // 1 byte of 0x00 (done with memset at the beginning of the function)

  aesBuffer[bufferIndex] = blockId;
}

static uint32_t ExtractMic(uint8_t *buffer, uint8_t bufferLength)
{
  uint32_t mic = 0;
  memcpy(&mic, &buffer[bufferLength - 4], sizeof(mic));
  return mic;
}

static uint32_t ComputeMic(uint8_t *key, uint8_t* buffer, uint8_t bufferLength) // micType is 0 for join request and 1 for data packet
{
  uint32_t mic = 0;

  AESCmac(key, aesBuffer, buffer, bufferLength); //if micType is 0, bufferLength the same, but for data messages bufferLength increases with 16 because a block is added

  memcpy(&mic, aesBuffer, sizeof( mic));

  return mic;
}

static void EncryptFRMPayload(uint8_t* buffer, uint8_t bufferLength, uint8_t dir, uint32_t frameCounter, uint8_t* key, uint8_t macBufferIndex, uint8_t* bufferToBeEncrypted, uint8_t multicastStatus)
{
  uint8_t k = 0, i = 0, j = 0;

  k = bufferLength / AES_BLOCKSIZE;
  for(i = 1; i <= k; i++)
    {
      AssembleEncryptionBlock(dir, frameCounter, i, 0x01, multicastStatus);
      AESEncodeLoRa(aesBuffer, key);

      for(j = 0; j < AES_BLOCKSIZE; j++)
        {
          bufferToBeEncrypted[macBufferIndex++] = aesBuffer[j] ^ buffer[AES_BLOCKSIZE * (i - 1) + j];
        }
    }

  if((bufferLength % AES_BLOCKSIZE) != 0)
    {
      AssembleEncryptionBlock(dir, frameCounter, i, 0x01, multicastStatus);
      AESEncodeLoRa(aesBuffer, key);

      for(j = 0; j < (bufferLength % AES_BLOCKSIZE); j++)
        {
          bufferToBeEncrypted[macBufferIndex++] = aesBuffer[j] ^ buffer[(AES_BLOCKSIZE * k) + j];
        }
    }
}

uint8_t LORAWAN_GetState(void)
{
  return loRa.macStatus.macState;
}

void LORAWAN_Mainloop(void)
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
