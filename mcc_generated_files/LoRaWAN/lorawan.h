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
 *                           lorawan.h
 *
 * LoRaWAN header file
 *
 ******************************************************************************/

#ifndef _LORAWAN_H
#define	_LORAWAN_H

#ifdef	__cplusplus
extern "C"
{
#endif



  /****************************** INCLUDES **************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <xc.h>

  /****************************** DEFINES ***************************************/
#define LoRaDeviceAddress   0xaf
  // Recommended protocol parameters
#define RECEIVE_DELAY1                              1000UL
#define RECEIVE_DELAY2                              2000UL
#define JOIN_ACCEPT_DELAY1                          5000UL
#define JOIN_ACCEPT_DELAY2                          6000UL
#define MAX_FCNT_GAP                                16384
#define MAX_MCAST_FCNT_GAP                          MAX_FCNT_GAP
#define ADR_ACK_LIMIT                               64
#define ADR_ACK_DELAY                               32
#define ACK_TIMEOUT                                 2000

#define LoRa_Chann_nr   4   //ilosc kanalow LoRa

  /***************************** TYPEDEFS ***************************************/
  typedef enum
  {
    OK = 0,
    LoRa_Send_problem,
    NETWORK_NOT_JOINED,
    MAC_STATE_NOT_READY_FOR_TRANSMISSION,
    INVALID_PARAMETER,
    KEYS_NOT_INITIALIZED,
    SILENT_IMMEDIATELY_ACTIVE,
    FRAME_COUNTER_ERROR_REJOIN_NEEDED,
    INVALID_BUFFER_LENGTH,
    MAC_PAUSED,
    NO_CHANNELS_FOUND,
    INVALID_CLASS,
    MCAST_PARAM_ERROR,
    MCAST_MSG_ERROR,
  } LorawanError_t;

  typedef enum
  {
    MAC_NOT_OK = 0, //LoRaWAN operation failed
    MAC_OK, //LoRaWAN operation successful
    RADIO_NOT_OK, //Radio operation failed
    RADIO_OK, //Radio operation successful
    INVALID_BUFFER_LEN, //during retransmission, we have changed SF and the buffer is too large
    MCAST_RE_KEYING_NEEDED
  } OpStatus_t;

  typedef enum
  {
    OTAA = 0, //LoRaWAN Over The Air Activation - OTAA
    ABP //LoRaWAN Activation By Personalization - ABP
  } ActivationType_t;

  typedef enum
  {
    UNCNF = 0, //LoRaWAN Unconfirmed Transmission
    CNF //LoRaWAN Confirmed Transmission
  } TransmissionType_t;

  typedef enum
  {
    ISM_EU868,
    ISM_EU433
  } IsmBand_t;

  typedef enum
  {
    CLASS_A = 0,
    CLASS_B,
    CLASS_C,
  } LoRaClass_t;

  typedef union
  {
    uint32_t value;

    struct
    {
      unsigned macState : 4; //determines the state of transmission (rx window open, between tx and rx, etc)
      unsigned networkJoined : 1; //if set, the network is joined
      unsigned automaticReply : 1; //if set, ACK and uplink packets sent due to  FPending will be sent immediately
      unsigned adr : 1; //if set, adaptive data rate is requested by server or application
      unsigned silentImmediately : 1; //if set, the Mac command duty cycle request was received
      unsigned macPause : 1; //if set, the mac Pause function was called. LoRa modulation is not possible
      unsigned rxDone : 1; //if set, data is ready for reception
      unsigned linkCheck : 1; //if set, linkCheck mechanism is enabled
      unsigned channelsModified : 1; //if set, new channels are added via CFList or NewChannelRequest command or enabled/disabled via Link Adr command
      unsigned txPowerModified : 1; //if set, the txPower was modified via Link Adr command
      unsigned nbRepModified : 1; //if set, the number of repetitions for unconfirmed frames has been modified
      unsigned prescalerModified : 1; //if set, the prescaler has changed via duty cycle request
      unsigned secondReceiveWindowModified : 1; //if set, the second receive window parameters have changed
      unsigned rxTimingSetup : 1; //if set, the delay between the end of the TX uplink and the opening of the first reception slot has changed
      unsigned rejoinNeeded : 1; //if set, the device must be rejoined as a frame counter issue happened
      unsigned mcastEnable : 1; //if set, the device is in multicast mode and can receive multicast messages
    };
  } LorawanStatus_t;

  /*************************** FUNCTIONS PROTOTYPE ******************************/

  typedef void (*RxAppDataCb_t)(uint8_t* pData, uint8_t dataLength, OpStatus_t status);
  typedef void (*RxJoinResponseCb_t)(bool status);

  // Initialization functions
  /**
   * @Summary
      Bidirectional communication start.
   * @Description
      This function starts a bidirectional communication process.
   * @Preconditions
      None
   * @Param
      buffer - a data buffer used to store the data to be sent
      bufferLength - the length in bytes of the data buffer (uint8_t)
   * @Returns
      Function returns the status of the operation (LorawanError_t).
   * @Example
      uint8_t dataToSend = 45;
      LORAWAN_Send (UNCNF, 20, &dataToSend, sizeof(dataToSend));
   */
  LorawanError_t LoRa_Send(void *buffer, uint8_t bufferLength);

  /**
   * @Summary
      LoRaWAN Initialization function
   * @Description
      This function initializes LoRaWAN stack and the radio module.
   * @Preconditions
      None
   * @Param
      RxPayload - pointer to function that gets called after the bidirectional communication ended.
   * @Return
      None
   * @Example
   */
  void LoRa_System_Init(void);

  void LoRa_Reset(IsmBand_t ismBandNew);

  /**
   * @Summary
      Sets the data rate for the next uplink.
   * @Description
      Communication between end-devices and gateways is spread out on different
      frequency channels and data rates.
      The selection of the data rate is a trade-off between communication range and
      message duration, communications with different data rates do not interfere with each other.
   * @Preconditions
      None
   * @Param
      valueNew - new data rate value
   * @Returns
      Return LoRaWAN Error type (LorawanError_t).
   * @Example
   */
  LorawanError_t LoRa_SetCurrentDataRate(uint8_t valueNew);

  /**
   * @Summary
      Sets TX output power.
   * @Description
      The TX output power (TXPower) is region-specific.
      txPowerNew must be provided as an index between 0 - 15.
      For more details please refer to LoRaWAN Specification V1.0 document.
   * @Preconditions
      None
   * @Param
      txPowerNew - new TX power value
   * @Returns
      Return LoRaWAN Error type (LorawanError_t).
   * @Example
   */
  LorawanError_t LoRa_SetTxPower(uint8_t txPowerNew);

  /**
   * @Summary
      Returns TX output power.
   * @Description
      The TX output power (TXPower) is region-specific.
      Tx Power is returned as an index between 0 - 15.
      For more details please refer to LoRaWAN Specification V1.0 document.
   * @Preconditions
      None
   * @Param
      None
   * @Returns
      Returns TX output power value (uint8_t).
   * @Example
   */
  uint8_t LoRa_GetTxPower(void);

  /**
   * @Summary
      Sets the synchronization word.
   * @Description
      This function sets the current synchronization word used during the communication.
      For more details please refer to LoRaWAN Specification V1.0 document.
   * @Preconditions
      None
   * @Param
      syncWord - the value for the new sync word
   * @Returns
      None
   * @Example
   */
  void LoRa_SetSyncWord(uint8_t syncWord);

  /**
   * @Summary
      Returns the synchronization word.
   * @Description
      This function returns the current synchronization word used during the communication.
      For more details please refer to LoRaWAN Specification V1.0 document.
   * @Preconditions
      None
   * @Param
      None
   * @Returns
      The value of the sync word (uint8_t).
   * @Example
   */
  uint8_t LoRa_GetSyncWord(void);

  /**
   * @Summary
    Function sets the current downlink counter.
   * @Description
    This function sets the current downlink counter used during the communication.
    This may be used to synchronize the downlink counter with the value stored by the server.
   * @Preconditions
    None
   * @Param
    ctr - value of the new counter
   * @Returns
    None
   * @Example
   */
  void LoRa_SetCounter(uint32_t ctr);

  /**
   * @Summary
      Function returns the current downlink counter.
   * @Description
      This function returns the current downlink counter used during the communication.
      This may be used to synchronize the downlink counter with the value stored by the server.
   * @Preconditions
      None
   * @Param
      None
   * @Returns
      Current downlink counter (uint32_t).
   * @Example
   */
  uint32_t LoRa_GetCounter(void);


  /**
   * @Summary
      Function sets battery level.
   * @Description
      This function sets the battery level required for Device Status Answer frame in use with the LoRaWAN protocol.
      The level is a decimal number representing the level of the battery, from 0 to 255.
      0 means external power, 1 means low level, 254 means high level,
      255 means the end device was not able to measure the battery level.
   * @Preconditions
      None
   * @Param
      batteryLevelNew - the new level value
   * @Returns
      None
   * @Example
   */
  void LoRa_SetBattery(uint8_t batteryLevelNew);

  /**
   * @Summary
      Function returns the frequency of the given channel.
   * @Description
      This command returns the frequency on the requested "channelId", entered in decimal form.
   * @Preconditions
      None
   * @Param
      channelId - the channel requested
   * @Returns
      The frequency of the given channel (value returned is in Hz).
   * @Example
   */
  uint32_t LoRa_GetFrequency(uint8_t channelId);

  /**
   * @Summary
      Function sets new data range for the given channel.
   * @Description
      This function sets the operating data rate range, minimum to maximum, for the given "channelId".
      By doing this the module can vary data rates between the minimum range and maximum range.
      Please refer to the LoRaWAN Specification V1.0 for the actual values of the data rates and the corresponding
      spreading factors (SF).
   * @Preconditions
      None
   * @Param
      channelId - the channel we change
      dataRangeNew - the first four, MSB, are representing the maximum value and the last four, LSB, are the minimum value.
   * @Returns
      none
   * @Example
      setting channel 13 data range between 1 and 3:

      <code>
      ...
      MacSetDataRange(13, 0x31); // ( 0x31 -> 0011 0001)
      ...
      <code>
   */
  LorawanError_t LoRa_SetDataRange(uint8_t channelId, uint8_t dataRangeNew);

  /**
   * @Summary
      Function returns the data range of a given channel.
   * @Description
      This function returns the operating data rate range, minimum to maximum, for the given "channelId".
   * @Preconditions
      None
   * @Param
      channelId - the given channel
   * @Returns
      Returns the minimum and maximum data range (uint8_t).
      The first four bits, MSB, are representing the maximum value and the last four bits, LSB, are the minimum value.
   * @Example
   */
  uint8_t LoRa_GetDataRange(uint8_t channelId);

  /**
   * @Summary
    Sets the frequency of the given channel.
   * @Description
    This function sets the frequency on the requested "channelId" to a new value.
   * @Preconditions
    None
   * @Param
    channelId - the given channel
    frequencyNew - the new frequency value (the value must be provided in Hz).
   * @Returns
    Returns LoRaWAN Error type (LorawanError_t)
   * @Example
   */
  LorawanError_t LoRa_SetFrequency(uint8_t channelId, uint32_t frequencyNew);

  /**
   * @Summary
      Function returns the configured ISM Band.
   * @Description
      This function returns the ISM Band.
   * @Preconditions
      None
   * @Param
      None
   * @Returns
      Returns ISB Band Type (IsmBand_t)                       
   * @Example
   */
  uint8_t LoRa_GetIsmBand(void);

  void LoRa_PrepareRetransmit(void);
  
  void LoRa_TxWdtTimeout(void);
  void LoRa_RxWdtTimeout(void);


  /**
   * @Summary
      LoRaWAN Mainloop function.
   * @Description
      This function is used for running the system timers and check the DIO pins.
      It must be called in the while(1) loop inside <main> function (once per loop).
   * @Preconditions
      None
   * @Param
      none
   * @Returns
      none                       
   * @Example
   */
  void LoRa_Mainloop(void);

#ifdef	__cplusplus
}
#endif

#endif	/* _LORAWAN_H */



