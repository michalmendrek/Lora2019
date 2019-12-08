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
 *                           lorawan_private.h
 *
 * LoRaWAN private header file
 *
 ******************************************************************************/

#ifndef _LORAWAN_PRIVATE_H
#define	_LORAWAN_PRIVATE_H

#ifdef	__cplusplus
extern "C"
{
#endif

  /****************************** INCLUDES **************************************/
#include "lorawan_defs.h"

  /****************************** DEFINES ***************************************/

#define INVALID_VALUE                           0xFF

#define ENABLED                                 1
#define DISABLED                                0

#define ACCEPTED                                1
#define REJECTED                                0

#define RESPONSE_OK                             1
#define RESPONSE_NOT_OK                         0

#define FLAG_ERROR                              0
#define FLAG_OK                                 1

#define MCAST_ENABLED                           1
#define MCAST_DISABLED                          0

#define RESERVED_FOR_FUTURE_USE                 0

#define MAXIMUM_BUFFER_LENGTH                   271

#define CLASS_C_RX_WINDOW_SIZE                  0

  //Data rate (DR) encoding 
#define DR0                                     0  
#define DR1                                     1  
#define DR2                                     2  
#define DR3                                     3  
#define DR4                                     4  
#define DR5                                     5
#define DR6                                     6  
#define DR7                                     7
#define DR8                                     8
#define DR9                                     9
#define DR10                                    10
#define DR11                                    11
#define DR12                                    12
#define DR13                                    13 
#define DR14                                    14
#define DR15                                    15    

#define MAJOR_VERSION3                          0

#define LAST_NIBBLE                             0x0F
#define FIRST_NIBBLE                            0xF0 

  // bit mask for MAC commands
  // link ADR and RX2 setup
#define CHANNEL_MASK_ACK                        0x01
#define DATA_RATE_ACK                           0x02
#define RX1_DR_OFFSET_ACK                       0x04
#define POWER_ACK                               0x04

#define FPORT_MIN                               1
#define FPORT_MAX                               223

  //#define MAX_NB_CMD_TO_PROCESS                   16     

  //13 = sizeof(MIC) + MHDR + FHDR + sizeof (fPort);
#define HDRS_MIC_PORT_MIN_SIZE 13 

#define ABP_TIMEOUT_MS                          20    

#define JA_APP_NONCE_SIZE                       3    
#define JA_NET_ID_SIZE                          3

#define MAX_FOPTS_LEN                           0x0F

  /***************************** TYPEDEFS ***************************************/

  typedef union
  {
    uint8_t value;

    struct
    {
      unsigned nbRep : 4;
      unsigned chMaskCntl : 3;
      unsigned rfu : 1;
    };
  } Redundancy_t;

  typedef enum
  {
    LoRa_Idle = 0,
    LoRa_Handshaking_TX,
    LoRa_Handshaking_RX,
    LoRa_SendData_TX,
    LoRa_SendData_RX,
    LoRa_Sent,
    LoRa_SendFailed,
    LoRa_Wait_retransmit,
    LoRa_transmit_Error,
            ToDo_LoRa_retransmit,
            ToDo_LoRa_retransmit_radio_configured,
    //    LoRa_transmit_OK,
    //    LoRa_Transmit_Fail
  } LoRaTransmitState_t;

  typedef enum
  {
    LoRa_transmitIdle = 0,
    LoRa_transmiting,
    LoRa_transmit_OK,
    LoRa_transmit_Fail
  } LoRaStatus_t;

  // types of frames

  typedef enum
  {
    FRAME_TYPE_JOIN_REQ = 0x00,
    FRAME_TYPE_JOIN_ACCEPT,
    FRAME_TYPE_DATA_UNCONFIRMED_UP,
    FRAME_TYPE_DATA_UNCONFIRMED_DOWN,
    FRAME_TYPE_DATA_CONFIRMED_UP,
    FRAME_TYPE_DATA_CONFIRMED_DOWN,
    FRAME_TYPE_RFU,
    FRAME_TYPE_PROPRIETARY,
  } LoRaMacFrameType_t;

  // MAC commands CID

  typedef enum
  {
    LINK_CHECK_CID = 0x02,
    LINK_ADR_CID = 0x03,
    DUTY_CYCLE_CID = 0x04,
    RX2_SETUP_CID = 0x05,
    DEV_STATUS_CID = 0x06,
    NEW_CHANNEL_CID = 0x07,
    RX_TIMING_SETUP_CID = 0x08,
  } LoRaMacCid_t;

  //activation parameters

  typedef union
  {
    uint32_t value;
    uint8_t buffer[4];
  } DeviceAddress_t;

  typedef union
  {
    uint32_t value;

    struct
    {
      uint16_t valueLow;
      uint16_t valueHigh;
    } members;
  } FCnt_t;

  //union used for instantiation of DeviceEui and Application Eui

  typedef union
  {
    uint8_t buffer[8];

    struct
    {
      uint32_t genericEuiL;
      uint32_t genericEuiH;
    } members;
  } GenericEui_t;

  typedef struct
  {
    ActivationType_t activationType;
    DeviceAddress_t deviceAddress;
    uint8_t networkSessionKey[16];
    uint8_t applicationSessionKey[16];
    uint8_t applicationKey[16];
    GenericEui_t applicationEui;
    GenericEui_t deviceEui;
    DeviceAddress_t mcastDeviceAddress;
    uint8_t mcastNetworkSessionKey[16];
    uint8_t mcastApplicationSessionKey[16];
  } ActivationParameters_t;

  typedef union
  {
    uint8_t value;

    struct
    {
      unsigned fOptsLen : 4;
      unsigned fPending : 1;
      unsigned ack : 1;
      unsigned adrAckReq : 1;
      unsigned adr : 1;
    };
  } FCtrl_t;
  
   //Protocol parameters

  typedef struct
  {
    uint16_t receiveDelay1;
    uint16_t receiveDelay2;
    uint16_t joinAcceptDelay1;
    uint16_t joinAcceptDelay2;
    uint16_t maxFcntGap;
    uint16_t maxMultiFcntGap;
    uint16_t ackTimeout;
    uint8_t adrAckLimit;
    uint8_t adrAckDelay;
  } ProtocolParams_t;

    typedef struct
  {
    uint32_t frequency;
    uint8_t dataRate;
  } ReceiveWindowParameters_t;

  // minimum and maximum data rate

  typedef union
  {
    uint8_t value;

    struct
    {
      unsigned min : 4;
      unsigned max : 4;
    };
  } DataRange_t;

  extern uint8_t LoRa_radioBuffer[];

  /*************************** FUNCTIONS PROTOTYPE ******************************/

  // Callback functions

  void LoRa_TimerHandshakingCallback_XYfe(uint8_t param);
  void LoRa_TimerRetransmitCallback_XYfl(uint8_t param);
  void LoRa_TimerWaitAckCallback_XYfl(uint8_t param);

  void LoRa_EnterReceive_XYfe_HD(void);

  void LoRa_UpdateMinMaxChDataRate_Yf(void);

  // Update and validation functions

  void LoRa_UpdateCurrentDataRate(uint8_t valueNew);

  void LoRa_UpdateTxPower(uint8_t txPowerNew);


  LorawanError_t LoRa_ValidateDataRate(uint8_t dataRate);

  LorawanError_t LoRa_ValidateTxPower(uint8_t txPowerNew);

  //Initialization functions

  LorawanError_t LoRa_SelectChannelForTransmission_XYf(uint8_t channelTx, uint8_t channelRx);

  uint16_t Random(uint16_t max);

    //MAC commands functions

  void LoRa_ConfigureRadio_XYfe(uint8_t dataRate, uint32_t freq);


 #ifdef	__cplusplus
}
#endif

#endif	/* _LORAWAN_PRIVATE_H */