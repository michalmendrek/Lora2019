/* 
 * File:   LoraSystenm.h
 * Author: kwisn
 *
 * Created on 28 listopada 2019, 13:14
 */

#ifndef LORASYSTENM_H
#define	LORASYSTENM_H

#ifdef	__cplusplus
extern "C"
{
#endif

#include "mcc_generated_files/mcc.h"
  
  typedef struct
{
    uint32_t frequency;
    uint32_t frequencyDeviation;
    uint32_t bitRate;
    uint16_t preambleLen;
    uint8_t syncWordLoRa;
    uint8_t syncWord[8];
    uint8_t syncWordLen;
    RadioModulation_t modulation;
    RadioDataRate_t dataRate;
    RadioLoRaBandWidth_t bandWidth;
    int8_t outputPower;
//    uint8_t crcOn;
//    uint8_t paBoost;
//    uint16_t frequencyHopPeriod;
//    uint8_t iqInverted;
//    RadioErrorCodingRate_t errorCodingRate;
//    uint8_t implicitHeaderMode;
//    uint8_t flags;
//    uint8_t dataBufferLen;
//    uint8_t *dataBuffer;
//    uint8_t timeOnAirTimerId;
//    uint8_t fskRxWindowTimerId;
//    uint8_t watchdogTimerId;
//    uint32_t watchdogTimerTimeout;
//    uint8_t initialized;
//    uint32_t (*fhssNextFrequency)(void);
//    uint8_t regVersion;
//    int8_t packetSNR;
//    RadioFSKShaping_t fskDataShaping;
//    RadioFSKBandWidth_t rxBw;
//    RadioFSKBandWidth_t afcBw;
} RadioConfiguration_t;

typedef struct __channel_config__ {
    uint32_t frequency;
    uint32_t frequencyDeviation;
    uint32_t bitRate;
    uint16_t preambleLen;
    uint8_t syncWordLoRa;
    uint8_t syncWord[8];
    uint8_t syncWordLen;
    RadioModulation_t modulation;
    RadioDataRate_t dataRate;
    RadioLoRaBandWidth_t bandWidth;
    int8_t outputPower;
} ChannelConfig;
  

typedef struct __lora__ {
  
  RadioConfiguration_t CHANNELS[5];
  uint8_t DataToSend[200];
  bool Flags[4];
  
} LORA;


LORA * lora_open(ChannelConfig config[5], uint8_t config_len);
uint8_t lora_send(LORA *ptr, uint8_t channel,uint8_t power, uint8_t *data, uint8_t len);
void lora_close(LORA *ptr);


#ifdef	__cplusplus
}
#endif

#endif	/* LORASYSTENM_H */

