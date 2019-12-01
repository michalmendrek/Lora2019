#include "LoraSystenm.h"
#include <stdio.h>
#include <stdlib.h>
#include "LoraSystem.h"

LORA * lora_open(ChannelConfig *config, uint8_t config_len)
{
  LORA *ptr = (LORA *) malloc(sizeof(LORA));
  memcpy(ptr->CHANNELS, config, sizeof(config) * config_len);

  RADIO_Init(ptr->DataToSend, ptr->CHANNELS->frequency);
  ptr->Flags[0] = true;
  return ptr;
}

void lora_close(LORA *ptr)
{
  //terminowanie radiowych rzeczy i innych
  ptr->Flags[0] = false;
  free(ptr);
  ptr = (LORA*) NULL;
}

uint8_t lora_send(LORA *ptr, uint8_t channel, uint8_t power, uint8_t *data, uint8_t len)
{
  if(!ptr->Flags[0])
    {
      return 0;
    }
  else
    {
      if(RADIO_Transmit(data, len) == OK)
        {
          return 1;
        }
    }


  return 0;
}
