#include "mcc_generated_files/mcc.h"
#include "LoraSystenm.h"

void main(void)
{

    SYSTEM_Initialize();

    LORA *ptr;
    //ptr = lora_open();
    if(ptr!=NULL)
      {
        
      }
    memcpy((char *)ptr->DataToSend, "DUPSKO",sizeof("DUPSKO"));
   //if(!lora_send(ptr)){}

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    while (1)
    {
        // Add your application code
    }
}
/**
 End of File
*/