#include "event.h"

extern bool CLEARED = true;

extern void phyphox_event_received(){
    printk("PHYPHOX EVENTHANDLER\r\n");
    /*
    00: Pause
    01: Play
    02: Clear
    FF: Sync
    */

    if(event_data.config[0]==0x01 && CLEARED){
        global_timestamp = k_uptime_ticks()/32768.0;
        CLEARED=false;
    }
    if(event_data.config[0]==0x02){
        CLEARED=true;
    }    
};