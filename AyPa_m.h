
#define Pin2Input(port,pin){port&=~(1<<pin);}
#define Pin2Output(port,pin){port|=(1<<pin);}
#define Pin2HIGH(port,pin){port|=(1<<pin);}
#define Pin2LOW(port,pin){port&=~(1<<pin);}
  //\
//#if (state==INPUT) port|=(1<<pin);\
//#elif port&=~(1<<pin);\
//#endif}
