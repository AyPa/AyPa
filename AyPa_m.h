//macro

#define Pin2Input(port,pin){port&=~(1<<pin);}
#define Pin2Output(port,pin){port|=(1<<pin);}
#define Pin2HIGH(port,pin){port|=(1<<pin);}
#define Pin2LOW(port,pin){port&=~(1<<pin);}
  //\
//#if (state==INPUT) port|=(1<<pin);\
//#elif port&=~(1<<pin);\
//#endif}

#define NOP __asm__ __volatile__ ("nop\n\t")
#define wait1us NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP; // 1 microsecond delay on a 20MHz Arduino
//#define wait500ns NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP; // 500ns delay on a 16MHz Arduino
//#define wait250ns NOP;NOP;NOP;NOP; // 250ns delay on a 16MHz Arduino
//#define wait125ns NOP;NOP; // 125ns delay on a 16MHz Arduino
//#define wait63ns NOP; // 63ns delay on a 16MHz Arduino

