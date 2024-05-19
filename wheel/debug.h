// Debug .h file

#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

class debugClass : public HardwareSerial {

    private:
        bool DEBUG;
    
    public:
        debugClass(uint8_t);
        
        void begin(long, )
        print(const char[])
        
}

bool DEBUG;



#endif