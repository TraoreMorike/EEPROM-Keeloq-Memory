#ifndef __ATEEPROM_H__
#define __ATEEPROM_H__

#include "Arduino.h"


// Pin Definitions
//#define EEPROM_CS_PIN   9
//#define EEPROM_SK_PIN   8
//#define EEPROM_DI_PIN   6
//#define EEPROM_DO_PIN   7


// AT93C46D EEPROM OP Codes
#define EEPROM_READ   0b110
#define EEPROM_EWEN   0b100
#define EEPROM_ERASE  0b111
#define EEPROM_WRITE  0b101
#define EEPROM_ERAL   0b100
#define EEPROM_EWDS   0b100
#define EEPROM_WRAL   0b100

// Macros
#define EEPROM_NUM_OF_SLOT      64
#define EEPROM_SLOT_BIT_SIZE    16
#define EEPROM_ADDR_MIN         0x00
#define EEPROM_ADDR_MAX         0x3F

// Base address for each remote
/*
// Memory Map
For information, the AT93C46D EEPROM has 64 slots, each slot has 16 bits.
A 1K bit EEPROM is organized as 64 x 16. 
The first 9 locations are reserved for the remote 0, 
the next 9 locations are reserved for the remote 1, and so on.

A Total of 7 remotes can be stored in the EEPROM.
_____________________________
|BASE_ADDR | 0x00             |
_____________________________
|KEY_0     | BASE_ADDR + 0x00 |
.............................   
|KEY_1     | BASE_ADDR + 0x01 |
.............................
|KEY_2     | BASE_ADDR + 0x02 |
.............................
|KEY_3     | BASE_ADDR + 0x03 |
.............................
|SYNC      | BASE_ADDR + 0x04 |
.............................
|SER_0_LSB | BASE_ADDR + 0x05 |
.............................
|SER_1_MSB | BASE_ADDR + 0x06 |
.............................
|SEED_0_LSB| BASE_ADDR + 0x07 |
.............................
|SEED_1_MSB| BASE_ADDR + 0x08 |
.............................
*/ 

#define REMOTE_0_BASE_ADDR      0x00
#define REMOTE_1_BASE_ADDR      0x0A
#define REMOTE_2_BASE_ADDR      0x14

#define KEY_0_ADDR(remote_base_addr)        ((remote_base_addr) + (0x00))
#define KEY_1_ADDR(remote_base_addr)        ((remote_base_addr) + (0x01))
#define KEY_2_ADDR(remote_base_addr)        ((remote_base_addr) + (0x02))
#define KEY_3_ADDR(remote_base_addr)        ((remote_base_addr) + (0x03))
#define SYNC_ADDR(remote_base_addr)         ((remote_base_addr) + (0x04))
#define SER_0_LSB_ADDR(remote_base_addr)    ((remote_base_addr) + (0x05))
#define SER_1_MSB_ADDR(remote_base_addr)    ((remote_base_addr) + (0x06))
#define SEED_0_LSB_ADDR(remote_base_addr)   ((remote_base_addr) + (0x07))
#define SEED_1_MSB_ADDR(remote_base_addr)   ((remote_base_addr) + (0x08))


class ATEEPROM;

class ATEEPROM {
    public:
        ATEEPROM(uint8_t cs, uint8_t sk, uint8_t di, uint8_t dout);
        void init();

        void readKey(uint8_t remote_base_addr, uint32_t* key_msb, uint32_t* key_lsb);
        void writeKey(uint8_t remote_base_addr, uint32_t key_msb, uint32_t key_lsb);
        void eraseKey(uint8_t remote_base_addr);

        void readSync(uint8_t remote_base_addr, uint32_t* sync);
        void writeSync(uint8_t remote_base_addr, uint32_t sync);
        void eraseSync(uint8_t remote_base_addr);

        void readSerial(uint8_t remote_base_addr, uint32_t* serial_msb, uint32_t* serial_lsb);
        void writeSerial(uint8_t remote_base_addr, uint16_t serial_msb, uint16_t serial_lsb);
        void eraseSerial(uint8_t remote_base_addr);

        int isSerialAuthorized(uint32_t serial);

        void readSeed(uint8_t remote_base_addr, uint32_t* seed_msb, uint32_t* seed_lsb);
        void writeSeed(uint8_t remote_base_addr, uint16_t seed_msb, uint16_t seed_lsb);
        void eraseSeed(uint8_t remote_base_addr);

        void eraseRemote(uint8_t remote_base_addr);

        
        
        

    private:
        void enableEEPROMWrite();
        void EEPROMDisableWrite();
        void EEPROMWriteSlot(uint8_t address, uint16_t data);
        uint16_t EEPROMReadSlot(uint8_t address);
        void EEPROMEraseSlot(uint8_t address);
        void EEPROMEraseAll();
        void EEPROMWriteAll(uint16_t data);

        void bitbang_write(uint16_t data, uint8_t dataLength);
        uint32_t bitbang_read(uint8_t readLength);
        uint16_t findMSB(uint16_t num);
        
        uint8_t _EEPROM_CS_PIN;
        uint8_t _EEPROM_SK_PIN;
        uint8_t _EEPROM_DI_PIN;
        uint8_t _EEPROM_DO_PIN;

        uint8_t _REMOTE_BASE_ADDR_LIST[3] = {REMOTE_0_BASE_ADDR, REMOTE_1_BASE_ADDR, REMOTE_2_BASE_ADDR};

        
};


#endif
