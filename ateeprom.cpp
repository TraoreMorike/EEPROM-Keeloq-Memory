#include "ATEEPROM.h"
#include "Arduino.h"

//#define DEBUG
//#define DEBUG_KEY

ATEEPROM::ATEEPROM(uint8_t cs, uint8_t sk, uint8_t di, uint8_t dout)
{
    _EEPROM_CS_PIN = cs;
    _EEPROM_SK_PIN = sk;
    _EEPROM_DI_PIN = di;
    _EEPROM_DO_PIN = dout;
}

/**
 * @brief Initializes the EEPROM module.
 * 
 * This function sets the pin modes for the EEPROM control pins.
 * 
 * @param None
 * @return None
 */
void ATEEPROM::init()
{
    pinMode(_EEPROM_CS_PIN, OUTPUT);
    pinMode(_EEPROM_SK_PIN, OUTPUT);
    pinMode(_EEPROM_DO_PIN, OUTPUT);
    pinMode(_EEPROM_DI_PIN, INPUT);
}

/**
 * Enables EEPROM write operation.
 */
void ATEEPROM::enableEEPROMWrite()
{
    uint16_t buffer = 0;
    buffer = (EEPROM_EWEN << 6) | 0x3F;

    #ifdef DEBUG
    Serial.print("DEBUG enableEEPROMWrite | Buffer: ");
    Serial.println(buffer, HEX);
    #endif
  
    digitalWrite(_EEPROM_CS_PIN, HIGH);

    bitbang_write(buffer, findMSB(buffer));

    digitalWrite(_EEPROM_CS_PIN, LOW);
}

/**
 * Disables write operations on the AT93 EEPROM.
 */
void ATEEPROM::EEPROMDisableWrite()
{
    uint16_t buffer = 0;
    buffer = (EEPROM_EWDS << 6) | 0x0F;

    #ifdef DEBUG
    Serial.print("DEBUG EEPROMDisableWrite | Buffer: ");
    Serial.println(buffer, HEX);
    #endif
  
    digitalWrite(_EEPROM_CS_PIN, HIGH);

    bitbang_write(buffer, findMSB(buffer));

    digitalWrite(_EEPROM_CS_PIN, LOW);
}

/**
 * Writes data to a specific slot in the EEPROM.
 *
 * @param address The address of the slot to write to.
 * @param data The data to be written to the slot.
 */
void ATEEPROM::EEPROMWriteSlot(uint8_t address, uint16_t data)
{
    uint16_t buffer = 0;
    buffer = (EEPROM_WRITE << 6) | (address);

    #ifdef DEBUG
    Serial.print("DEBUG EEPROMWriteSlot | Buffer : ");
    Serial.println(buffer, HEX);
    #endif

    // Enable EEPROM write access
    enableEEPROMWrite();

    digitalWrite(_EEPROM_CS_PIN, HIGH);
    bitbang_write(buffer, findMSB(buffer));

    buffer = data;

    #ifdef DEBUG
    Serial.print("DEBUG EEPROMWriteSlot | Data : ");
    Serial.println(buffer, HEX);
    #endif

    bitbang_write(buffer, ((sizeof(buffer) * 8) - 1));

    digitalWrite(_EEPROM_CS_PIN, LOW);
    delay(10);

    // Disable EEPROM write access
    EEPROMDisableWrite();
}

/**
 * Reads a slot from the EEPROM.
 *
 * This function reads a slot from the EEPROM at the specified address.
 *
 * @param address The address of the slot to read.
 * @return The value read from the slot.
 */
uint16_t ATEEPROM::EEPROMReadSlot(uint8_t address)
{
    uint16_t buffer = 0;
  
    buffer = (EEPROM_READ << 6) | (address);

    #ifdef DEBUG
    Serial.print("DEBUG EEPROMReadSlot | Buffer | Address : ");
    Serial.print(buffer, HEX);
    Serial.print(" | ");
    Serial.println(address, HEX);
    #endif

    // Enable EEPROM write access
    enableEEPROMWrite();

    digitalWrite(_EEPROM_CS_PIN, HIGH);
    
    bitbang_write(buffer, findMSB(buffer));
    
    buffer = bitbang_read(sizeof(buffer) * 8);

    digitalWrite(_EEPROM_CS_PIN, LOW);

    // Disable EEPROM write access
    EEPROMDisableWrite();

    return buffer;
}

/**
 * @brief Erases a slot in the EEPROM memory.
 * 
 * This function erases a specific slot in the EEPROM memory by sending the appropriate command and address to the EEPROM chip.
 * It enables EEPROM write access, sends the erase command and address to the chip, and then disables EEPROM write access.
 * 
 * @param address The address of the slot to be erased.
 */
void ATEEPROM::EEPROMEraseSlot(uint8_t address)
{
    uint16_t buffer = 0;
    buffer = (EEPROM_ERASE << 6) | (address & 0x3F);
    
    // Enable EEPROM write access
    enableEEPROMWrite();

    digitalWrite(_EEPROM_CS_PIN, HIGH);

    bitbang_write(buffer, findMSB(buffer));
    delay(6);

    digitalWrite(_EEPROM_CS_PIN, LOW);

    // Disable EEPROM write access
    EEPROMDisableWrite();
}

/**
 * @brief Erases all data in the EEPROM.
 * 
 * This function erases all data in the EEPROM by performing the necessary steps to erase the entire memory.
 * It enables EEPROM write access, sends the erase command to the EEPROM, and then disables EEPROM write access.
 * 
 * @note This function assumes that the EEPROM is connected and properly initialized.
 */
void ATEEPROM::EEPROMEraseAll()
{
    uint16_t buffer = 0;
    buffer = (EEPROM_ERAL << 6) | 0x1F;
    
    // Enable EEPROM write access
    enableEEPROMWrite();

    digitalWrite(_EEPROM_CS_PIN, HIGH);
    
    bitbang_write(buffer, findMSB(buffer));

    digitalWrite(_EEPROM_CS_PIN, LOW);
    delay(5);

    // Disable EEPROM write access
    EEPROMDisableWrite();
}

/**
 * Writes the given data to the EEPROM.
 *
 * @param data The data to be written to the EEPROM.
 */
void ATEEPROM::EEPROMWriteAll(uint16_t data)
{
    uint16_t buffer = 0;
    buffer = (EEPROM_WRAL << 6) | 0x7F;
    
    // Enable EEPROM write access
    enableEEPROMWrite();

    digitalWrite(_EEPROM_CS_PIN, HIGH);

    bitbang_write(buffer, findMSB(buffer));
    
    
    bitbang_write(data, sizeof(data) * 8);

    digitalWrite(_EEPROM_CS_PIN, LOW);

    delay(10);
    
    // Disable EEPROM write access
    EEPROMDisableWrite();
}

/**
 * Writes data to the EEPROM using bit-banging.
 *
 * This function writes the specified data to the EEPROM using a bit-banging technique.
 * It takes the data to be written and the length of the data as parameters.
 *
 * @param data The data to be written to the EEPROM.
 * @param dataLength The length of the data in bits.
 */
void ATEEPROM::bitbang_write(uint16_t data, uint8_t dataLength)
{
  for (int i = dataLength; i >= 0; i--) {
        digitalWrite(_EEPROM_DO_PIN, ((data >> i) & 0x01) ? HIGH : LOW);
        digitalWrite(_EEPROM_SK_PIN, HIGH);
        delayMicroseconds(30);
        
        digitalWrite(_EEPROM_SK_PIN, LOW);
        digitalWrite(_EEPROM_DO_PIN, LOW);
        delayMicroseconds(30);
    }   
}

/**
 * Reads data from the EEPROM using bit-banging.
 *
 * This function reads data from the EEPROM by bit-banging the communication protocol.
 * It reads the data bit by bit, starting from the most significant bit (MSB) and moving
 * towards the least significant bit (LSB).
 *
 * @param readLength The number of bits to read from the EEPROM.
 * @return The data read from the EEPROM as a 16-bit unsigned integer.
 */
uint32_t ATEEPROM::bitbang_read(uint8_t readLength)
{
    uint32_t data = 0;
    
    for (int i = readLength -1; i >= 0; i--) {
        // Read the data bit MSB first
        delayMicroseconds(30);
        digitalWrite(_EEPROM_SK_PIN, HIGH);
        delayMicroseconds(30);
        
        if (digitalRead(_EEPROM_DI_PIN) == HIGH) {
        data |= (1 << i);
        } else {
        data &= ~(1 << i);
        }

        digitalWrite(_EEPROM_SK_PIN, LOW);
        
    }
    delayMicroseconds(30);
    
   /*
   for (int i = 0; i < readLength; i++) {
        
        delayMicroseconds(5);
        digitalWrite(_EEPROM_SK_PIN, HIGH);
        delayMicroseconds(5);
        data |= (digitalRead(_EEPROM_DI_PIN) << ((readLength - 1) - i));
        digitalWrite(_EEPROM_SK_PIN, LOW);
    }

    delayMicroseconds(5);
    */

    return data;
}

/**
 * Finds the most significant bit (MSB) position in a given unsigned integer.
 * 
 * @param num The unsigned integer to find the MSB position for.
 * @return The position of the MSB in the given unsigned integer.
 */
uint16_t ATEEPROM::findMSB(uint16_t num)
{
    int msb = -1; // Initialize msb to -1 (indicating not found)
    uint16_t bits_count = sizeof(num) * 8; // Total bits in unsigned int
    
    // Iterate over each bit position
    for (int i = bits_count - 1; i >= 0; i--) {
        if ((num >> i) & 1) { // Check if bit at position i is set
            msb = i; // Update msb position
            break; // Stop iteration once MSB is found
        }
    }

    return msb; // Return the MSB position

}


/**
 * @brief Reads the key from the EEPROM memory.
 * 
 * This function reads the key from the EEPROM memory at the specified remote base address.
 * The key is stored in four separate slots in the EEPROM memory, and this function combines
 * the key parts to form the complete key.
 * 
 * @param remote_base_addr The remote base address where the key is stored.
 * @param key_lsb Pointer to a variable where the least significant bits of the key will be stored.
 * @param key_msb Pointer to a variable where the most significant bits of the key will be stored.
 */
void ATEEPROM::readKey(uint8_t remote_base_addr, uint32_t* key_msb, uint32_t* key_lsb)
{
    // Read the key from the EEPROM memory
    uint32_t key_0 = EEPROMReadSlot(KEY_0_ADDR(remote_base_addr));
    uint32_t key_1 = EEPROMReadSlot(KEY_1_ADDR(remote_base_addr));
    uint32_t key_2 = EEPROMReadSlot(KEY_2_ADDR(remote_base_addr));
    uint32_t key_3 = EEPROMReadSlot(KEY_3_ADDR(remote_base_addr));

    #ifdef DEBUG_KEY
    Serial.print("DEBUG readKey | Key 0: ");
    Serial.print(key_0, HEX);
    Serial.print(" | Key 1: ");
    Serial.print(key_1, HEX);
    Serial.print(" | Key 2: ");
    Serial.print(key_2, HEX);
    Serial.print(" | Key 3: ");
    Serial.println(key_3, HEX);
    #endif

    // Combine the key parts to form the complete key
    *key_lsb = (key_1 << 16) | key_0;
    *key_msb = (key_3 << 16) | key_2;
}

/**
 * Writes a key to the EEPROM memory.
 * 
 * @param remote_base_addr The base address of the remote device.
 * @param key_lsb The least significant bits of the key.
 * @param key_msb The most significant bits of the key.
 */
void ATEEPROM::writeKey(uint8_t remote_base_addr, uint32_t key_msb, uint32_t key_lsb)
{
    // Key 0 is the least significant 16 bits of the key_lsb
    uint16_t key_0 = key_lsb & 0xFFFF;
    // Key 1 is the most significant 16 bits of the key_lsb
    uint16_t key_1 = (key_lsb >> 16) & 0xFFFF;
    // Key 2 is the least significant 16 bits of the key_msb
    uint16_t key_2 = key_msb & 0xFFFF;
    // Key 3 is the most significant 16 bits of the key_msb
    uint16_t key_3 = (key_msb >> 16) & 0xFFFF;

    EEPROMWriteSlot(KEY_0_ADDR(remote_base_addr), key_0);
    EEPROMWriteSlot(KEY_1_ADDR(remote_base_addr), key_1);
    EEPROMWriteSlot(KEY_2_ADDR(remote_base_addr), key_2);
    EEPROMWriteSlot(KEY_3_ADDR(remote_base_addr), key_3);
    
}

/**
 * Erases the key stored in the EEPROM memory.
 * 
 * @param remote_base_addr The base address of the remote device.
 */
void ATEEPROM::eraseKey(uint8_t remote_base_addr) {
    EEPROMEraseSlot(KEY_0_ADDR(remote_base_addr));
    EEPROMEraseSlot(KEY_1_ADDR(remote_base_addr));
    EEPROMEraseSlot(KEY_2_ADDR(remote_base_addr));
    EEPROMEraseSlot(KEY_3_ADDR(remote_base_addr));
}

/**
 * Reads the synchronization word from the EEPROM memory.
 * 
 * @param remote_base_addr The base address of the remote device.
 * @param sync Pointer to a variable where the synchronization word will be stored.
 */
void ATEEPROM::readSync(uint8_t remote_base_addr, uint32_t* sync)   {
    *sync = EEPROMReadSlot(SYNC_ADDR(remote_base_addr));
}

/**
 * Writes the synchronization word to the EEPROM memory.
 * 
 * @param remote_base_addr The base address of the remote device.
 * @param sync The synchronization word to be written.
 */
void ATEEPROM::writeSync(uint8_t remote_base_addr, uint32_t sync) {
    EEPROMWriteSlot(SYNC_ADDR(remote_base_addr), sync); 
}

/**
 * Erases the synchronization word stored in the EEPROM memory.
 * 
 * @param remote_base_addr The base address of the remote device.
 */
void ATEEPROM::eraseSync(uint8_t remote_base_addr) {
    EEPROMEraseSlot(SYNC_ADDR(remote_base_addr));
}


/**
 * Reads the serial number from the EEPROM memory.
 * 
 * @param remote_base_addr The base address of the remote device.
 * @param serial_msb Pointer to a variable where the most significant bits of the serial number will be stored.
 * @param serial_lsb Pointer to a variable where the least significant bits of the serial number will be stored.
 */
void ATEEPROM::readSerial(uint8_t remote_base_addr, uint32_t* serial_msb, uint32_t* serial_lsb) {
    *serial_msb = EEPROMReadSlot(SER_1_MSB_ADDR(remote_base_addr));
    *serial_lsb = EEPROMReadSlot(SER_0_LSB_ADDR(remote_base_addr));
}

/**
 * Writes the serial number to the EEPROM memory.
 * 
 * @param remote_base_addr The base address of the remote device.
 * @param serial_msb The most significant bits of the serial number.
 * @param serial_lsb The least significant bits of the serial number.
 */
void ATEEPROM::writeSerial(uint8_t remote_base_addr, uint16_t serial_msb, uint16_t serial_lsb) {
    EEPROMWriteSlot(SER_1_MSB_ADDR(remote_base_addr), serial_msb);
    EEPROMWriteSlot(SER_0_LSB_ADDR(remote_base_addr), serial_lsb);
}

/**
 * Erases the serial number stored in the EEPROM memory.
 * 
 * @param remote_base_addr The base address of the remote device.
 */
void ATEEPROM::eraseSerial(uint8_t remote_base_addr) {
    EEPROMEraseSlot(SER_1_MSB_ADDR(remote_base_addr));
    EEPROMEraseSlot(SER_0_LSB_ADDR(remote_base_addr));
}

/**
 * Checks if the given serial number is authorized.
 * 
 * @param serial The serial number to check.
 * @return Return the index of the authorized serial number, or -1 if the serial number is not authorized.
 */
int ATEEPROM::isSerialAuthorized(uint32_t serial) {
    uint32_t serial_msb, serial_lsb;
    
    uint32_t authorized_serial[3];
    
    int remote_base_addr = -1;

    // Fetch serial numbers from EEPROM
    for (int i = 0; i < 3; i++) {
        readSerial(_REMOTE_BASE_ADDR_LIST[i], &serial_msb, &serial_lsb);
        authorized_serial[i] = (serial_msb << 16) | serial_lsb;
        
        // Check if the serial number is authorized
        if (serial == authorized_serial[i]) {
            remote_base_addr = _REMOTE_BASE_ADDR_LIST[i];
            break;
        }
    }

    return remote_base_addr;
}

/**
 * Reads the seed value from the EEPROM memory.
 * 
 * @param remote_base_addr The base address of the remote device.
 * @return The seed value read from the EEPROM memory.
 */
void ATEEPROM::readSeed(uint8_t remote_base_addr, uint32_t* seed_msb, uint32_t* seed_lsb) {
    *seed_msb = EEPROMReadSlot(SEED_1_MSB_ADDR(remote_base_addr));
    *seed_lsb = EEPROMReadSlot(SEED_0_LSB_ADDR(remote_base_addr));
    
    #ifdef DEBUG_SEED
    Serial.print("DEBUG Seed | Seed 0: ");
    Serial.print(seed_lsb, HEX);
    Serial.print(" | Seed 1: ");
    Serial.println(seed_msb, HEX);
    #endif
}

/**
 * Writes the seed value to the EEPROM memory.
 * 
 * @param remote_base_addr The base address of the remote device.
 * @param seed_msb The most significant bits of the seed value.
 * @param seed_lsb The least significant bits of the seed value.
 */
void ATEEPROM::writeSeed(uint8_t remote_base_addr, uint16_t seed_msb, uint16_t seed_lsb) {
    #ifdef DEBUG_SEED
    Serial.print("DEBUG Seed | Seed 0: ");
    Serial.print(seed_lsb, HEX);
    Serial.print(" | Seed 1: ");
    Serial.println(seed_msb, HEX);
    #endif

    EEPROMWriteSlot(SEED_1_MSB_ADDR(remote_base_addr), seed_msb);
    EEPROMWriteSlot(SEED_0_LSB_ADDR(remote_base_addr), seed_lsb);
}

/**
 * Erases the seed value stored in the EEPROM memory.
 * 
 * @param remote_base_addr The base address of the remote device.
 */
void ATEEPROM::eraseSeed(uint8_t remote_base_addr) {
    EEPROMEraseSlot(SEED_1_MSB_ADDR(remote_base_addr));
    EEPROMEraseSlot(SEED_0_LSB_ADDR(remote_base_addr));
}

/**
 * Erases all data stored in the EEPROM memory for a specific remote device.
 * 
 * @param remote_base_addr The base address of the remote device.
 */
void ATEEPROM::eraseRemote(uint8_t remote_base_addr) {
    eraseKey(remote_base_addr);
    eraseSync(remote_base_addr);
    eraseSerial(remote_base_addr);
    eraseSeed(remote_base_addr);
}