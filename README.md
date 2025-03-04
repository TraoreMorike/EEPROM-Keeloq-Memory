# AT93C46D as a KEELOQ® memory Library

The ATEEPROM library provides an interface to interact with the AT93C46D EEPROM chip using an Arduino. 
Custom functions has been implemented to easily manage KEELOQ® based keyfob such as the HCS301.  

## Features

- Initialize the EEPROM module
- Read and write keys, synchronization words, serial numbers, and seed values
- Erase specific slots or all data in the EEPROM
- Check if a serial number is authorized

## Installation

1. Download the ATEEPROM library.
2. Extract the contents to your Arduino libraries folder (e.g., `Documents/Arduino/libraries/ATEEPROM`).

## Usage

### Include the Library

Include the ATEEPROM library in your Arduino sketch:

```cpp
#include <ATEEPROM.h>
```

### Initialize the EEPROM

Create an instance of the 

ATEEPROM

 class and initialize it with the appropriate pin numbers:

```cpp
ATEEPROM eeprom(csPin, skPin, diPin, doPin);
eeprom.init();
```

### Read and Write Data

#### Read Key

```cpp
uint32_t key_msb, key_lsb;
eeprom.readKey(remote_base_addr, &key_msb, &key_lsb);
```

#### Write Key

```cpp
eeprom.writeKey(remote_base_addr, key_msb, key_lsb);
```

#### Erase Key

```cpp
eeprom.eraseKey(remote_base_addr);
```

#### Read Sync

```cpp
uint32_t sync;
eeprom.readSync(remote_base_addr, &sync);
```

#### Write Sync

```cpp
eeprom.writeSync(remote_base_addr, sync);
```

#### Erase Sync

```cpp
eeprom.eraseSync(remote_base_addr);
```

#### Read Serial

```cpp
uint32_t serial_msb, serial_lsb;
eeprom.readSerial(remote_base_addr, &serial_msb, &serial_lsb);
```

#### Write Serial

```cpp
eeprom.writeSerial(remote_base_addr, serial_msb, serial_lsb);
```

#### Erase Serial

```cpp
eeprom.eraseSerial(remote_base_addr);
```

#### Check if Serial is Authorized

```cpp
int authorized = eeprom.isSerialAuthorized(serial);
```

#### Read Seed

```cpp
uint32_t seed_msb, seed_lsb;
eeprom.readSeed(remote_base_addr, &seed_msb, &seed_lsb);
```

#### Write Seed

```cpp
eeprom.writeSeed(remote_base_addr, seed_msb, seed_lsb);
```

#### Erase Seed

```cpp
eeprom.eraseSeed(remote_base_addr);
```

#### Erase All Data for a Remote

```cpp
eeprom.eraseRemote(remote_base_addr);
```
## Disclaimer

KEELOQ® is a trademark or registered trademark of Microchip Technology Inc.

## License

This library is licensed under the MIT License. See the LICENSE file for more details.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request on GitHub.

---

This README provides an overview of the ATEEPROM library and its usage. For more detailed information, refer to the source code and comments in the ateeprom.cpp and ateeprom.h files.
