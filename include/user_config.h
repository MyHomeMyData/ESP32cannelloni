//
// User specific configuration data
//

#include <Arduino.h>

// WiFi:
const char * ssid     = "your_ssid";
const char * password = "your_password";

// Cannelloni server for virtual CAN bus:
const char * HOST     = "192.168.178.42";   // ip or dns of cannelloni server
const uint16_t PORT   = 2001;               // tcp port no. for bidirectional communication
                                            // Start cannelloni server:
                                            // > cannelloni -I vcan0 -C s -p -t 20000 -l 2001

// CAN transceiver:
const uint8_t CAN_TX_PIN     = 21;          // GPIO pin connected to tx pin of CAN tranceiver
const uint8_t CAN_RX_PIN     = 22;          // GPIO pin connected to rx pin of CAN tranceiver
const long    CAN_BAUDRATE   = 250E3;       // Baudrate if CAN bus. Viessmann E3 series uses 250000 bits/s

// Filtering of CAN IDs on pyhsical CAN bus:
const uint16_t CAN_ID_RX_MIN = 0x250;       // CAN ids below this value will be ignored
const uint16_t CAN_ID_RX_MAX = 0x6ff;       // CAN ids beyond this value will be ignored

// End of user configuration