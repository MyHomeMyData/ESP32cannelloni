//
// User specific configuration data
//

#include <Arduino.h>

// Select TCP/IP transport protocol:
#define UDP_PROTOCOL 1                      // Use UDP if defined, TCP otherwise

// WiFi:
const char * ssid     = "your_ssid";
const char * password = "your_password";

// Cannelloni host for virtual CAN bus:
#ifdef UDP_PROTOCOL
const IPAddress HOST  = {192,168,178,42};   // ip of cannelloni host
const uint16_t PORT_REMOTE = 2000;          // udp port no. @host
const uint16_t PORT_LISTEN = 2001;          // local udp port to listen for messages
                                            // host> cannelloni -I vcan0 -R 192.168.178.43 -r 2001 -l 2000
                                            // (assuming ESP32 IP address = 192.168.178.43)
#else
const char * HOST     = "192.168.178.42";   // ip or dns of cannelloni server
const uint16_t PORT   = 2001;               // tcp port no. for bidirectional communication
                                            // Start cannelloni server:
                                            // host> cannelloni -I vcan0 -C s -p -t 20000 -l 2001
#endif

// CAN transceiver:
const uint8_t CAN_TX_PIN     = 21;          // GPIO pin connected to tx pin of CAN tranceiver
const uint8_t CAN_RX_PIN     = 22;          // GPIO pin connected to rx pin of CAN tranceiver
const long    CAN_BAUDRATE   = 250E3;       // Baudrate if CAN bus. Viessmann E3 series uses 250000 bits/s

// Filtering of CAN IDs on pyhsical CAN bus:
const uint16_t CAN_ID_RX_MIN = 0x250;       // CAN ids below this value will be ignored
const uint16_t CAN_ID_RX_MAX = 0x6ff;       // CAN ids beyond this value will be ignored

const uint8_t DEBUG = 0;

// End of user configuration