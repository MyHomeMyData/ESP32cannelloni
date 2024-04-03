/*

  Cannelloni light for ESP32

  TCP/IP tunnel for CAN bus data
  Supports UDP and TCP communication only, use of UDP recommended

  Based on project cannelloni: https://github.com/mguentner/cannelloni 

  06.03.2024 MyHomeMyData V0.1.0

  23.03.2024 MyHomeMyData V0.1.1 Reset CAN buffer on connection to server

  28.03.2024 MyHomeMyData V0.1.2 Force attempt to reconnect to cannelloni server in case of TCP communication error
                                 Force restart of ESP32 in case of stalled TCP communication

  31.03.2024 MyHomeMyData V0.2.0 Added support for UDP protocol
                                 Select protocol via user_config.h

MIT License

Copyright (c) 2024 MyHomeMyData

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include "user_config.h"    // Import user specific configuration data

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#ifdef UDP_PROTOCOL
#include "AsyncUDP.h"       // https://github.com/espressif/arduino-esp32/tree/master/libraries/AsyncUDP
#endif
#include <ESPmDNS.h>
#include <CAN.h>            // CAN library by Sandeep Mistry  
                            // https://github.com/sandeepmistry/arduino-CAN/blob/master/README.md

#ifdef UDP_PROTOCOL
const char* PGM_INFO = "Cannelloni light for ESP32 V0.2.0 UDP";
#else
const char* PGM_INFO = "Cannelloni light for ESP32 V0.2.0 TCP";
#endif

#ifdef UDP_PROTOCOL
const uint16_t CANNELLONI_VERSION = 2;
const uint16_t LEN_UDP_HEADER     = 5;
const uint16_t LEN_CAN_HEADER     = 5;
const uint16_t LEN_CAN_DATA_MAX   = 8;
const uint16_t LEN_CAN_MSG_MAX    = LEN_CAN_HEADER+LEN_CAN_DATA_MAX;
const uint16_t CNT_CAN_MSG_MAX    = 32;     // Max. number of CAN messages within one udp datagram
const uint16_t LEN_UDP_BUF        = LEN_UDP_HEADER + LEN_CAN_MSG_MAX*CNT_CAN_MSG_MAX;
const uint16_t CNT_UDP_BUF        = 8;      // Number of udp buffers
const uint16_t LEN_UDP_DATA_BUF   = LEN_UDP_BUF*CNT_UDP_BUF;
uint8_t udpdata[LEN_UDP_DATA_BUF];          // Udp Tx buffers
uint16_t ibuf_write = 0;
uint16_t ibuf_read  = 0;
uint16_t udp_lens[CNT_UDP_BUF];             // Lengths of valid data in udp buffers

uint8_t* base_ptr       = udpdata;
uint8_t* base_ptr_write = base_ptr;
uint8_t* base_ptr_read  = base_ptr;
uint8_t* can_ptr_write  = base_ptr_write+LEN_UDP_HEADER;

bool switch_to_next_buf = false;  // Switch to next udp buffer before storing next CAN message
const int MILLIS_WAIT_MAX = 20;   // Max. waiting time until sending buffered CAN messages
bool can_storage_busy = false;    // Data storage within canOnReceive() is active
uint16_t cnt_buf_to_send = 0;     // Number of udp buffers ready to be sent via udp
uint64_t cnt_udp_sent = 0;        // Total number of udü buffers sent

unsigned long millis_sent_last = 0;

uint16_t udp_tx_seqNo = 0;
AsyncUDP udpListener;             // Udp listener for receiving packages from cannelloni host
AsyncUDP udpRemote;               // Upd remote connection to cannelloni host for sending upd packages
#else
const String CANNELLONI_TOKEN = "CANNELLONIv1";
const uint8_t LEN_CAN_DATA_MAX   = 8;
const uint8_t LEN_CAN_MSG_MAX = 13;     // Max. length of a CAN frame as TCP message: 4 (CAN ID) + 1 (dlc) + 8 (data bytes)
const int CNT_CAN_MSG_MAX     = 512;    // Max. number of buffered CAN messages
const int LEN_CAN_BUF         = LEN_CAN_MSG_MAX*CNT_CAN_MSG_MAX;
const int LEN_TCP_BUF         = 1500;

uint8_t can_buf[LEN_CAN_BUF];           // Buffer for CAN frames received on physical CAN bus
uint8_t tcp_buf[LEN_TCP_BUF];           // Buffer for CAN frames received from cannelloni server

int ibuf_write      = 0;                // Index of next CAN frame for writing
int ibuf_read       = 0;                // Index of next CAN frame for reading
uint8_t* base_ptr   = can_buf;          // Pointer to start of CAN buffer
uint8_t* ptr_write  = base_ptr;         // Pointer to next buffer position for writing
uint8_t* ptr_read   = base_ptr;         // Pointer to next buffer position for reading
#endif

uint64_t cnt_can_rx_total   = 0;    // Total number of CAN frames received on physical CAN bus
uint64_t cnt_can_tx_total   = 0;    // Total number of CAN frames transmitted on physical CAN bus
uint64_t cnt_tcp_rx_total   = 0;    // Total number of tcp frames received from cannelloni server (one frame may contain more than one CAN frame)
uint64_t cnt_tcp_tx_total   = 0;    // Total number of tcp frames transmitted to cannelloni server (one CAN frame only)
uint64_t cnt_tcp_tx_pending = 0;    // Number of can frames pending (received but not yet sent to cannelloni server)
uint16_t cnt_tcp_tx_retries = 0;    // Number of failed tcp transmitions to cannelloni server
bool canBusy = false;               // canOnReceive() is active

WebServer server(80);
WiFiClient client;
bool hostAvailable = false;           // true, if TCP connection to server is established
bool serverConfirmed = false;           // true, if server sent correct identification string
uint16_t cnt_wifi_connect_incomplete = 0;       // Counts incomplete connecting attempts to wifi

unsigned long millis_last = 0;
unsigned long millis_now  = 0;
unsigned long millis_wifi_disconnect  = 0;

unsigned long micros_start = 0;
unsigned long micros_stopp = 0;
unsigned long micros_max   = 0;
unsigned long micros_total = 0;

TaskHandle_t Task_loopCore0;   // https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/

// Buffers for logging:
const int CNT_LOG_MAX = 16;   // Max. number of log lines
int cnt_log = 0;              // Number of valid log lines
char cbuf[120];
String logStrings[CNT_LOG_MAX];
int iLogRead  = 0;
int iLogWrite = 0;

#define NTP_SERVER "de.pool.ntp.org"
#define TZ_INFO "WEST-1DWEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00" // Western European Time

// Zeitvariablen
struct tm local;
time_t now;
double time_now; //Zeitpunkt jetzt
int lastday=0;
bool ntp_done = false;

const uint8_t DEBUG = 1;

// Logging:
String timeStr() {
  char tsBuf[10];
  sprintf(tsBuf,"%02d:%02d:%02d",local.tm_hour,local.tm_min,local.tm_sec);
  return String(tsBuf);
}

void htmlLog(const char * cbuf) {
  logStrings[iLogWrite] = timeStr()+" -> "+String(cbuf);
  iLogWrite = (++iLogWrite % CNT_LOG_MAX);
  if (cnt_log < CNT_LOG_MAX) cnt_log++;
}

// Core 0 Tasks:
void printStats() {
  if ( (DEBUG > 0) && (hostAvailable) ) {
    Serial.print(timeStr());
    Serial.print(": CAN rx=");
    Serial.print(cnt_can_rx_total);
    Serial.print("; tcp tx pending=");
    Serial.print(cnt_tcp_tx_pending);
    Serial.print("; tcp tx retries=");
    Serial.print(cnt_tcp_tx_retries);
    Serial.print("; tcp rx=");
    Serial.print(cnt_tcp_rx_total);
    Serial.print("; CAN tx=");
    Serial.print(cnt_can_tx_total);
    if (cnt_tcp_tx_total>0) {
      Serial.print("; tcp_write_max=");
      Serial.print(micros_max);
      Serial.print("; tcp_write_mean=");
      #ifdef UDP_PROTOCOL
      Serial.print(micros_total/cnt_udp_sent);
      #else
      Serial.print(micros_total/cnt_tcp_tx_total);
      #endif
    }
    Serial.println();
  }
}

void localTimeUpdate() {
  // Update local time
  time(&now);
  localtime_r(&now, &local);
  time_now=mktime(&local);
  if (local.tm_mday != lastday) {
    Serial.println("Updating local time ...");
    configTzTime(TZ_INFO, NTP_SERVER); 
    getLocalTime(&local, 5000);       // Synchronize with NTP time once per day
    htmlLog("Local time successfully updated via NTP service.");
    lastday=local.tm_mday;
  }
}

void loopWebServer() {
  server.handleClient(); // Liegt eine Webserveranfrage vor?
}

void handleRoot() {

  String sc="";
  String message = "";

  #ifndef UDP_PROTOCOL
  if (!hostAvailable) sc = "NOT connected";
  if ( (hostAvailable) && (!serverConfirmed) ) sc = "Connected but NOT confirmed";
  if ( (hostAvailable) && (serverConfirmed) )  sc = "<font color=green><strong>Connected and confirmed</strong></font>";
  #endif

  message += "<!DOCTYPE html><html><head><style>table {border-collapse: collapse;} table, td, th {border: 2px solid black; cellpadding: 4px;}</style><meta http-equiv=refresh content=1></head>";
  message += "<body style=\"background-color:white;\"><font face=\"Calibri\" size=\"2\">";
  message += "<h1>"+String(PGM_INFO)+"</h1>";
  
  #ifndef UDP_PROTOCOL
  message += "<h2>Cannelloni host: "+sc+"</h2><br>";
  #endif

  message += "<table><tr>";
  message += "<th align=center>"+timeStr()+"</th><th align=center>TX</th><th align=center>RX</th></tr>";
  message += "<tr><td align=left>TCP Transmission Summary</td><td align=right>"+String(cnt_tcp_tx_total)+"</td><td align=right>"+String(cnt_tcp_rx_total)+"</td></tr>";
  message += "<tr><td align=left>CAN Transmission Summary</td><td align=right>"+String(cnt_can_tx_total)+"</td><td align=right>"+String(cnt_can_rx_total)+"</td>";
  message += "</tr></table>";

  message += "<h2>Log:</h2>";

  iLogRead = iLogWrite-cnt_log;
  if (iLogRead<0) iLogRead += CNT_LOG_MAX;
  for (uint16_t i=0; i<cnt_log;i++) {
    message += logStrings[(iLogRead+i) % CNT_LOG_MAX] + "<br>";
  }

  message += "</font></body></html>"; 
  
  server.send(200,"text/html", message);
}

void handleNotFound(){
  String message="Path ";
  message += server.uri();
  message += " not found.\n";
  server.send(404, "text/plain", message);
}

#ifndef UDP_PROTOCOL
bool connectToServer(const char* host, uint16_t port) {

  serverConfirmed = false;
  sprintf(cbuf, "Connecting to cannelloni server on %s:%d", host, port);
  htmlLog(cbuf);
  Serial.print("Connecting to cannelloni server on ");
  Serial.print(host);
  Serial.print(":");
  Serial.print(port);
  Serial.print(" ");

  unsigned long timeout = millis()+3600000;  // 1 hour timeout
  while ( (!client.connect(host, port)) && (millis()<timeout) ) {
    Serial.print(".");
    loopWebServer();
    delay(1000);
  }
  if (client.connected()) {
    htmlLog("Connected.");
    Serial.println();
    Serial.println("Connected.");
  } else {
    htmlLog("Could not connect to server.");
    Serial.println();
    Serial.println("Could not connect to server.");
    return false;
  }

  loopWebServer();
  bool cannelloni = false;
  timeout = millis()+5000;  // 5 seconds timeout
  while ( (!cannelloni) && (millis()<timeout) ) {
    delay(100);
    if (client.available() > 0) {
      String line = client.readString();
      if (line == CANNELLONI_TOKEN) {
        htmlLog("Cannelloni server confirmed.");
        Serial.println("Cannelloni server confirmed.");
        client.print(CANNELLONI_TOKEN);
        serverConfirmed = true;
      }
      cannelloni = true;
    }
  }
  if (!cannelloni) {
    htmlLog("Cannelloni server NOT confirmed.");
    Serial.println("Cannelloni server NOT confirmed.");
  }
  // Server successfully connected. Initialize buffer and counters for CAN messages:
  ptr_write = ptr_read = base_ptr;
  cnt_can_rx_total = cnt_tcp_tx_total = 0;
  ibuf_read = ibuf_write = 0;
  micros_max = micros_total = 0;
  return true;
}
#endif

void checkWiFi() {
  if ((!WiFi.isConnected()) && (millis_wifi_disconnect==0)) {
    millis_wifi_disconnect = millis_now;
  } 
  if ((!WiFi.isConnected()) && (millis_wifi_disconnect!=0) && ((millis_now-millis_wifi_disconnect)>30000)) {
    millis_wifi_disconnect = millis_now;
    Serial.println("Reconnect to WiFi ...");
    WiFi.reconnect();
  } 
}

#ifdef UDP_PROTOCOL
void onUdpListenerPacket(AsyncUDPPacket packet) {
  uint16_t packet_len = packet.length();
  if (packet_len >= 10) {
    uint8_t* p = packet.data();
    uint8_t version = p[0];
    uint8_t op_code = p[1];
    uint8_t seq_no  = p[2];
    uint16_t frame_cnt = p[3]*256+p[4];
    cnt_tcp_rx_total += frame_cnt;
    if ( (version == CANNELLONI_VERSION) && (op_code == 0) ) {
      int frame = 0;
      int ofs = 5;
      while ( (frame<frame_cnt) && (ofs<=packet_len-LEN_UDP_HEADER) ) {
        ofs += 2;   // Skip 2 bytes for extended CAN ID
        uint16_t can_id = p[ofs++]*256+p[ofs++];
        uint8_t dlc = p[ofs++];
        if (dlc & 0xf0) {
          sprintf(cbuf, "WARNING: Extended CAN frames are not supported. Skipping.");
          htmlLog(cbuf);
          Serial.println(cbuf);
          ofs += 1 + (dlc & 0x0f);
          break;
        }
        if (ofs<=packet_len-dlc) {
          CAN.beginPacket(can_id);
          CAN.write(&p[ofs], dlc);
          CAN.endPacket();
          ofs += dlc;
          delay(5);
        }
        cnt_can_tx_total++;
        frame++;
      }
    } else {
          sprintf(cbuf, "WARNING: Received udp packet has wrong version (%d) or op-code (%d).",version,op_code);
          htmlLog(cbuf);
          Serial.println(cbuf);
    }
  }
}

void setupUdpListener(uint16_t port) {
  if(udpListener.listen(port)) {
      udpListener.onPacket(onUdpListenerPacket);
      sprintf(cbuf, "Udp listener started.");
      htmlLog(cbuf);
      Serial.println(cbuf);
      sprintf(cbuf, "Start cannelloni on host, e.g. for vcan0: cannelloni -I vcan0 -R %s -r %d -l %d",WiFi.localIP().toString().c_str(),PORT_REMOTE,PORT_LISTEN);
      htmlLog(cbuf);
      Serial.println(cbuf);
  }
}

void initBuf(uint8_t seqNo) {
  base_ptr_write = base_ptr+LEN_UDP_BUF*ibuf_write;
  can_ptr_write  = base_ptr_write+LEN_UDP_HEADER;
  base_ptr_write[0] = CANNELLONI_VERSION;
  base_ptr_write[1] = 0x00;  // OpCode = DATA
  base_ptr_write[2] = seqNo; // Sequence no.
  base_ptr_write[3] = 0x00;  // Count MSB
  base_ptr_write[4] = 0x00;  // Count LSB
  udp_lens[ibuf_write] = LEN_UDP_HEADER;
  udp_lens[(ibuf_write+1) % CNT_UDP_BUF] = 0;  // No data yet in next buffer
}

void switchToNextBuf(uint8_t seqNo) {
  ibuf_write = (++ibuf_write % CNT_UDP_BUF);
  initBuf(seqNo);
}

void checkClient() {
  if (WiFi.isConnected()) {
    if (!udpRemote.connected()) {
      if (hostAvailable) {
        Serial.println("Connection to host lost.");
        htmlLog("Connection to host lost.");
      }
      hostAvailable = false;
      udpRemote.close();
      unsigned long timeout = millis()+5000;
      while ( (!ntp_done) && (millis()<timeout) ) delay(100);
      hostAvailable = udpRemote.connect(HOST,PORT_REMOTE);
      if (hostAvailable) {
        sprintf(cbuf, "Udp remote connection established.");
        htmlLog(cbuf);
        Serial.println(cbuf);
      }
    }
  }
}

#else
void checkClient() {
  if (WiFi.isConnected()) {
    if (!client.connected()) {
      if (hostAvailable) {
        Serial.println("Connection to server lost.");
        htmlLog("Connection to server lost.");
      }
      hostAvailable = false;
      serverConfirmed = false;
      client.stop();
      unsigned long timeout = millis()+5000;
      while ( (!ntp_done) && (millis()<timeout) ) delay(100);
      hostAvailable = connectToServer(HOST, PORT);
    }
  }
}

void getTcpData() {
  if (hostAvailable) {
    size_t len=client.available();
    if (len > 0) {
      if (!serverConfirmed) {
        String line = client.readString();
        if (line == CANNELLONI_TOKEN) {
          client.print(CANNELLONI_TOKEN);
          htmlLog("Cannelloni server confirmed.");
          Serial.println("Cannelloni server confirmed.");
          serverConfirmed = true;
        }
        len=client.available();
      }
      int cnt_read = client.read(tcp_buf, len);
      if (cnt_read != len) {
        sprintf(cbuf, "ERROR: TCP read() did not receive all data: expected=%d; read=%d",len,cnt_read);
        htmlLog(cbuf);
        Serial.println(cbuf);
      }
      int ofs = 0;
      while (ofs<len) {
        ofs += 2;   // Skip 2 bytes for extended CAN ID
        uint16_t can_id = tcp_buf[ofs++]*256+tcp_buf[ofs++];
        uint8_t dlc = tcp_buf[ofs++];
        if (dlc & 0xf0) {
          Serial.println("Extended CAN frames are not supported. Skipping.");
          htmlLog("Extended CAN frames are not supported. Skipping.");
          ofs += 1 + (dlc & 0x0f);
          break;
        }
        if (ofs<=len-dlc) {
          cnt_tcp_rx_total++;
          if (CAN.beginPacket(can_id, dlc)) {
            size_t written = CAN.write(&tcp_buf[ofs], dlc);
            CAN.endPacket();
            if (written = dlc) {
              cnt_can_tx_total++;
            } else {
              sprintf(cbuf, "ERROR: CAN write() did not send all data: size=%d; sent=%d",dlc,written);
              htmlLog(cbuf);
              Serial.println(cbuf);
            }
            delay(2);
          } else {
            CAN.endPacket();
            Serial.println("CAN bus not available: Could not send data.");
            htmlLog("CAN bus not available: Could not send data.");
            ofs += dlc;
          }
          ofs += dlc;
        }
      }
    }
  }
}
#endif

#ifdef UDP_PROTOCOL
void printCanMsg(int id, uint8_t dlc_, uint8_t* data) {
  Serial.print(timeStr());
  sprintf(cbuf, ": %03X [%d] ", id, dlc_);
  Serial.print(cbuf);
  for (uint8_t i=0;i<dlc_;i++) {
    sprintf(cbuf,"%02X ",data[i]);
    Serial.print(cbuf);
  }
  Serial.println();
}

void printUdpBuffer(uint8_t* udp) {
  uint8_t* ptrCan = udp+LEN_UDP_HEADER;
  for (int iMsg=0;iMsg<udp[4];iMsg++) {
    uint8_t dlc_ = ptrCan[4];
    printCanMsg(ptrCan[3]+(ptrCan[2] << 8), dlc_, ptrCan+5);
    ptrCan += LEN_CAN_HEADER+dlc_;
  }
}
#endif

void sendCanDataToHost() {
  #ifdef UDP_PROTOCOL
  if ( ((millis_now-millis_sent_last) >= MILLIS_WAIT_MAX) && (udp_lens[ibuf_read] > LEN_UDP_HEADER) && (!can_storage_busy) ) {
    switch_to_next_buf = true;
    cnt_buf_to_send++;
  }

  if (cnt_buf_to_send > 0) {
    // Udp buffer ready to be sent
    base_ptr_read = base_ptr+LEN_UDP_BUF*ibuf_read;
    micros_start = micros();
    size_t size = udp_lens[ibuf_read];
    size_t sent = udpRemote.write(base_ptr_read, size);
    micros_stopp = micros();
    uint64_t micros_diff = micros_stopp-micros_start;
    micros_total += micros_diff;
    if (DEBUG > 1) printUdpBuffer(base_ptr_read);
    if ((micros_stopp-micros_start) > micros_max) {
      micros_max = micros_stopp-micros_start;
    }
    if ((micros_stopp-micros_start) > micros_max) {
      micros_max = micros_stopp-micros_start;
    }
    cnt_tcp_tx_total += base_ptr_read[4];
    ibuf_read = (++ibuf_read % CNT_UDP_BUF);
    cnt_buf_to_send--;
    cnt_udp_sent++;
    millis_sent_last = millis_now;
    if (sent != size) {
      sprintf(cbuf, "ERROR: UDP write() did not send all data: size=%d; sent=%d; duration=%d us",size,sent,micros_diff);
      htmlLog(cbuf);
      Serial.println(cbuf);
    }
  }
  #else
  if (ptr_write != ptr_read) {
    size_t size = ptr_read[4]+5;
    micros_start = micros();
    size_t sent = client.write(ptr_read, size);
    micros_stopp = micros();
    unsigned long micros_diff = micros_stopp-micros_start;
    micros_total += micros_diff;
    if ((micros_stopp-micros_start) > micros_max) {
      micros_max = micros_stopp-micros_start;
    }
    if (sent == size) {
      cnt_tcp_tx_total++;
      ibuf_read = (++ibuf_read % CNT_CAN_MSG_MAX);
      ptr_read = base_ptr+LEN_CAN_MSG_MAX*ibuf_read;
      cnt_tcp_tx_retries = 0;
    } else {
      cnt_tcp_tx_retries++;
      sprintf(cbuf, "ERROR: TCP write() did not send all data: size=%d; sent=%d; duration=%d us",size,sent,micros_diff);
      htmlLog(cbuf);
      Serial.println(cbuf);
      if (cnt_tcp_tx_retries <= 3) {
        // Try to reconnet to server
        sprintf(cbuf, "Force attempt to reconnect to cannelloni server ...");
        htmlLog(cbuf);
        Serial.println(cbuf);
        client.stop();    // Force attempt to reconnect to server
      } else {
        // TCP communication stalled => Force restart of ESP32
        sprintf(cbuf, "ERROR: TCP write() stalled. ESP32 will be restarted after 5 seconds.");
        htmlLog(cbuf);
        Serial.println(cbuf);
        client.stop();    // Close connection to server
        delay(5000);
        ESP.restart();
      }
    }
    cnt_tcp_tx_pending = cnt_can_rx_total - cnt_tcp_tx_total;
  }
  #endif
}

void canOnReceive(int packetSize) {
  if (canBusy) {
    sprintf(cbuf, "WARNING: canOnReceive() is busy. Skipping CAN frame.");
    htmlLog(cbuf);
    Serial.println(cbuf);
    return;
  }
  canBusy = true;

  if (!hostAvailable) {
    //Serial.println("Received CAN frame skipped due to missing Wifi connection.");
    return;
  }
  // received a CAN frame
  if (CAN.packetExtended()) {
    //Serial.print("Received extended CAN frame. Skipping.");
    return;
  }
  if (CAN.packetRtr()) {
    // Remote transmission request, packet contains no data
    //Serial.print("Received CAN remote transmission request. Skipping.");
    return;
  }
  if (packetSize > LEN_CAN_DATA_MAX) {
    //Serial.print("Bad length of CAN frame. Skipping.");
    return;
  }

  uint16_t can_id = CAN.packetId();
  if ( (can_id<CAN_ID_RX_MIN) || (can_id>CAN_ID_RX_MAX) ) return;   // Filter for CAN IDs

  #ifdef UDP_PROTOCOL
  can_storage_busy = true;

  cnt_can_rx_total++;
  can_id = CAN.packetId();

  if (switch_to_next_buf) {
    switch_to_next_buf = false;
    switchToNextBuf(++udp_tx_seqNo);
  }
  base_ptr_write[4]++;    // Increase Udp message count LSB

  *can_ptr_write++ = 0x00;                   // CAN ID bits 24-31
  *can_ptr_write++ = 0x00;                   // CAN ID bits 16-23
  *can_ptr_write++ = (can_id >> 8) & 0xff ;  // CAN ID bits 08-15
  *can_ptr_write++ = can_id & 0xff;          // CAN ID bits 00-07
  *can_ptr_write++ = packetSize & 0xf;       // CAN DLC
  CAN.readBytes(can_ptr_write, packetSize);  // Read CAN data
  can_ptr_write += packetSize;
  udp_lens[ibuf_write] += LEN_CAN_HEADER+packetSize;
  if (base_ptr_write[4] >= CNT_CAN_MSG_MAX) {
    // Max. number of messages in buffer reached
    switchToNextBuf(++udp_tx_seqNo);
    cnt_buf_to_send++;
  }
  millis_sent_last = millis();
  can_storage_busy = false;
  canBusy = false;
  #else
  uint8_t* ptr = ptr_write;         // Use local pointer
  *ptr++ = 0x00;                    // CAN ID bits 24-31
  *ptr++ = 0x00;                    // CAN ID bits 16-23
  *ptr++ = (can_id >> 8) & 0xff ;   // CAN ID bits 08-15
  *ptr++ = can_id & 0xff;           // CAN ID bits 00-07
  *ptr++ = packetSize & 0xf;        // CAN DLC
  CAN.readBytes(ptr, packetSize);   // Read CAN data
  cnt_can_rx_total++;

  // Switch to next message buffer:
  ibuf_write = (++ibuf_write % CNT_CAN_MSG_MAX);
  ptr_write = base_ptr+LEN_CAN_MSG_MAX*ibuf_write;
  #endif
}

void WiFiEvent(WiFiEvent_t event)
{
    Serial.print("[WiFi-event]: ");
    switch (event) {
        case ARDUINO_EVENT_WIFI_READY: 
            Serial.println("WiFi interface ready");
            break;
        case ARDUINO_EVENT_WIFI_SCAN_DONE:
            Serial.println("Completed scan for access points");
            break;
        case ARDUINO_EVENT_WIFI_STA_START:
            Serial.println("WiFi client started");
            break;
        case ARDUINO_EVENT_WIFI_STA_STOP:
            Serial.println("WiFi clients stopped");
            hostAvailable = false;
            client.stop();
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.println("Connected to access point");
            if (++cnt_wifi_connect_incomplete > 4) {
              Serial.println("WiFi connection failed. Restarting ESP ...");
              delay(100);
              ESP.restart();
            }
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("Disconnected from WiFi access point");
            hostAvailable = false;
            #ifdef UDP_PROTOCOL
            udpRemote.close();
            udpListener.close();
            #else
            serverConfirmed = false;
            client.stop();
            #endif
            if (++cnt_wifi_connect_incomplete > 4) {
              Serial.println("WiFi connection failed. Restarting ESP ...");
              delay(100);
              ESP.restart();
            }
            break;
        case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
            Serial.println("Authentication mode of access point has changed");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.print("Obtained IP address: ");
            Serial.print(WiFi.localIP());
            Serial.print(", hostname: ");
            Serial.println(WiFi.getHostname());
            // NTP Zeit aktualisieren
            if (MDNS.begin("esp32")) {
              Serial.println("MDNS-Responder running.");
            }
            configTzTime(TZ_INFO, NTP_SERVER); 
            getLocalTime(&local, 5000);
            lastday=local.tm_mday;
            localTimeUpdate();
            ntp_done = true;
            cnt_wifi_connect_incomplete = 0;
            #ifdef UDP_PROTOCOL
            setupUdpListener(PORT_LISTEN);
            #endif
            server.begin();
            Serial.println("HTTP-server started.");
            break;
        case ARDUINO_EVENT_WIFI_STA_LOST_IP:
            Serial.println("Lost IP address and IP address is reset to 0");
            hostAvailable = false;
            client.stop();
            server.stop();
            break;
        case ARDUINO_EVENT_WPS_ER_SUCCESS:
            Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
            break;
        case ARDUINO_EVENT_WPS_ER_FAILED:
            Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
            break;
        case ARDUINO_EVENT_WPS_ER_TIMEOUT:
            Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
            break;
        case ARDUINO_EVENT_WPS_ER_PIN:
            Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
            break;
        case ARDUINO_EVENT_WIFI_AP_START:
            Serial.println("WiFi access point started");
            break;
        case ARDUINO_EVENT_WIFI_AP_STOP:
            Serial.println("WiFi access point  stopped");
            break;
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            Serial.println("Client connected");
            break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            Serial.println("Client disconnected");
            break;
        case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
            Serial.println("Assigned IP address to client");
            break;
        case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
            Serial.println("Received probe request");
            break;
        case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
            Serial.println("AP IPv6 is preferred");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
            Serial.println("STA IPv6 is preferred");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP6:
            Serial.println("Ethernet IPv6 is preferred");
            break;
        case ARDUINO_EVENT_ETH_START:
            Serial.println("Ethernet started");
            break;
        case ARDUINO_EVENT_ETH_STOP:
            Serial.println("Ethernet stopped");
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.println("Ethernet connected");
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Serial.println("Ethernet disconnected");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.println("Obtained IP address");
            break;
        default:
            Serial.println("UNKOWN event");
            break;
    }
  }

void loopCore0(void * pvParameters) {
  unsigned long millis_lastprint = 0;
  for (;;) {
    delay(50);
    localTimeUpdate();
    if ( (hostAvailable) && ((millis()-millis_lastprint)>10000) ) {
      millis_lastprint = millis();
      printStats();
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(100);

  #ifdef UDP_PROTOCOL
  initBuf(++udp_tx_seqNo);            // Initialize first Udp buffer
  #else
  memset(base_ptr,0, LEN_CAN_BUF);    // Initialize can buffer
  memset(tcp_buf, 0, LEN_TCP_BUF);    // Initialize tcp buffer
  #endif

  Serial.println(PGM_INFO);
  htmlLog(PGM_INFO);
  #ifndef UDP_PROTOCOL
  sprintf(cbuf, "Start cannelloni server, e.g. for vcan0: cannelloni -I vcan0 -C s -p -t 20000 -l %d",PORT);
  #endif
  Serial.println(cbuf);
  Serial.println();

  sprintf(cbuf, "CPU Freq = %d MHz",getCpuFrequencyMhz());   // https://deepbluembedded.com/esp32-change-cpu-speed-clock-frequency/
  htmlLog(cbuf);
  Serial.println(cbuf);

  sprintf(cbuf, "Start connecting to WiFi %s", ssid);
  htmlLog(cbuf);
  Serial.println(cbuf);
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(WiFiEvent);
  WiFi.setHostname("ESP32cannelloni");
  WiFi.begin(ssid, password);
  cnt_wifi_connect_incomplete = 0;

  // Configure CAN tranceiver:
  CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
  CAN.begin(CAN_BAUDRATE);

  // Register callback for data from pyhsical CAN bus:
  CAN.onReceive(canOnReceive);
  Serial.println("CAN bus listener started.");

  // Webserver initialisieren
  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);

  // Assign printing of regular status messages to core 0 (all other tasks will run on core 1):
  xTaskCreatePinnedToCore(
              loopCore0,        // Task function.
              "loopCore0",      // name of task.
              10000,            // Stack size of task
              NULL,             // parameter of the task
              0,                // priority of the task
              &Task_loopCore0,  // Task handle to keep track of created task
              0);               // pin task to core 0
  delay(50);
  
}

void loop() {
  millis_now = millis();
  checkWiFi();              // Check WiFi connection and do a reconnect if needed
  checkClient();            // Check connection to cannelloni server and try to reconnect if needed
  if (hostAvailable) {
    #ifndef UDP_PROTOCOL
    getTcpData();           // Fetch data from cannelloni server and send it to local CAN bus
    #endif
    sendCanDataToHost();    // Check for data available from local CAN bus and send it to cannelloni server
  }
  loopWebServer();
}
