/*

  Cannelloni light for ESP32

  TCP tunnel for CAN bus data
  Supports TCP communication only, no UDP

  Based on project cannelloni: https://github.com/mguentner/cannelloni 

  06.03.2024 MyHomeMyData V0.1.0

  23.03.2024 MyHomeMyData V0.1.1 Reset CAN buffer on connection to server

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

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <CAN.h>            // CAN library by Sandeep Mistry  
                            // https://github.com/sandeepmistry/arduino-CAN/blob/master/README.md

#include "user_config.h"    // Import user specific configuration data

const char* PGM_INFO = "Cannelloni light for ESP32 V0.1.1";

const String CANNELLONI_TOKEN = "CANNELLONIv1";
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

uint32_t cnt_can_rx_total   = 0;    // Total number of CAN frames received on physical CAN bus
uint32_t cnt_can_tx_total   = 0;    // Total number of CAN frames transmitted on physical CAN bus
uint32_t cnt_tcp_rx_total   = 0;    // Total number of tcp frames received from cannelloni server (one frame may contain more than one CAN frame)
uint32_t cnt_tcp_tx_total   = 0;    // Total number of tcp frames transmitted to cannelloni server (one CAN frame only)
uint32_t cnt_tcp_tx_retries = 0;    // Number of failed tcp transmitions to cannelloni server
uint32_t cnt_tcp_tx_pending = 0;    // Number of can frames pending (received but not yet sent to cannelloni server)

WebServer server(80);
WiFiClient client;
bool serverAvailable = false;           // true, if TCP connection to server is established
bool serverConfirmed = false;           // true, if server sent correct identification string
uint8_t cnt_wifi_connect_incomplete = 0;       // Counts incomplete connecting attempts to wifi

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

const uint8_t DEBUG = 0;

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
  if (DEBUG > 0) {
    Serial.print("Statistics: CAN rx=");
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
      Serial.print(micros_total/cnt_tcp_tx_total);
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

  int mean=0;
  String sc="";

  if (!serverAvailable) sc = "NOT connected";
  if ( (serverAvailable) && (!serverConfirmed) ) sc = "Connected but NOT confirmed";
  if ( (serverAvailable) && (serverConfirmed) )  sc = "<font color=green><strong>Connected and confirmed</strong></font>";

  String message="<!DOCTYPE html><html><head><style>table {border-collapse: collapse;} table, td, th {border: 2px solid black; cellpadding: 4px;}</style><meta http-equiv=refresh content=1></head>";

  message += "<body style=\"background-color:white;\"><font face=\"Calibri\" size=\"2\">";
  message += "<h1>"+String(PGM_INFO)+"</h1>";
  
  message += "<h2>Cannelloni server: "+sc+"</h2><br>";

  message += "<table><tr>";
  message += "<th align=center>"+timeStr()+"</th><th align=center>TX</th><th align=center>RX</th></tr>";
  message += "<tr><td align=left>TCP Transmission Summary</td><td align=right>"+String(cnt_tcp_tx_total)+"</td><td align=right>"+String(cnt_tcp_rx_total)+"</td></tr>";
  message += "<tr><td align=left>CAN Transmission Summary</td><td align=right>"+String(cnt_can_tx_total)+"</td><td align=right>"+String(cnt_can_rx_total)+"</td>";
  message += "</tr></table>";

  message += "<h2>Log:</h2>";

  iLogRead = iLogWrite-cnt_log;
  if (iLogRead<0) iLogRead += CNT_LOG_MAX;
  for (uint8_t i=0; i<cnt_log;i++) {
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

bool connectToServer(const char* host, uint16_t port) {

  serverConfirmed = false;
  sprintf(cbuf, "Connecting to cannelloni server on %s:%d", host, port);
  htmlLog(cbuf);
  Serial.print("Connecting to cannelloni server on ");
  Serial.print(host);
  Serial.print(":");
  Serial.print(port);
  Serial.print(" ");

  unsigned long timeout = millis()+30000;  // 15 seconds timeout
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
  micros_max = micros_total = 0;
  return true;
}

void checkWiFi() {
  millis_now = millis();
  if ((!WiFi.isConnected()) && (millis_wifi_disconnect==0)) {
    millis_wifi_disconnect = millis_now;
  } 
  if ((!WiFi.isConnected()) && (millis_wifi_disconnect!=0) && ((millis_now-millis_wifi_disconnect)>30000)) {
    millis_wifi_disconnect = millis_now;
    Serial.println("Reconnect to WiFi ...");
    WiFi.reconnect();
  } 
}

void checkClient() {
  if (WiFi.isConnected()) {
    if (!client.connected()) {
      if (serverAvailable) {
        Serial.println("Connection to server lost.");
        htmlLog("Connection to server lost.");
      }
      serverAvailable = false;
      serverConfirmed = false;
      client.stop();
      unsigned long timeout = millis()+5000;
      while ( (!ntp_done) && (millis()<timeout) ) delay(100);
      serverAvailable = connectToServer(HOST, PORT);
    }
  }
}

void getTcpData() {
  if (serverAvailable) {
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

void sendCanDataToServer() {
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
    } else {
      cnt_tcp_tx_retries++;
      sprintf(cbuf, "ERROR: TCP write() did not send all data: size=%d; sent=%d; duration=%d us",size,sent,micros_diff);
      htmlLog(cbuf);
      Serial.println(cbuf);
    }
    cnt_tcp_tx_pending = cnt_can_rx_total - cnt_tcp_tx_total;
  }
}

void canOnReceive(int packetSize) {
  if (!serverAvailable) {
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
  if (packetSize > 8) {
    //Serial.print("Bad length of CAN frame. Skipping.");
    return;
  }

  uint16_t can_id = CAN.packetId();
  if ( (can_id<CAN_ID_RX_MIN) || (can_id>CAN_ID_RX_MAX) ) return;   // Filter for CAN IDs

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
            serverAvailable = false;
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
            serverAvailable = false;
            serverConfirmed = false;
            client.stop();
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
              Serial.println("MDNS-Responder gestartet.");
            }
            Serial.println("Hole NTP Zeit");
            configTzTime(TZ_INFO, NTP_SERVER); 
            getLocalTime(&local, 5000);
            lastday=local.tm_mday;
            localTimeUpdate();
            ntp_done = true;
            cnt_wifi_connect_incomplete = 0;
            server.begin();
            Serial.println("HTTP-Server gestartet.");
            break;
        case ARDUINO_EVENT_WIFI_STA_LOST_IP:
            Serial.println("Lost IP address and IP address is reset to 0");
            serverAvailable = false;
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
    }}

void loopCore0(void * pvParameters) {
  unsigned long millis_lastprint = 0;
  for (;;) {
    delay(50);
    localTimeUpdate();
    if ( (serverAvailable) && ((millis()-millis_lastprint)>10000) ) {
      millis_lastprint = millis();
      printStats();
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(100);

  memset(base_ptr,0, LEN_CAN_BUF);   // Initialize can buffer
  memset(tcp_buf, 0, LEN_TCP_BUF);   // Initialize tcp buffer

  Serial.println(PGM_INFO);
  htmlLog(PGM_INFO);
  sprintf(cbuf, "Start cannelloni server, e.g. for vcan0: cannelloni -I vcan0 -C s -p -t 20000 -l %d",PORT);
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
  checkWiFi();              // Check WiFi connection and do a reconnect if needed
  checkClient();            // Check connection to cannelloni server and try to reconnect if needed
  if (serverAvailable) {
    getTcpData();           // Fetch data from cannelloni server and send it to local CAN bus
    sendCanDataToServer();  // Check for data available from local CAN bus and send it to cannelloni server
  }
  loopWebServer();
}
