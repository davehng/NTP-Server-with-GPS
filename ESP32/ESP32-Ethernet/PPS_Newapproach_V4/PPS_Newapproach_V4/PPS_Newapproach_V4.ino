#include <ETH.h>
#include <WiFiUdp.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <lwip/def.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// --- CONFIGURATION ---
#define GPS_RX_PIN       15
#define GPS_TX_PIN       4
#define PPS_PIN          2
#define ETH_ADDR         1
#define ETH_POWER_PIN    16
#define ETH_MDC_PIN      23
#define ETH_MDIO_PIN     18
#define ETH_TYPE         ETH_PHY_LAN8720
#define ETH_CLK_MODE     ETH_CLOCK_GPIO0_IN
#define GPS_BAUD         38400
#define CORRECTION_US    1210
#define NTP_EPOCH_OFFSET 2208988800UL

// --- OBJECTS & GLOBALS ---
TinyGPSPlus gps;
WiFiUDP udp;
HardwareSerial gpsSerial(1);
volatile uint32_t lastPPSMicros = 0;

// NTP Packet Structure
typedef struct {
  uint8_t li_vn_mode; uint8_t stratum; uint8_t poll; uint8_t precision;
  uint32_t rootDelay; uint32_t rootDispersion; uint32_t refId;
  uint32_t refTm_s; uint32_t refTm_f;
  uint32_t origTm_s; uint32_t origTm_f;
  uint32_t rxTm_s; uint32_t rxTm_f;
  uint32_t txTm_s; uint32_t txTm_f;
} ntp_packet_t;

// --- INTERRUPT SERVICE ROUTINE ---
void IRAM_ATTR PPS_ISR() {
  lastPPSMicros = micros();
}

// --- INITIALIZATION ---
void initNetwork() {
  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, LOW); delay(100);
  digitalWrite(ETH_POWER_PIN, HIGH); delay(100);
  
  ETH.begin(ETH_TYPE, ETH_ADDR, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_POWER_PIN, ETH_CLK_MODE);
  
  // Minimal wait for IP (Blocking here is acceptable for network reliability)
  while (ETH.localIP() == IPAddress(0,0,0,0)) { delay(500); }
  
  Serial.print("IP: "); Serial.println(ETH.localIP());
  udp.begin(123);
}

// --- TASKS ---
void readGPSTime() {
  uint32_t localPPS = lastPPSMicros; // Snapshot
  uint32_t nowMicros = micros();
  
  // Pump Serial into GPS object
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  // Only proceed if we have a fresh, valid fix
  if (gps.time.isUpdated() && gps.time.isValid()) {
    
    // Sanity Check: PPS should have fired ~200-900ms ago
    uint32_t delta = nowMicros - localPPS;
    if (delta > 900000 || delta < 1000) return; 

    // Prepare data
    struct tm t = {0};
    t.tm_year = gps.date.year() - 1900;
    t.tm_mon  = gps.date.month() - 1;
    t.tm_mday = gps.date.day();
    t.tm_hour = gps.time.hour();
    t.tm_min  = gps.time.minute();
    t.tm_sec  = gps.time.second();
    
    time_t gpsSecs = mktime(&t);
    struct timeval now;
    gettimeofday(&now, NULL);

    // Calculate Offset
    struct timeval adj;
    adj.tv_sec  = gpsSecs - now.tv_sec;
    adj.tv_usec = (delta + CORRECTION_US) - now.tv_usec;

    // Normalize
    adj.tv_sec  += adj.tv_usec / 1000000;
    adj.tv_usec %= 1000000;
    if (adj.tv_usec < 0) { adj.tv_usec += 1000000; adj.tv_sec--; }

    // Apply Time
    if (abs(adj.tv_sec) >= 2) {
       // Hard Set
       struct timeval tv = { .tv_sec = gpsSecs, .tv_usec = delta + CORRECTION_US };
       tv.tv_sec += tv.tv_usec / 1000000; tv.tv_usec %= 1000000;
       settimeofday(&tv, NULL);
       Serial.println("Hard Sync");
    } else {
       // Slew
       adjtime(&adj, NULL);
    }
  }
}

void handleNTPRequest() {
  if (udp.parsePacket() >= sizeof(ntp_packet_t)) {
    ntp_packet_t packet;
    udp.read((uint8_t*)&packet, sizeof(ntp_packet_t));

    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    // Convert to NTP Timestamp format
    uint32_t s = htonl(tv.tv_sec + NTP_EPOCH_OFFSET);
    uint32_t f = htonl((uint32_t)((double)tv.tv_usec * 4294.967296)); 

    // Construct Response
    packet.li_vn_mode = 0x24; // Server, Stratum 1
    packet.stratum = 1;
    packet.refId = htonl(0x47505300); // "GPS"
    packet.origTm_s = packet.txTm_s; // Copy client transmit time
    packet.origTm_f = packet.txTm_f;
    packet.rxTm_s = s; packet.rxTm_f = f;
    packet.txTm_s = s; packet.txTm_f = f;

    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write((uint8_t*)&packet, sizeof(ntp_packet_t));
    udp.endPacket();
  }
}

// --- MAIN LOOPS ---
void gpsTask(void *p) { 
  while(1) { 
    readGPSTime(); 
    vTaskDelay(pdMS_TO_TICKS(10)); 
  } 
}

void ntpTask(void *p) { 
  while(1) { 
    handleNTPRequest(); 
    vTaskDelay(pdMS_TO_TICKS(1)); 
  } 
}

void setup() {
  Serial.begin(115200);
  
  // Hardware Init (Non-blocking)
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), PPS_ISR, RISING);
  
  initNetwork();

  // Tasks
  xTaskCreatePinnedToCore(gpsTask, "GPS", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(ntpTask, "NTP", 4096, NULL, 3, NULL, 0);
}

void loop() { 
  delay(1000); 
}