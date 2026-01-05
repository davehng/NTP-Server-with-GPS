#include <ETH.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <lwip/def.h>
#include <lwip/sockets.h> // Required for BSD Sockets
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// --- CONFIGURATION ---
// Pins (WT32-ETH01)
#define GPS_RX_PIN       15
#define GPS_TX_PIN       4
#define PPS_PIN          2
#define ETH_ADDR         1
#define ETH_POWER_PIN    16
#define ETH_MDC_PIN      23
#define ETH_MDIO_PIN     18
#define ETH_TYPE         ETH_PHY_LAN8720
#define ETH_CLK_MODE     ETH_CLOCK_GPIO0_IN

// Settings
#define GPS_BAUD         38400
#define CORRECTION_US    1210
#define NTP_EPOCH_OFFSET 2208988800UL
#define NTP_PORT         123

// --- OBJECTS & GLOBALS ---
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
volatile uint32_t lastPPSMicros = 0;

// NTP Packet Structure
typedef struct {
  uint8_t li_vn_mode; 
  uint8_t stratum; 
  uint8_t poll; 
  uint8_t precision;
  uint32_t rootDelay; 
  uint32_t rootDispersion; 
  uint32_t refId;
  uint32_t refTm_s; 
  uint32_t refTm_f;
  uint32_t origTm_s; 
  uint32_t origTm_f;
  uint32_t rxTm_s; 
  uint32_t rxTm_f;
  uint32_t txTm_s; 
  uint32_t txTm_f;
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
  
  // Minimal wait for IP (Blocking here is acceptable for startup reliability)
  while (ETH.localIP() == IPAddress(0,0,0,0)) { delay(500); }
  
  Serial.print("IP: "); Serial.println(ETH.localIP());
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

// High Performance Blocking NTP Task
void ntpTask(void *parameter) {
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock < 0) { 
    Serial.println("Socket failed"); 
    vTaskDelete(NULL); 
  }

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(NTP_PORT);
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    Serial.println("Bind failed"); 
    vTaskDelete(NULL);
  }

  while (1) {
    ntp_packet_t packet;
    struct sockaddr_in source_addr;
    socklen_t addrLen = sizeof(source_addr);

    // RECVFROM: Blocks until data arrives (Zero Polling Jitter)
    int len = recvfrom(sock, &packet, sizeof(packet), 0, (struct sockaddr *)&source_addr, &addrLen);

    if (len == sizeof(ntp_packet_t)) {
      struct timeval tv;
      gettimeofday(&tv, NULL);
      
      // Convert to NTP Format
      uint32_t s = htonl(tv.tv_sec + NTP_EPOCH_OFFSET);
      uint32_t f = htonl((uint32_t)((double)tv.tv_usec * 4294.967296)); 

      // Fill Response
      packet.li_vn_mode = 0x24; // Server, Stratum 1
      packet.stratum = 1;
      packet.refId = htonl(0x47505300); // "GPS"
      
      // Originate = Client's Transmit Time
      packet.origTm_s = packet.txTm_s; 
      packet.origTm_f = packet.txTm_f;
      
      // RX and TX times are effectively "now" for this simplified server
      packet.rxTm_s = s; packet.rxTm_f = f;
      packet.txTm_s = s; packet.txTm_f = f;

      sendto(sock, &packet, sizeof(packet), 0, (struct sockaddr *)&source_addr, addrLen);
    }
  }
}

// --- MAIN LOOPS ---
void gpsTask(void *p) { 
  while(1) { 
    readGPSTime(); 
    vTaskDelay(pdMS_TO_TICKS(10)); 
  } 
}

void setup() {
  // 1. Lock CPU to max freq to prevent power-saving latency
  setCpuFrequencyMhz(240);
  
  Serial.begin(115200);
  
  // 2. Hardware Init
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), PPS_ISR, RISING);
  
  initNetwork();

  // 3. Start Tasks
  // GPS Maintenance: Priority 2 (Standard)
  xTaskCreatePinnedToCore(gpsTask, "GPS", 4096, NULL, 2, NULL, 1);
  
  // NTP Server: Priority 20 (High Real-Time Priority)
  xTaskCreatePinnedToCore(ntpTask, "NTP", 4096, NULL, 20, NULL, 1);
}

void loop() { 
  delay(1000); 
}