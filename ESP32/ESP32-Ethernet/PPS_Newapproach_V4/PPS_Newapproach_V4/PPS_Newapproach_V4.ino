#include <ETH.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <lwip/def.h>
#include <lwip/sockets.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// --- NEW INCLUDES FOR PRODUCTION FEATURES ---
#include <esp_task_wdt.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

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
#define STATUS_LED_PIN   33 

// Settings
#define GPS_BAUD         38400
#define CORRECTION_US    1210
#define NTP_EPOCH_OFFSET 2208988800UL
#define NTP_PORT         123
#define WDT_TIMEOUT      30   // Reboot if frozen for 30 seconds

// --- GLOBALS ---
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
volatile uint32_t lastPPSMicros = 0;
volatile uint32_t lastSyncSec = 0;
volatile uint32_t lastSyncFrac = 0;

typedef struct {
  uint8_t li_vn_mode; uint8_t stratum; uint8_t poll; uint8_t precision;
  uint32_t rootDelay; uint32_t rootDispersion; uint32_t refId;
  uint32_t refTm_s; uint32_t refTm_f;
  uint32_t origTm_s; uint32_t origTm_f;
  uint32_t rxTm_s; uint32_t rxTm_f;
  uint32_t txTm_s; uint32_t txTm_f;
} ntp_packet_t;

// --- ERROR RECOVERY ---
void fatalError(const char* msg) {
  Serial.print("FATAL: "); Serial.println(msg);
  Serial.println("Restarting system in 2 seconds...");
  delay(2000); 
  ESP.restart();
}

// --- ISR ---
void IRAM_ATTR PPS_ISR() {
  lastPPSMicros = micros();
}

// --- INIT ---
void initNetwork() {
  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, LOW); delay(100);
  digitalWrite(ETH_POWER_PIN, HIGH); delay(100);
  
  if (!ETH.begin(ETH_TYPE, ETH_ADDR, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_POWER_PIN, ETH_CLK_MODE)) {
    fatalError("Ethernet PHY failed to start");
  }

  // Timeout recovery for DHCP
  unsigned long start = millis();
  while (ETH.localIP() == IPAddress(0,0,0,0)) { 
    if (millis() - start > 30000) fatalError("DHCP Timeout");
    delay(500); 
  }
  Serial.print("IP: "); Serial.println(ETH.localIP());
}

void initOTA() {
  ArduinoOTA.setHostname("Stratum1-NTP-Server");
  
  // Optional: Set password for updates
  // ArduinoOTA.setPassword("admin"); 

  ArduinoOTA.onStart([]() {
    Serial.println("Start updating...");
    // Detach interrupt to prevent crashes during flash write
    detachInterrupt(digitalPinToInterrupt(PPS_PIN));
  });
  
  ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    ESP.restart();
  });

  ArduinoOTA.begin();
}

// --- TASKS ---
void readGPSTime() {
  // 1. REGISTER WATCHDOG (Protect against GPS hardware hang)
  esp_task_wdt_add(NULL);

  uint32_t localPPS = lastPPSMicros;
  uint32_t nowMicros = micros();
  static int previousSecond = -1;

  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  if (gps.time.isUpdated() && gps.time.isValid()) {
    int currentSecond = gps.time.second();
    if (currentSecond == previousSecond) return;
    previousSecond = currentSecond;

    uint32_t delta = nowMicros - localPPS;
    if (delta > 900000 || delta < 1000) return; 

    // Blink LED to indicate "Pulse Valid"
    static bool ledState = false;
    digitalWrite(STATUS_LED_PIN, ledState = !ledState);

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

    struct timeval adj;
    adj.tv_sec  = gpsSecs - now.tv_sec;
    adj.tv_usec = (delta + CORRECTION_US) - now.tv_usec;

    adj.tv_sec  += adj.tv_usec / 1000000;
    adj.tv_usec %= 1000000;
    if (adj.tv_usec < 0) { adj.tv_usec += 1000000; adj.tv_sec--; }

    if (abs(adj.tv_sec) >= 2) {
       struct timeval tv = { .tv_sec = gpsSecs, .tv_usec = delta + CORRECTION_US };
       tv.tv_sec += tv.tv_usec / 1000000; tv.tv_usec %= 1000000;
       settimeofday(&tv, NULL);
    } else {
       adjtime(&adj, NULL);
    }

    lastSyncSec  = htonl(gpsSecs + NTP_EPOCH_OFFSET);
    lastSyncFrac = htonl((uint32_t)((double)(delta + CORRECTION_US) * 4294.967296));
  }
  
  // 2. KICK THE DOG (Tell hardware we are still alive)
  esp_task_wdt_reset();
}

void ntpTask(void *parameter) {
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock < 0) fatalError("Unable to create socket");

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(NTP_PORT);
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    fatalError("Unable to bind to port 123");
  }

  while (1) {
    ntp_packet_t packet;
    struct sockaddr_in source_addr;
    socklen_t addrLen = sizeof(source_addr);

    memset(&packet, 0, sizeof(ntp_packet_t));

    int len = recvfrom(sock, &packet, sizeof(packet), 0, (struct sockaddr *)&source_addr, &addrLen);

    if (len == sizeof(ntp_packet_t)) {
      struct timeval tv;
      gettimeofday(&tv, NULL);
      
      uint32_t s = htonl(tv.tv_sec + NTP_EPOCH_OFFSET);
      uint32_t f = htonl((uint32_t)((double)tv.tv_usec * 4294.967296)); 

      packet.li_vn_mode = 0x24; 
      packet.stratum = 1;
      packet.refId = htonl(0x47505300); // "GPS"
      packet.refTm_s = lastSyncSec;
      packet.refTm_f = lastSyncFrac;
      packet.origTm_s = packet.txTm_s; 
      packet.origTm_f = packet.txTm_f;
      packet.rxTm_s = s; packet.rxTm_f = f;
      packet.txTm_s = s; packet.txTm_f = f;

      sendto(sock, &packet, sizeof(packet), 0, (struct sockaddr *)&source_addr, addrLen);
    } 
    else if (len < 0 && errno != EAGAIN && errno != EINTR) {
       fatalError("Socket crashed");
    }
  }
}

void gpsTask(void *p) { 
  while(1) { 
    readGPSTime(); 
    vTaskDelay(pdMS_TO_TICKS(10)); 
  } 
}

void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  
  // 1. INIT WATCHDOG (Must happen early)
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,  // Convert seconds to ms
      .idle_core_mask = (1 << 0) | (1 << 1), // Watch both Core 0 and Core 1
      .trigger_panic = true              // Reset on timeout
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);               // Watch the current task (setup/loop)

  pinMode(STATUS_LED_PIN, OUTPUT);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), PPS_ISR, RISING);
  
  initNetwork();
  initOTA(); // Start Over-The-Air updater

  xTaskCreatePinnedToCore(gpsTask, "GPS", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(ntpTask, "NTP", 4096, NULL, 20, NULL, 1);
}

void loop() { 
  // Handle OTA updates
  ArduinoOTA.handle(); 
  
  // Feed the watchdog for the main loop
  esp_task_wdt_reset();
  
  delay(100); 
}