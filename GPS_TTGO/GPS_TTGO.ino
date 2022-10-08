//Include libraries 
#include <lmic.h>
#include <hal/hal.h>
//#include <SPI.h>
//#include <SoftWire.h> <-- instalar
#include <SSD1306.h>
#include <TinyGPS++.h>
#include <axp20x.h>

// Oled Pin
#define PIN_SDA 21
#define PIN_SCL 22


TinyGPSPlus gps;                            
HardwareSerial GPS(1);
AXP20X_Class axp;

// GPS output
float latitude = 0 ;
float longitude = 0;
float altitudee = 0;
int satelites = 0;
String lati = "";
String longi = "";
String alti = "";
String sat = "";
bool conect = false;



SSD1306 display(0x3C, PIN_SDA, PIN_SCL);


// Little-indian format APPEUI
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
// Little endian format, DEVEUI
static const u1_t PROGMEM DEVEUI[8] = { 0x99, 0x68, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
// big indian format APPKEY
static const u1_t PROGMEM APPKEY[16] = { 0xF2, 0x7E, 0xC7, 0x1B, 0x5A, 0x5B, 0xB8, 0x57, 0xDF, 0x1D, 0xCC, 0x83, 0x14, 0x81, 0xC9, 0x46 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// payload to send to TTN gateway
static uint8_t payload[8];
static osjob_t sendjob;

// Schedule TX every this many seconds 
const unsigned TX_INTERVAL = 60;

// Pin mapping for Lily TTGO T-beam
const lmic_pinmap lmic_pins = {
  .nss = 18, 
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
};


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            display.print("TTN: Joining...");
            display.display();
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            display.clear();
            display.drawString(0, 0, "TTN: Connected");
            display.display();
            delay(2000);
            conect = true;
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              //LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (int i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (int i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            display.clear();
            display.setFont(ArialMT_Plain_10);
            display.drawString(0, 0, "TTN: Connected");
            display.drawString(0, 10, "Status: Sent!");
            display.drawString(0, 20, lati);
            display.drawString(0, 30, longi);
            display.drawString(0, 40, alti);
            display.drawString(0, 50, sat);
            display.display();
            Serial.println(F("EV_TXSTART"));
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            display.clear();
            display.setFont(ArialMT_Plain_10);
            display.drawString(0, 0, "TTN: Connected");
            display.drawString(0, 10, "Status: Sending");
            display.drawString(0, 20, lati);
            display.drawString(0, 30, longi);
            display.drawString(0, 40, alti);
            display.drawString(0, 50, sat);
            display.display();
            Serial.println(F("EV_TXSTART"));
            delay(2000);
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      
      
      uint32_t payloadlatitude = (latitude+500)*10000;
      payload[0] = payloadlatitude >> 16;
      payload[1] = payloadlatitude >> 8;
      payload[2] = payloadlatitude;

      uint32_t payloadlongitude = (longitude+500)*10000;
      payload[3] = payloadlongitude >> 16;
      payload[4] = payloadlongitude >> 8;
      payload[5] = payloadlongitude;
      
      // Upstream data
      LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
      Serial.println(F("EV_TXSTART"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    delay(5000);
    while (! Serial);
    Serial.begin(9600);
    Wire.begin(21, 22);
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin PASS");
      } else {
    Serial.println("AXP192 Begin FAIL");
    }
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    GPS.begin(9600, SERIAL_8N1, 34, 12);   //17-TX 18-RX
    display.init();
    display.setFont(ArialMT_Plain_10);
    delay(50);
    display.drawString( 10, 10, "Starting up ...");
    display.drawString( 10, 20, "- and initializing.");
    display.display();
    delay(500);


    // LMIC init
    os_init();
    LMIC_reset();
    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7,14);
    #if defined(CFG_eu868)
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    #elif defined(CFG_us915)
    LMIC_selectSubBand(1);
    #endif
    do_send(&sendjob);
}

 static void smartDelay(unsigned long ms)
  {
    unsigned long start = millis();
    do
    {
      while (GPS.available())
        gps.encode(GPS.read());
    } while (millis() - start < ms);
  }

void loop() {
  if (conect == true){
    Serial.print("Latitude  : ");
    Serial.println(gps.location.lat(), 5);
    Serial.print("Longitude : ");
    Serial.println(gps.location.lng(), 4);
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
    Serial.print("Altitude  : ");
    Serial.print(gps.altitude.feet() / 3.2808);
    Serial.println("M");
    Serial.print("Time      : ");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
    Serial.print("Speed     : ");
    Serial.println(gps.speed.kmph()); 
    Serial.println("**********************");
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    altitudee = gps.altitude.feet();
    satelites = gps.satellites.value();
    lati = "Lat = " + String(latitude);
    longi = "Lon = " + String(longitude);
    alti = "Alt = " + String(altitudee);
    sat = "Satelites = " + String(satelites);
    smartDelay(1000);
  
    if (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS data received: check wiring"));
  }
  os_runloop_once();
}
