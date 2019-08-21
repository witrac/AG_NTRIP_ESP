TaskHandle_t Core1;
TaskHandle_t Core2;
// ESP32 Ntrip Client by Coffeetrac
// Release: V1.25
// 01.01.2019 W.Eder
// Enhanced by Matthias Hammer 12.01.2019

boolean debugmode = false;

// IO pins --------------------------------
#define RX0      3
#define TX0      1

#define RX1     26  //simpleRTK TX(xbee) = RX(f9p)
#define TX1     25  //simpleRTK RX(xbee) = TX(f9p)

#define RX2     17  
#define TX2     16 

#define SDA     21  //I2C Pins
#define SCL     22

//########## BNO055 adress 0x28 ADO = 0 set in BNO_ESP.h means ADO -> GND
//########## MMA8451 adress 0x1D SAO = 0 set in MMA8452_AOG.h means SAO open (pullup!!)

#define restoreDefault_PIN 36  // set to 1 during boot, to restore the default values


//libraries -------------------------------
#include <Wire.h>
#include <WiFi.h>
#include <base64.h>
#include "Network_AOG.h"
#include "EEPROM.h"
#include "BNO_ESP.h"
#include "MMA8452_AOG.h"
#include "BluetoothSerial.h"
#include "device_config.h"
#include "esp_wpa2.h"

// Declarations
void DBG(String out, byte nl = 0);

//Accesspoint name and password:
const char* ssid_ap     = "NTRIP_Client_ESP_Net";
const char* password_ap = "";

unsigned int portMy = 5544;       //this is port of this module: Autosteer = 5577 IMU = 5566 GPS = 
unsigned int portAOG = 8888;      // port to listen for AOG

//IP address to send UDP data to:
IPAddress ipDestination(192, 168, 1, 255);
unsigned int portDestination = 9999;  // Port of AOG that listens

// Variables ------------------------------
// program flow
bool AP_running=0, EE_done = 0, restart=0;
int value = 0; 
unsigned long repeat_ser;   
//int error = 0;
unsigned long repeatGGA, lifesign;

//loop time variables in microseconds
const unsigned int LOOP_TIME = 100; //10hz 
unsigned int lastTime = LOOP_TIME;
unsigned int currentTime = LOOP_TIME;
unsigned int dT = 50000;

//Kalman variables
float rollK = 0, Pc = 0.0, G = 0.0, P = 1.0, Xp = 0.0, Zp = 0.0;
float XeRoll = 0;
const float varRoll = 0.1; // variance,
const float varProcess = 0.0055; //0,00025 smaller is more filtering

// GPS-Bridge
int cnt=0;
int i=0;  
byte gpsBuffer[100], c;
char imuBuffer[20];
bool newSentence = false;
bool newIMUSentence = false;
char lastSentence[100]="";

char strmBuf[512];         // rtcm Message Buffer

 //IMU, inclinometer variables
  bool imu_initialized = 0;
  int16_t roll = 0, roll_corr = 0;
  uint16_t x_ , y_ , z_;

//Array to send data back to AgOpenGPS
byte GPStoSend[100]; 
byte IMUtoSend[] = {0x7F,0xEE,0,0,0,0,0,0,0,0};
byte IMUtoSendLenght = 10; //lenght of array to AOG

// Instances ------------------------------
MMA8452 accelerometer;
WiFiServer server(80);
WiFiClient ntripCl;
WiFiClient client_page;
AsyncUDP udp;
BluetoothSerial SerialBT;


// Setup procedure ------------------------
void setup() {
  pinMode(restoreDefault_PIN, INPUT);  //
  
  restoreEEprom();
  Wire.begin(SDA, SCL, 400000);

  //  Serial1.begin (NtripSettings.baudOut, SERIAL_8N1, RX1, TX1); 
  if (debugmode) { Serial1.begin(115200, SERIAL_8N1, RX1, TX1); } //set new Baudrate
  else { Serial1.begin(NtripSettings.baudOut, SERIAL_8N1, RX1, TX1); } //set new Baudrate
  Serial2.begin(115200,SERIAL_8N1,RX2,TX2); 

  Serial.begin(115200);
  if(!SerialBT.begin("BT_GPS_ESP")){
      DBG("\nAn error occurred initializing Bluetooth\n");
  }
  
 
  
  //------------------------------------------------------------------------------------------------------------  
  //create a task that will be executed in the Core1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(Core1code, "Core1", 10000, NULL, 1, &Core1, 0);
  delay(500); 
  //create a task that will be executed in the Core2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(Core2code, "Core2", 10000, NULL, 1, &Core2, 1); 
  delay(500); 
  //------------------------------------------------------------------------------------------------------------
 
}

void loop() {
}

//--------------------------------------------------------------
//  Debug Messaging
//--------------------------------------------------------------
bool debug = 1;  // Print Debug Messages to Serial0

void DBG(String out, byte nl){
  if (debug == 1) {
    if (nl) Serial.println(out);
    else Serial.print(out);
  }
}

void DBG(int out, byte nl = 0){
  if (debug == 1) {
    if (nl) Serial.println(out);
    else Serial.print(out);
  }
}

void DBG(long out, byte nl = 0){
  if (debug == 1) {
    if (nl) Serial.println(out);
    else Serial.print(out);
  }
}

void DBG(char out, byte nl = 0){
  if (debug == 1) {
    if (nl) Serial.println(out);
    else Serial.print(out);
  }
}

void DBG(char out, byte type, byte nl = 0){ // type = HEX,BIN,DEZ..
  if (debug == 1) {
    if (nl) Serial.println(out,type);
    else Serial.print(out,type);
  }
}

void DBG(IPAddress out, byte nl = 0){
  if (debug == 1) {
    if (nl) Serial.println(out);
    else Serial.print(out);
  }
}
