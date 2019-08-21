/**
    device_config
    \brief Device setup config
 */

struct Storage{
  
  char ssid[24]          = "ssid";             // WiFi network Client name
  char password[24]      = "pass";             // WiFi network password
  // Ntrip Caster Data
  char host[40]          = "195.200.70.200";   // Server IP
  int  port              = 2101;               // Server Port
  char mountpoint[40]    = "FPS_BY_RTCM3_3G";  // Mountpoint
  char ntripUser[40]     = "NTRIPUsername";    // Username
  char ntripPassword[40] = "NTRIPPassword";    // Password

  byte sendGGAsentence = 2; // 0 = No Sentence will be sended
                            // 1 = fixed Sentence from GGAsentence below will be sended
                            // 2 = GGA from GPS will be sended
  
  byte GGAfreq =10;         // time in seconds between GGA Packets

  char GGAsentence[100] = "$GPGGA,051353.171,4751.637,N,01224.003,E,1,12,1.0,0.0,M,0.0,M,,*6B"; //hc create via www.nmeagen.org
  
  long baudOut = 9600;     // Baudrate of RTCM Port

  byte send_UDP_AOG  = 1;   // 0 = Transmission of NMEA Off
                            // 1 = Transmission of NMEA Sentences to AOG via Ethernet-UDP
                            // 2 = Bluetooth

  byte enableNtrip   = 1;   // 1 = NTRIP Client enabled
  
  byte AHRSbyte      = 0;   // 0 = No IMU, No Inclinometer
                            // 1 = BNO055 IMU installed
                            // 2 = MMA8452 Inclinometer installed
                            // 3 = BNO055 + MMA 8452 installed
}; Storage NtripSettings;

//WPA2-Enterprise
bool wpa_enterprise = false;	       // Connect to wpa2-enterprise
const char* peap_ssid = "peap_ssid"; // Wifi ssid
#define PEAP_IDENTITY "identity"     // PEAP identity
#define PEAP_PASSWORD "password"     // PEAP password

//static IP
bool static_ip = false;		           // Enable static IP
IPAddress myip(192, 168, 0, 201);    // Roofcontrol module
IPAddress gwip(192, 168, 0, 1);      // Gateway & Accesspoint IP
IPAddress mask(255, 255, 255, 0);    // Mask
IPAddress myDNS(8, 8, 8, 8);         // DNS
