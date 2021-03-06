#include <Arduino.h>
#include <ETH.h>
#include <SPI.h>
#include <HTTPClient.h>

#include <WiFiUdp.h> 

#include "mbedtls/aes.h"
#include "SPIFFSTest.h"
#include "Cipher.h"

CSPIFFS mSpiffs;
Cipher * cipher = new Cipher();

/*
 * ETH_CLOCK_GPIO0_IN   - default: external clock from crystal oscillator
 * ETH_CLOCK_GPIO0_OUT  - 50MHz clock from internal APLL output on GPIO0 - possibly an inverter is needed for LAN8720
 * ETH_CLOCK_GPIO16_OUT - 50MHz clock from internal APLL output on GPIO16 - possibly an inverter is needed for LAN8720
 * ETH_CLOCK_GPIO17_OUT - 50MHz clock from internal APLL inverted output on GPIO17 - tested with LAN8720
 */

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT

// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_POWER_PIN -1

// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE ETH_PHY_LAN8720

// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR 0

// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN 23

// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN 18

#define NRST 5

static bool eth_connected = false;
static bool eth_done = false;

WiFiUDP Udp;                      // create UDP object
unsigned int localUdpPort = 2333; // Local port number

IPAddress local_ip(192,168,0,145);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);
IPAddress dns1(8,8,8,8);
IPAddress dns2 = (uint32_t)0x00000000;

void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_ETH_START:
    Serial.println("ETH Started");
    // set eth hostname here
    ETH.setHostname("esp32-ethernet");
    break;
  case SYSTEM_EVENT_ETH_CONNECTED:
    Serial.println("ETH Connected");
    break;
  case SYSTEM_EVENT_ETH_GOT_IP:
    Serial.print("ETH MAC: ");
    Serial.print(ETH.macAddress());
    Serial.print(", IPv4: ");
    Serial.print(ETH.localIP());
    if (ETH.fullDuplex())
    {
      Serial.print(", FULL_DUPLEX");
    }
    Serial.print(", ");
    Serial.print(ETH.linkSpeed());
    Serial.println("Mbps");
    eth_connected = true;
    break;
  case SYSTEM_EVENT_ETH_DISCONNECTED:
    Serial.println("ETH Disconnected");
    eth_connected = false;
    break;
  case SYSTEM_EVENT_ETH_STOP:
    Serial.println("ETH Stopped");
    eth_connected = false;
    break;
  default:
    break;
  }
}

void testClient(const char *host, uint16_t port)
{
  Serial.print("\nconnecting to ");
  Serial.println(host);

  WiFiClient client;
  if (!client.connect(host, port))
  {
    Serial.println("connection failed");
    return;
  }
  client.printf("GET / HTTP/1.1\r\nHost: %s\r\n\r\n", host);
  while (client.connected() && !client.available())
    ;
  while (client.available())
  {
    Serial.write(client.read());
  }

  Serial.println("closing connection\n");
  client.stop();
}

void ETH_test(void)
{
  pinMode(NRST, OUTPUT);
  digitalWrite(NRST, 0);
  delay(200);
  digitalWrite(NRST, 1);
  delay(200);
  digitalWrite(NRST, 0);
  delay(200);
  digitalWrite(NRST, 1);

  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
}

void setup()
{
  Serial.begin(921000);
  Serial.setRxBufferSize(1500);
  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, 1);


  char * key = "abcdefghijklmnop";
  String plainText = "Tech tutorials xTech tutorials xxyzgvszufsdgftzsdfgsdfzfsfdzfsdzfsdtzfdtzsfdtzsfdtzfsdtzfstzfzsfdfzs";

  Serial.print("\nSetting cipher key: ");
  Serial.println(key);
  cipher->setKey(key);

  Serial.println("\nOriginal plain text:");
  Serial.println(plainText);

  Serial.println("\nCiphered text:");
  String text = cipher->encryptString(plainText);
  Serial.println(text);

  Serial.println("\nDeciphered text:");
  Serial.println(cipher->decryptString(text));


  delay(2000);

  WiFi.onEvent(WiFiEvent);
  ETH_test();

  ETH.config(local_ip, gateway, subnet, dns1, dns2); // Static IP, leave without this line to get IP via DHCP
  Udp.begin(localUdpPort); // Enable UDP listening to receive data

  

}

#define stream
//#define debug

#define UDP2UART
//#define UART2UDP

//#define ENCRYPT

bool flagToSend=false;
int packetSize=1300;
long timeoutPackage=100; //ms
long counterPackage=millis();
char buf[1500];
int counter=0;

void loop()
{

#ifdef UDP2UART

  int packetSize = Udp.parsePacket(); // Get the current team header packet length
  if (packetSize)                     // If data is available
  {
    char buf[packetSize];
    Udp.read(buf, packetSize); // Read the current packet data
    //Udp.readBytes(buf,packetSize);

  #ifdef debug
    Serial.println();
    Serial.print("Received: ");
    Serial.println(buf);
    Serial.print("PacketSize: ");
    Serial.println(packetSize);
    Serial.print("From IP: ");
    Serial.println(Udp.remoteIP());
    Serial.print("From Port: ");
    Serial.println(Udp.remotePort());

    //cipher->setKey(key);
    String text = cipher->encryptString(buf);
    Serial.println("\n Ciphered text:");
    Serial.print(text);
    Serial.println("\n Deciphered text:");
    Serial.println(cipher->decryptString(text));
    //Serial.print(plainText);

  #endif

  #ifdef stream

    #ifdef ENCRYPT
    String text = cipher->encryptString(buf);
    Serial.print(text);
    #endif
    #ifndef ENCRYPT
    Serial.print(buf);
    #endif

  #endif

    /*
    //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); // Ready to send data
    Udp.beginPacket("192.168.1.159", 2222); // Ready to send data
    //Udp.print("Received: ");    // Copy data to send buffer
    Udp.write((const uint8_t*)buf, packetSize); // Copy data to send buffer
    Udp.endPacket();            // send data
    */

  }

#endif

#ifdef UART2UDP
  if (Serial.available()>0){
    
    counterPackage=millis();
    counter=0;

    while(millis()-counterPackage<timeoutPackage){ // set package timeout for 100m    
      if(Serial.available()){
        buf[counter++]=Serial.read();
      }
      if (counter>=packetSize) break;
    }
    flagToSend=true;
    
  }

  if(flagToSend){
    Udp.beginPacket("192.168.0.200", 2222); // Ready to send data
    Udp.write((const uint8_t*)buf, counter);   // Copy data to send buffer
    Udp.endPacket();
    flagToSend=false;
    counter=0;
  }                        // send data

#endif

}