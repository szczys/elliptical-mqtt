#include <WiFiUdp.h>
#include <WiFi.h>

#include "driver/spi_slave.h"
#include <SPI.h>

#include <PubSubClient.h>

const char hexstring[] = "0123456789ABCDEF";
#define HEXBUFLEN 41
char hexstringbuff[HEXBUFLEN];
char lasthex[HEXBUFLEN];
uint8_t sendmsgflag;

// give it a name:
int led = 2;

/*Define these in the wificredentials.h file */
#include "wificredentials.h"
const char* ssid     = WIFISSID;
const char* password = WIFIPWD;
/********this file is excluded by.gitignore***/
const char* topic = "exercise/elliptical";

// Using ESP32 HSPI (SPI2)
// CS is SS --> 
// WR is SCK --> 
// DATA is MOSI --> 
#define SO    12
#define SI    13
#define SCLK  14
#define SS    15

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define BYTEARRAYLEN  16
volatile uint16_t bitcount;
volatile uint8_t bytearray[BYTEARRAYLEN];
volatile uint8_t messageFlag;
volatile uint8_t curbyte;
volatile uint8_t bytefiller;

const char* mqtt_server = "192.168.1.135";

char last[4] = "Off";
#define LIGHTOFF 0
#define LIGHTON 1
#define LIGHTUNKNOWN 2

WiFiClient espClient;
PubSubClient client(espClient);

void clrhexbuff(char buff[]) {
  for (uint8_t i=0; i<HEXBUFLEN; i++) buff[i] = 0;
}

uint8_t samebuff(char buff_1[], char buff_2[]) {
  for (uint8_t i=0; i<HEXBUFLEN; i++) {
    if (buff_1[i] != buff_2[i]) return 0;
    if (buff_1[i] == 0) return 1;
  }
  return 1;
}

void copybuff(char from_buff[], char to_buff[]) {
  for (uint8_t i=0; i<HEXBUFLEN; i++) { to_buff[i] = from_buff[i]; }
}

void setup()
{
  Serial.begin(115200);
  delay(10);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  messageFlag = 0;
  clearByteArray();
  curbyte = 0;
  bytefiller = 0;

  pinMode(SS, INPUT_PULLUP);
  pinMode(SCLK, INPUT_PULLUP);
  pinMode(SI, INPUT_PULLUP);
  pinMode(2, OUTPUT);
                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  pinMode(0, INPUT);

  clrhexbuff(hexstringbuff);
  clrhexbuff(lasthex);
  sendmsgflag = 0;

  //Falling SS enables falling clock interrupt
  //Rising SS disabled falling clock and flags serial output
  attachInterrupt(digitalPinToInterrupt(SS), changeSS, CHANGE);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  last[0] = payload[length-3]; last[1] = payload[length-2]; last[2] = payload[length-1]; last[3] = 0;
  Serial.print(last[0]);Serial.print(last[1]);Serial.print(last[2]);Serial.print(last[3]);
  //Do Something here. Old code for future reference:
  //if (interpretLightCmd() == LIGHTOFF) { digitalWrite(led,0); }
  //else if (interpretLightCmd() == LIGHTON) { digitalWrite(led,1); }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("lighting/porchlight/status");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//Copy buffered message to hex values
void msgtohex() {
  byte value = 0; 
  for (int i = 0; i<BYTEARRAYLEN; i++)
  {
    value = bytearray[i];
    
    //Store hex values as a concatenated string
    hexstringbuff[2*i] = hexstring[value >> 4];
    hexstringbuff[(2*i)+1] = hexstring[value & 0xF];
  }
}

// the loop routine runs over and over again forever:
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  if (digitalRead(0)==0) {
    //digitalWrite(led,1);
    client.publish(topic, "Button");
    
    while (digitalRead(0)==0) ;;
  }

  if (messageFlag) {
    msgtohex(); //Loads hexstringbuff from bytearray
    if (samebuff(hexstringbuff,lasthex) == 0) {
      Serial.println(hexstringbuff);
      client.publish(topic, hexstringbuff);
      copybuff(hexstringbuff, lasthex);
      clrhexbuff(hexstringbuff);
    }
    messageFlag = 0;  //Throw out messages that came in while we were operating
  }
}

void clearByteArray(void) {
  for (uint8_t i=0; i<BYTEARRAYLEN; i++) {
    bytearray[i] = 0;
  }
}

void ICACHE_RAM_ATTR fallingSCLK(void) {
  GPIO.out_w1ts = (1 << GPIO_NUM_2);
  curbyte = (curbyte << 1) + ((GPIO_REG_READ(GPIO_IN_REG)&(BIT(GPIO_NUM_13)))!=0);
  if (++bitcount > 7) {
    bitcount = 0;
    bytearray[bytefiller++] = curbyte;
  }
  GPIO.out_w1tc = (1 << GPIO_NUM_2);
}

void changeSS(void) {
  if(digitalRead(SS)) {
    //Rising Edge
    //Serial.println("Rising Edge!");
    //Packet is done, stop listening to clock and flag for a completed message
    detachInterrupt(digitalPinToInterrupt(SCLK));
    //Shift in the remaining bits to the nearest byte
    if (bitcount) {
      bytearray[bytefiller] = (curbyte << (8-bitcount));   
    }

    //Tell main to print the message
    messageFlag = 1;
  }
  else {
    //Serial.println("Falling Edge!");
    //Falling Edge
    //Listen for falling clock and keep track of bit count
    if (messageFlag == 0) {
      //Only receive packet if we know the buffer is empty
      bitcount = 0;
      bytefiller = 0;
      attachInterrupt(digitalPinToInterrupt(SCLK), fallingSCLK, RISING);
    }
  }
}
