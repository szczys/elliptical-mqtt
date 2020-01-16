#include <WiFiUdp.h>
#include <WiFi.h>

#include "driver/spi_slave.h"
#include <SPI.h>

#include <PubSubClient.h>

const char hexstring[] = "0123456789ABCDEF";
#define HEXBUFLEN 41
char hexstringbuff[HEXBUFLEN];
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

spi_slave_transaction_t * driver;
uint16_t t_size = 20;

const char* mqtt_server = "192.168.1.135";

char last[4] = "Off";
#define LIGHTOFF 0
#define LIGHTON 1
#define LIGHTUNKNOWN 2

WiFiClient espClient;
PubSubClient client(espClient);

void clrhexstringbuff() {
  for (uint8_t i=0; i<HEXBUFLEN; i++) hexstringbuff[i] = 0;
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
                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  pinMode(0, INPUT);

  clrhexstringbuff();
  sendmsgflag = 0;

  SPI.begin();
  gpio_set_pull_mode((gpio_num_t)SCLK, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)SS, GPIO_PULLUP_ONLY);
  //Driver acts as buffer for incoming spi data, last null is optional, may be used to flag each transaction with your own data / variable.
  driver = new spi_slave_transaction_t{ (size_t)(t_size * 8) ,  0 , heap_caps_malloc(_max(t_size,32), MALLOC_CAP_DMA), heap_caps_malloc(_max(t_size,32), MALLOC_CAP_DMA),NULL };
  spi_bus_config_t buscfg = {
    buscfg.mosi_io_num = SI,
    buscfg.miso_io_num = SO,
    buscfg.sclk_io_num = SCLK
  };
  spi_slave_interface_config_t slvcfg = { SS,0,1,0,setupIntr,transIntr };//check the IDF for further explanation
  spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, 1); //DMA channel 1
  spi_slave_queue_trans(HSPI_HOST, driver, portMAX_DELAY);//ready for input (no transmit)
  //exter_intr = ext;   
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
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

//Callback called after the SPI registers are loaded with new data.
void setupIntr(spi_slave_transaction_t * trans) {
  //I do not use this...
}

//Callback called after a transaction is done.
void transIntr(spi_slave_transaction_t * trans) {
  byte value = 0; 
  uint16_t largo = driver->trans_len;
  largo = largo / 8; //this value is in bits
  Serial.println("INCOMING: ");
  for (int i = 0; i<largo; i++)
  {
    value = ((char*)driver->rx_buffer)[i];
    Serial.print(value, HEX);
    Serial.print(',');
    
    //Store hex values as a concatenated string
    hexstringbuff[2*i] = hexstring[value >> 4];
    hexstringbuff[(2*i)+1] = hexstring[value & 0xF];
  }
  Serial.println(" --- " + (String)largo);
  //
  ++sendmsgflag;

  
  //Set it to listen again into Master SPI
  driver->length = t_size * 8;
  driver->trans_len = 0;
  spi_slave_queue_trans(HSPI_HOST, driver, portMAX_DELAY);
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
  //digitalWrite(led,0);
  //delay(50);

  if (sendmsgflag) {
    sendmsgflag = 0;
    Serial.print(hexstringbuff);
    client.publish(topic, hexstringbuff);
    clrhexstringbuff();
  }
  /*
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second
  */
}
