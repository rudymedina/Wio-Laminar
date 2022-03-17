#include "rpcWiFi.h" //Wi-Fi library 
#include "TFT_eSPI.h" //TFT LCD library 
#include "Free_Fonts.h" //free fonts library 
#include <SoftwareSerial.h>;
#include "DHT.h"
#include <OneWire.h>                
#include <DallasTemperature.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"


//declarated valors
float temp1 = 0;
float temp2 = 0;
float co2 = 0;
float valorLDR = 0;
float t1 = 0;
float t2 = 0;
float h1 = 0;
float h2 = 0;
#define pin1 D2
#define pin2 D3
float RX = BCM14;
float TX = BCM15;
int pinLDR = D0;

int ndispositivos = 0;
float tempC;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long interval = 70000;
unsigned long interval2 = 70000;
unsigned long interval3 = 70000;
unsigned long interval4 = 70000;
unsigned long interval5 = 70000;

unsigned long prevTime_T1 = millis();
unsigned long prevTime_T2 = millis();
unsigned long prevTime_T3 = millis(); 
unsigned long prevTime_T4 = millis(); 
unsigned long prevTime_T5 = millis();

TFT_eSPI tft; //initialize TFT LCD
TFT_eSprite spr = TFT_eSprite(&tft); //initialize sprite buffer

//PINES PARA CONEXION ESP32-----------
OneWire ourWire(D4);       //A1-i04-4           
DHT dht1(pin1, DHT22);    //D2-i026-26
DHT dht2(pin2, DHT22);    //D6-i027-27
SoftwareSerial myCo2(RX, TX); // RX=D10-io5-naranja, TX=D11-io23-cafe
DallasTemperature ds18(&ourWire); //Se declara una variable u objeto para nuestro sensor
DeviceAddress address1 = { 0x28, 0xD, 0x85, 0x67, 0xD, 0x0, 0x0, 0x4B };
DeviceAddress address2 = { 0x28, 0x3B, 0x3E, 0x67, 0xD, 0x0, 0x0, 0x2B };
byte request[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
unsigned char response[9];
// Configuraciones del sistema----------------------
#define WLAN_SSID       "Fresh" 
#define WLAN_PASS       "OKM690wsx" // OKM690wsx
#define MQTT_SERVER      "35.184.160.240" 
#define MQTT_SERVERPORT  1883
#define MQTT_USERNAME    ""
#define MQTT_KEY         ""
#define MQTT_PUBLIC_CO2      "DarMal/Laminar/L3C0"
#define MQTT_PUBLIC_TEMP_0   "DarMal/Laminar/L3T0" 
#define MQTT_PUBLIC_HUMI_0   "DarMal/Laminar/L3H0" 
#define MQTT_PUBLIC_TEMP_1   "DarMal/Laminar/L3T1"
#define MQTT_PUBLIC_HUMI_1   "DarMal/Laminar/L3H1"
#define MQTT_PUBLIC_DS18_0   "DarMal/Laminar/L3T2"
#define MQTT_PUBLIC_DS18_1   "DarMal/Laminar/L3T3"

// conexion y rutamiento de topics para los sensores--------
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_USERNAME, MQTT_KEY);
Adafruit_MQTT_Publish temp_0_hdt_0 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_TEMP_0);
Adafruit_MQTT_Publish hum_0_hdt_0 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_HUMI_0);
Adafruit_MQTT_Publish temp_1_hdt_1 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_TEMP_1);
Adafruit_MQTT_Publish hum_1_hdt_1 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_HUMI_1);
Adafruit_MQTT_Publish ds18_0 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_DS18_0);
Adafruit_MQTT_Publish ds18_1 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_DS18_1);
Adafruit_MQTT_Publish co2_z19 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_CO2);
//-----------------------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("*******Dary Malinovky*******");
    connectWiFi();
    Serial.print("RRSI: ");
    Serial.println(WiFi.RSSI());
    connectMQTT();
    dht1.begin();
    dht2.begin();
    ds18.begin();
    myCo2.begin(9600);

    //  ----------------LCD------------------------

  //LCD setup
  tft.begin(); //start TFT LCD
  tft.setRotation(3); //set TFT LCD rotation
  tft.fillScreen(TFT_BLACK); //fill background

  //header title 
  tft.fillRect(0,0,320,50,TFT_GREEN); //fill rectangle 
  tft.setFreeFont(FSB12); //set font type 
  tft.setTextColor(TFT_BLACK); //set text color
  tft.drawString("**DARY MALINOVKY**", 10, 20); //draw string 
    
  //text strings 
  tft.setTextColor(TFT_WHITE); 
  tft.setFreeFont(FS18); 
  tft.drawString("Temperature:", 10,60);
  tft.drawString("Humidity:", 10,110);
  tft.drawString("Co2:", 10,160);
  
  tft.setTextColor(TFT_GREEN); 
  tft.setFreeFont(FMB24);  
  tft.drawString("C",260,60);
  tft.drawString("%", 215,100);
  tft.drawString("PPM", 200,150);
  tft.drawFastHLine(0,140,320,TFT_GREEN); //draw horizontal line
} 
void loop() {
  unsigned long currentTime = millis();

  if (currentTime - prevTime_T1 > interval) {
    leerdht1();
    prevTime_T1 = currentTime;
    }
  if (currentTime - prevTime_T2 > interval) {
    leerdht2();
    prevTime_T2 = currentTime;
    }
  if (currentTime - prevTime_T3 > interval) {
    leerds18();
    prevTime_T3 = currentTime;
    }
  if (currentTime - prevTime_T4 > interval) {
    leerco2();
    prevTime_T4 = currentTime;
    }
  if (currentTime - prevTime_T5 > interval) {
    Serial.println("отправленные данные MQTT успешно");
    Serial.println("______________________________________");
    prevTime_T5 = currentTime;
    }
  checkWifi();
  reconnect();

}

void leerdht1() {  
    t1 = dht1.readTemperature();
    h1 = dht1.readHumidity();
  
  Serial.print("Temperatura L3T0: ");
  Serial.print(t1);
  Serial.println(" ºC.");
  Serial.print("Humedad L3D0: ");
  Serial.print(h1);
  Serial.println(" %.");
  temp_0_hdt_0.publish(t1);
  hum_0_hdt_0.publish(h1);
  //sprite buffer for temperature
  spr.createSprite(55, 40); //create buffer
  spr.fillSprite(TFT_BLACK); //fill background color of buffer
  spr.setFreeFont(FMB24); //set font type 
  spr.setTextColor(TFT_WHITE); //set text color
  spr.drawNumber(t1, 0, 0); //display number 
  spr.pushSprite(200, 50); //push to LCD 
  spr.deleteSprite(); //clear buffer

  //sprite buffer for humidity 
  spr.createSprite(55, 40);
  spr.fillSprite(TFT_BLACK);
  spr.setFreeFont(FMB24);
  spr.setTextColor(TFT_WHITE);
  spr.drawNumber(h1, 0, 0); //display number 
  spr.pushSprite(155, 100);
  spr.deleteSprite();  
}
void leerdht2() { 
  t2 = dht2.readTemperature();
  h2 = dht2.readHumidity();
  
  Serial.print("Temperatura L3T1: ");
  Serial.print(t2);
  Serial.println(" ºC.");
  Serial.print("Humedad L3H1: ");
  Serial.print(h2);
  Serial.println(" %.");
  temp_1_hdt_1.publish(t2);
  hum_1_hdt_1.publish(h2);  
}
void leerds18() {
  ds18.requestTemperatures();   //envía el comando para obtener las temperaturas
  temp1= ds18.getTempC(address1);
  temp2= ds18.getTempC(address2);
  Serial.print("temp_ds12_0 = ");
  Serial.print(temp1);
  Serial.println(" C");
  ds18_0.publish(temp1);
  Serial.print("temp_ds12_1 = ");
  Serial.print(temp2);
  Serial.println(" C");
  ds18_1.publish(temp2);
}
void leerco2() { 
  myCo2.write(request, 9);
  myCo2.write((byte)0x00);

  memset(response, 0, 9);
  myCo2.readBytes(response, 9);
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) crc+=response[i];
  crc = 255 - crc;
  crc++;
  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
    Serial.println("CRC error");
  } else {
    unsigned int HLconcentration = (unsigned int) response[2];
    unsigned int LLconcentration = (unsigned int) response[3];
    co2 = (256*HLconcentration) + LLconcentration;
    Serial.println(co2);
    for (i = 0; i < 9; i++) {
      Serial.print("0x");
      Serial.print(response[i],HEX);
      Serial.print("  ");
    }
    Serial.println("  ");
   co2_z19.publish(co2); //  Serial.write((byte) 0x00);

  }  
  //sprite buffer for Co2 
  spr.createSprite(55, 40);
  spr.fillSprite(TFT_BLACK);
  spr.setFreeFont(FMB24);
  spr.setTextColor(TFT_WHITE);
  //spr.drawNumber(co2, 0, 0); 
  spr.drawNumber(co2, 0, 0, 4);
  spr.pushSprite(100, 160);
  spr.deleteSprite(); 
} 
void connectWiFi() {
    //WiFi.mode(WIFI_STA);
    //WiFi.disconnect();
    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.setFreeFont(FMB12);
    tft.setCursor((320 - tft.textWidth("Connecting to Wi-Fi.."))/2, 120);
    tft.print("Connecting to Wi-Fi..");
  Serial.println("Connecting to WiFi..");
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempts++;
        if (attempts > 30) {
         NVIC_SystemReset();
      }     
    }
    tft.fillScreen(TFT_BLACK);
    tft.setCursor((320 - tft.textWidth("Connected!"))/2, 120);
    tft.print("Connected..!");
    //Serial.println("Connected to the WiFi network");
    //Serial.print("IP Address: ");
    //Serial.println (WiFi.localIP()); // prints out the device's IP address
  
}
void checkWifi() {
  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("(void checkWifi) Reconnecting to WiFi...");
    WiFi.disconnect();
    NVIC_SystemReset();
    previousMillis = currentMillis;
  }
}
//------------------------------
void connectMQTT() {
  if (mqtt.connected())
    return;
  Serial.print("Connecting to MQTT... ");
  int attempts2 = 0;
  while (mqtt.connect() != 0) {
       delay(500);
       Serial.print(".");
       attempts2++;
       mqtt.disconnect();
       if (attempts2 > 30) {
         NVIC_SystemReset();
      }
  }
  Serial.println("MQTT Connected!");

}
void reconnect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("test alive MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(3000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         NVIC_SystemReset();
       }
  }
  //Serial.println("MQTT Connected!");
}
