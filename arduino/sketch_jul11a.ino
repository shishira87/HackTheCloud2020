#include <WiFi.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <Arduino_JSON.h>
#include<SoftwareSerial.h>

#define CONTROL_WIRE 10
#define OUTPUT_WIRE 5 // ncp
#define INPUT_WIRE 4 //ncp
#define RX 14
#define TX 12

#define SSID_ *********
#define PW_ **********
#define SERVER_NAME https://ancient-coast-29473.herokuapp.com/getCoords  

uint64_t time_delay = 5000;
uint64_t prev_time = 0;

char* coordinates;
int coordinates_arr[3];

SoftwareSerial braincomm(RX,TX);

void setup() {
  
Serial.begin(115200);
pinMode(LED_BUILTIN, OUTPUT);
pinMode(OUTPUT_WIRE,OUTPUT);
pinMode(CONTROL_WIRE,OUTPUT);
pinMode(INPUT_WIRE,INPUT);
braincomm.begin(9600);

WiFi.begin(SSID_, PW_);
  
  while(WiFi.status() != WL_CONNECTED){}
  
  Serial.print("\nConnected\nIP Address:");
  Serial.println(WiFi.localIP());
 

}

void loop() {
  
if(millis()-prev_time > time_delay){

  if(WiFi.status()== WL_CONNECTED){

    coordinates = httpGETRequest(SERVER_NAME);
    JSONVar reading = JSON.parse(coordinates);

    if(JSON.typeof(reading)== "undefined"){
      return;
    }

    JSONVar keys = reading.keys();
    
      for (int i = 0; i < keys.length(); i++) {
        JSONVar value = reading[keys[i]];
        Serial.print(keys[i]);
        Serial.print(" = ");
        Serial.println(value);
        coordinates_arr[i] = double(value);
      }
      Serial.print("x = ");
      Serial.println(coordinates_arr[0]);
      braincomm.write(coordinate_arr[0]);
      Serial.print("y = ");
      Serial.println(coordinates_arr[1]);
      braincomm.write(coordinate_arr[1]);
      Serial.print("z = ");
      Serial.println(coordinates_arr[2]);
      braincomm.write(coordinate_arr[2]);
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}
  }
}

String httpGETRequest(const char* serverName) {
  HTTPClient http;
  
  http.begin(serverName);
  
  
  int httpResponseCode = http.GET();
  
  String payload = "{}"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();

  return payload;
}
  
  

     
  
