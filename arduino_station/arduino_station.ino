/*
 * 
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK  - pin 13
 ** CS    - pin 4 (for MKRZero SD: SDCARD_SS_PIN)
 * 
 * 
 */
 
 
 
 /*
   BME and/or
  MPU -> Nano
  
  SDA -> A4
  SCL -> A5
  GND -> GND
  VCC -> 5V
  */
  
  
// Attach CC1101 pins to their corresponding SPI pins
// Uno pins:
// CSN (SS) => 10
// MOSI => 11
// MISO => 12
// SCK => 13
// GD0 => A valid interrupt pin for your platform (defined below this)

/*
 * ARDUINO /  CC-1101  /    ARDUINO
 *             _____
 *       VDD -|1   2|- VDD  <- Vcc 3v3
 * 11 -> SI  -|3   4|- SCK  <- 13
 * 12 <- SO  -|5   6|- GDO2
 * 10 -> CSn -|7   8|- GDO0 <-> 2 (ISR+?)
 *       GND -|9  10|- GND  -> GND
 *             ¯¯¯¯¯
 */
  
  
#define DB_FILE_BASE_NAME "data"
#define DB_FILE_SEPARATOR "_"
#define DB_FILE_EXTENSION ".db"

//for sd card
#include <SPI.h>
#include <SD.h>
const int chipSelect = 4;

//for bme
#define SEALEVELPRESSURE_HPA (1013.25)
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme; // I2C

//for cc1101
/*
#include <Arduino.h>
#include "cc1101.h"
#include "ccpacket.h"
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define CC1101Interrupt 4 // Pin 19
#define CC1101_GDO0 19
#elif defined(__MK64FX512__)
// Teensy 3.5
#define CC1101Interrupt 9 // Pin 9
#define CC1101_GDO0 9
#else
#define CC1101Interrupt 0 // Pin 2
#define CC1101_GDO0 2
#endif

#define LEDOUTPUT 7
CC1101 radio;

byte rcvAddr = 0x22;
byte sndAddr = 0x00;//0x00 : broadcast
byte channel = 0x00;
bool packetWaiting;
*/

int db_file_i=0;

void setup(){
  Serial.begin(9600);
  // see if the card is present and can be initialized:
  Serial.print(F("Card init:"));
  if (!SD.begin(chipSelect)) {
    Serial.println(F(" fail"));
  }else{
    Serial.println(F(" ok"));
  }
  Serial.print(F("BME init:"));
  if (!bme.begin()) {
    Serial.println(F(" fail"));
  }else{
    //configure sensor
    bme.setSampling(Adafruit_BME280::MODE_FORCED, // needs to call bme.takeForcedMeasurement(); before reading
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
    Serial.println(F(" ok"));
  }
  
  
  
  //radio init
  /*
  radio.init(CFREQ_433, 0);//CFREQ_433;//868-915-433-918, 0 or MODE_LOW_SPEED (38 or 4.8kbps)
  radio.setSyncWord({199, 10});
  radio.setDevAddress(rcvAddr);
  radio.setChannel(channel);//or radio.disableAddressCheck();
  //radio.setCarrierFreq(freq);
  radio.setTxPowerAmp(PA_LowPower);//PA_LowPower-PA_LongDistance

  Serial.print(F("CC1101_PARTNUM "));
  Serial.println(radio.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
  Serial.print(F("CC1101_VERSION "));
  Serial.println(radio.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));
  Serial.print(F("CC1101_MARCSTATE "));
  Serial.println(radio.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f);
  */

 // setup the blinker output
 /*
 pinMode(LEDOUTPUT, OUTPUT);
 digitalWrite(LEDOUTPUT, LOW);

 // blink once to signal the setup
 blinker();
 

  Serial.println(F("CC1101 radio initialized."));
  attachInterrupt(CC1101Interrupt, messageReceived, FALLING);
  */
}
/*
void blinker(){
     digitalWrite(LEDOUTPUT, HIGH);
     delay(100);
     digitalWrite(LEDOUTPUT, LOW);
     delay(100);
}*/

/*
void messageReceived(){
    packetWaiting = true;
}*/
void loop(){
  serialReceive();
  /*
  if (packetWaiting) {
    receive();
  }
  */
}

/*
void receive(){
  detachInterrupt(CC1101Interrupt);
  packetWaiting = false;
  CCPACKET packet;
  if (radio.receiveData(&packet) > 0) {
      if (!packet.crc_ok) {
          Serial.print(F(" crc NOT ok"));
      }
      //if (packet.crc_ok && packet.length > 0) {
      if (packet.length > 0) {
          //respond to ping
          char message[59];
          strncpy(message,packet.data+1, 25);
          strcpy(message+strlen(message)," dst:0x");
          String(packet.data[0],HEX).toCharArray(message+strlen(message),5);
          strcpy(message+strlen(message)," len:");
          itoa(packet.length,message+strlen(message),10);
          strcpy(message+strlen(message)," lqi:");
          itoa(lqi(packet.lqi),message+strlen(message),10);
          strcpy(message+strlen(message)," rssi:");
          itoa(rssi(packet.rssi),message+strlen(message),10);
          strcpy(message+strlen(message),"dBm SND:PONG_");
          //itoa(incr,message+strlen(message),10);incr++;
          
          Serial.print("RCV:");
          Serial.print(message);
          
          CCPACKET packet;
          // We also need to include the 0 byte at the end of the string and addr
          packet.length = strlen(message)  + 2;
          packet.data[0] = sndAddr;
          strncpy((char *) packet.data+1, message, packet.length);
  
          if(radio.sendData(packet)){
            Serial.print(F(" SUCCESS responding !"));
          }else{
            Serial.print(F(" ERROR during response..."));
          }
      }else{

        
      }
      Serial.println("");
  }
  attachInterrupt(CC1101Interrupt, messageReceived, FALLING);
}*/
void readBME(){
  double temp, pres, alt, humid;
  bme.takeForcedMeasurement();
  temp = bme.readTemperature();
  pres = bme.readPressure() / 100.0F;
  alt  = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humid= bme.readHumidity();
  Serial.print(temp);
  Serial.print(F(" "));
  Serial.print(pres);
  Serial.print(F(" "));
  Serial.print(alt);
  Serial.print(F(" "));
  Serial.println(humid);
}
void readfromSD(){
  File myFile = SD.open("data.txt");
  if (myFile) {
    Serial.println(F("file:"));
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
  } else {
    Serial.println(F("file error"));
  }
}
void writeOnSd(){
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("test");
    dataFile.close();
    Serial.println(F("data added !"));
  }else{
    Serial.println(F("error"));
  }
  /*
  char test[100];
  sprintf(test, "Year: %u, Blog Link: %s",2,"asdf");*/
}
//todo read : https://arduinobasics.blogspot.com/2019/05/sprintf-function.html
void serialReceive(){
  if(Serial.available()){
    String input = Serial.readStringUntil("\n");
    //switch(getValue(input,' ',0)[0]){
    switch(input[0]){
      case 'c':
        Serial.println(F("[c]onfigure RTC TODO"));
        readfromSD();
        break;
      case 't':
        Serial.println(F("[t]ime :"));
        Serial.print(DB_FILE_BASE_NAME);
        Serial.print(DB_FILE_SEPARATOR);
        Serial.print(db_file_i);
        Serial.println(DB_FILE_EXTENSION);
        break;
      case 'f':
        Serial.println(F("[f]requency :"));
        break;
      case 'l':
        Serial.println(F("[l]ist files :"));
        break;
      case 'e':
        Serial.println(F("[e]xtract data :"));
        writeOnSd();
        break;
      case 'b':
        Serial.println(F("[b]egin new file :"));

        break;
      case 'd':
        Serial.println(F("[d]elete file :"));
        break;
      case 's':
        Serial.println(F("[s]how BME :"));
        readBME();
        break;
      case 'h':
      /*
        Serial.println("[H]elp : the following commands are available :");
        Serial.println("[c]onfigure RTC with format dd.mm.yyy hh:mm");
        Serial.println("[t]ime, show current RTC");
        Serial.println("[f]requency change between samples, format hh:mm");
        Serial.println("[l]ist file on sd card");
        Serial.println("[e]xtract temp data from file");
        Serial.println("[b]egin a new file");
        Serial.println("[d]elete file from sd card");
        Serial.println("[s]how current BME280 state");
        Serial.println("[h]elp, shows this text");
        */
        break;
      default:
        Serial.println(F("Unknown cmd, h for help"));
        break;
    }
  }
}
/*
//https://arduino.stackexchange.com/questions/1013/how-do-i-split-an-incoming-string
String getValue(String data, char separator, int index){
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}*/
