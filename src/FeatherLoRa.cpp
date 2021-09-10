//  ███████╗███████╗ █████╗ ████████╗██╗  ██╗███████╗██████╗     ██╗      ██████╗ ██████╗  █████╗ 
//  ██╔════╝██╔════╝██╔══██╗╚══██╔══╝██║  ██║██╔════╝██╔══██╗    ██║     ██╔═══██╗██╔══██╗██╔══██╗
//  █████╗  █████╗  ███████║   ██║   ███████║█████╗  ██████╔╝    ██║     ██║   ██║██████╔╝███████║
//  ██╔══╝  ██╔══╝  ██╔══██║   ██║   ██╔══██║██╔══╝  ██╔══██╗    ██║     ██║   ██║██╔══██╗██╔══██║
//  ██║     ███████╗██║  ██║   ██║   ██║  ██║███████╗██║  ██║    ███████╗╚██████╔╝██║  ██║██║  ██║
//  ╚═╝     ╚══════╝╚═╝  ╚═╝   ╚═╝   ╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝    ╚══════╝ ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝

#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>

//this will determine which version of the LoRa code to upload to an arduino
//0 is RX, 1 is TX
#define BOARD_SELECT 1

// for feather m0 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match TX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


//wb tx initializations
#if(BOARD_SELECT == 1)
  
  #include <MS5607.h>
  #include <Adafruit_GPS.h>
  #include <Adafruit_Sensor.h>
  #include <DHT.h>
  #include <DHT_U.h>
  #include <Adafruit_MAX31856.h>
  #include <Adafruit_FXOS8700.h>


  #define GPSSerial Serial1
  #define GPSECHO false
  //define pins
  #define PINS 99
  #define DHTPIN 2


  //declare sensors
  MS5607 P_SENS;
  Adafruit_GPS GPS(&GPSSerial);
  #define DHTTYPE  22
  DHT_Unified dht(DHTPIN, DHTTYPE);
  Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10, 11, 12, 13);
  Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

  //declare vars

  uint32_t timer = millis();

  float phase;
  
  float temp;
  float pressure;

#endif



void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);
  delay(5000);

  Serial.println("LoRa System Initializing");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  #if(BOARD_SELECT == 1)
    phase = 0;

    if(!P_SENS.begin()){
      Serial.println("MS5607 failed!");
    }else Serial.println("MS5607 initialise successfully!");

    GPS.begin(9600);

    dht.begin();
    sensor_t sensor;

    maxthermo.begin();

    maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);

    if(!accelmag.begin(ACCEL_RANGE_4G))
    {
      /* There was a problem detecting the FXOS8700 ... check your connections */
      Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
      while(1);
    }

  #endif

}

float packetnum = 0;

void loop(){
  #if(BOARD_SELECT==0)
    if (rf95.available()){
      // Should be a message for us now
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if (rf95.recv(buf, &len))
      {
        float* rbuf = (float*)buf;

        Serial.print("Got: ");
        Serial.println("Packet #: " +(String)rbuf[1]);
        Serial.println("Seconds: " +(String)rbuf[2]);
        Serial.println("Phase: " + (String)rbuf[3]);
        Serial.println("Pressure: " +(String)rbuf[4] + " mbar");
        Serial.println("Temperature: " +(String)rbuf[5]+" C");
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);
        

      }
      else
      {
        Serial.println("Receive failed");
      }
    }
  #elif(BOARD_SELECT == 1)   
    delay(1000); // Wait 1 second between transmits, could also 'sleep' here!

    P_SENS.getDigitalValue();
    pressure = (float)P_SENS.getPressure();
    temp = (float)P_SENS.getTemperature();

    Serial.println("Pressure    - "+String(pressure)+" mbar");
    Serial.println("Temperature - "+String(temp)+" C");

    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences!
      // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
      Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return; // we can fail to parse a sentence in which case we should just wait for another
    }
    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis()) timer = millis();
      
    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000) {
      timer = millis(); // reset the timer
      Serial.print("\nTime: ");
      Serial.print(GPS.hour, DEC); Serial.print(':');
      Serial.print(GPS.minute, DEC); Serial.print(':');
      Serial.print(GPS.seconds, DEC); Serial.print('.');
      Serial.println(GPS.milliseconds);
      Serial.print("Date: ");
      Serial.print(GPS.day, DEC); Serial.print('/');
      Serial.print(GPS.month, DEC); Serial.print("/20");
      Serial.println(GPS.year, DEC);
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
      if (GPS.fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      }
    }


    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    }
    else {
      Serial.print(F("Temperature: "));
      Serial.print(event.temperature);
      Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    }
    else {
      Serial.print(F("Humidity: "));
      Serial.print(event.relative_humidity);
      Serial.println(F("%"));
    }

    Serial.print("Cold Junction Temp: "); 
    Serial.println(maxthermo.readCJTemperature());

    Serial.print("Thermocouple Temp: "); 
    Serial.println(maxthermo.readThermocoupleTemperature());
    // Check and print any faults
    uint8_t fault = maxthermo.readFault();
    if (fault) {
      if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
      if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
      if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
      if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
      if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
      if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
      if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
      if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
    }

    sensors_event_t aevent, mevent;

    /* Get a new sensor event */
    accelmag.getEvent(&aevent, &mevent);

    /* Display the accel results (acceleration is measured in m/s^2) */
    Serial.print("A ");
    Serial.print("X: "); Serial.print(aevent.acceleration.x, 4); Serial.print("  ");
    Serial.print("Y: "); Serial.print(aevent.acceleration.y, 4); Serial.print("  ");
    Serial.print("Z: "); Serial.print(aevent.acceleration.z, 4); Serial.print("  ");
    Serial.println("m/s^2");

    /* Display the mag results (mag data is in uTesla) */
    Serial.print("M ");
    Serial.print("X: "); Serial.print(mevent.magnetic.x, 1); Serial.print("  ");
    Serial.print("Y: "); Serial.print(mevent.magnetic.y, 1); Serial.print("  ");
    Serial.print("Z: "); Serial.print(mevent.magnetic.z, 1); Serial.print("  ");
    Serial.println("uT");

    float radiopacket[20] = {1,packetnum++,(float)millis()/1000,phase,pressure,temp};

    Serial.println("Sending...");
    delay(10);
    rf95.send((uint8_t *)radiopacket, sizeof(radiopacket)*4);
  #endif
}