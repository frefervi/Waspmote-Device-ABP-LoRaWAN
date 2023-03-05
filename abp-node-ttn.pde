/*  
 *  --------- ABP-based LoRaWAN End-Devide with Waspmote Technology ----------
 *  Device:  Waspmote Plug and Sense
 *  Model: Smart Agriculture PRO
 *  
 *  Explanation: This is the basic code to create frames with all the 
 *  Waspmote Smart Agriculture PRO sensors and send the frame to the 
 *  LoRaWAN network implemented with The Things Network Server, 
 *  using ABP activation.
 *  
 *  Copyright (C) 2016 Libelium Comunicaciones Distribuidas S.L. 
 *  http://www.libelium.com 
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 * 
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 * 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Program version:        1.0
 *  Design and Implementation:  Freddy Villavicencio
 */

// Libraries
#include <WaspLoRaWAN.h>
#include <WaspFrame.h>
#include <WaspSensorAgr_v30.h>

// Socket to use
//////////////////////////////////////////////
uint8_t socket = SOCKET0;
/////////////////////////////////////////////

// Device parameters for Back-End registration (Example)
////////////////////////////////////////////////////////////
char DEVICE_EUI[]  = "70B3D57ED0052F93";
char DEVICE_ADDR[] = "260CA484";
char NWK_SESSION_KEY[] = "1DBFD2BBE953ED1DD814F2BE4B1318B6";
char APP_SESSION_KEY[] = "3433DE019478F7D992014D8E57C14FAD";
////////////////////////////////////////////////////////////

// Define port to use in Back-End: from 1 to 223
uint8_t PORT = 3;

// Variable
uint8_t error;

// Instance sensor object
radiationClass radSensor;
pt1000Class pt1000Sensor;
weatherStationClass weather;
watermarkClass wmSensor2(SOCKET_2);

// Variables to store sensors readings
float temperature;          //Air temperature (°C)
float humdity;              //Relative humidity (%RH)
float pressure,valuePres;   //Air pressure (kPa)
float pt1000Temperature;    //Soil temperature (°C)
float watermark2;           //Soil moisture 2 (cbar)
float radiation,valueRad;   //Solar radiation PAR (umol·m-2·s-1)
float anemometer;           //Wind speed (km/h)
float pluviometer1;         //Current hour accumulated rainfall (mm/h)
float pluviometer2;         //Previous hour accumulated rainfall (mm/h)
float pluviometer3;         //Last 24h accumulated rainfall (mm/day)
uint8_t vane;               //Wind direction

// Variable to store the number of pending pulses
int pendingPulses;


void setup()
{
  // Setup for Serial port over USB
  USB.ON();
  USB.println(F("Start Waspmote Program"));
  // Turn on the sensor board
  Agriculture.ON();
  
  // Start LoRaWAN Configuration
  setupLoRaWAN();   
  // Measure sensors
  measureSensors();
  // Print Data Sensors
  printData();  
  // Create Frame
  generateFrame();
  // Send Frame
  sendLoRaWAN();  
}


void setupLoRaWAN()
{
  USB.println(F("------------------------------------"));
  USB.println(F("Module configuration"));
  USB.println(F("------------------------------------"));  
  //////////////////////////////////////////////
  // 1. Switch on
  //////////////////////////////////////////////
  error = LoRaWAN.ON(socket);
  // Check status
  if( error == 0 ) 
  {
    USB.println(F("1. Switch ON OK"));     
  }
  else 
  {
    USB.print(F("1. Switch ON error = ")); 
    USB.println(error, DEC);
  } 

  //////////////////////////////////////////////
  // 2. Set Device EUI
  //////////////////////////////////////////////
  error = LoRaWAN.setDeviceEUI(DEVICE_EUI);
  // Check status
  if( error == 0 ) 
  {
    USB.println(F("2. Device EUI set OK"));     
  }
  else 
  {
    USB.print(F("2. Device EUI set error = ")); 
    USB.println(error, DEC);
  }

  //////////////////////////////////////////////
  // 3. Set Device Address
  //////////////////////////////////////////////
  error = LoRaWAN.setDeviceAddr(DEVICE_ADDR);
  // Check status
  if( error == 0 ) 
  {
    USB.println(F("3. Device address set OK"));     
  }
  else 
  {
    USB.print(F("3. Device address set error = ")); 
    USB.println(error, DEC);
  }

  //////////////////////////////////////////////
  // 4. Set Network Session Key
  //////////////////////////////////////////////
  error = LoRaWAN.setNwkSessionKey(NWK_SESSION_KEY);
  // Check status
  if( error == 0 ) 
  {
    USB.println(F("4. Network Session Key set OK"));     
  }
  else 
  {
    USB.print(F("4. Network Session Key set error = ")); 
    USB.println(error, DEC);
  }

  //////////////////////////////////////////////
  // 5. Set Application Session Key
  //////////////////////////////////////////////
  error = LoRaWAN.setAppSessionKey(APP_SESSION_KEY);
  // Check status
  if( error == 0 ) 
  {
    USB.println(F("5. Application Session Key set OK"));     
  }
  else 
  {
    USB.print(F("5. Application Session Key set error = ")); 
    USB.println(error, DEC);
  }
  
  //////////////////////////////////////////////
  // 6. For 900MHz US bands with gateways limited 
  // to 8 channels, disable the unavailable channels
  //////////////////////////////////////////////
  for (int ch = 0; ch <= 7; ch++)
  {
    LoRaWAN.setChannelStatus(ch, "off");
  }
  for (int ch = 16; ch <= 64; ch++)
  {
    LoRaWAN.setChannelStatus(ch, "off");
  }
  //Sub-Banda 2 (channels from 8 to 15) for TTN is enabled
  USB.println(F("6. Channels status set OK")); 
    
  //////////////////////////////////////////////
  // 7. Change data rate
  //////////////////////////////////////////////
  /*  LoRaWAN US or AU:
        0: SF = 10, BW = 125 kHz, BitRate =   980 bps
        1: SF =  9, BW = 125 kHz, BitRate =  1760 bps
        2: SF =  8, BW = 125 kHz, BitRate =  3125 bps
        3: SF =  7, BW = 125 kHz, BitRate =  5470 bps
  */
  error = LoRaWAN.setDataRate(3);
  // Check status
  if ( error == 0 )
  {
    USB.println(F("7. Data rate set OK"));
  }
  else
  {
    USB.print(F("7. Data rate set error= "));
    USB.println(error, DEC);
  } 
  
  //////////////////////////////////////////////
  // 8. Save configuration
  //////////////////////////////////////////////
  error = LoRaWAN.saveConfig();
  // Check status
  if( error == 0 ) 
  {
    USB.println(F("8. Save configuration OK"));     
  }
  else 
  {
    USB.print(F("8. Save configuration error = ")); 
    USB.println(error, DEC);
  }
  USB.println(F("------------------------------------"));
  USB.println(F("Module configured"));
  USB.println(F("------------------------------------"));
}


void measureSensors()
{
  USB.println(F("------------------------------------"));
  USB.println(F("Measurement Process"));
  USB.println(F("------------------------------------"));
  // Read the temperature sensor
  temperature = Agriculture.getTemperature();
  // Read the humidity sensor
  humdity = Agriculture.getHumidity();
  // Read the pressure sensor
  valuePres = Agriculture.getPressure();
  pressure = valuePres / 1000;
  // Read the PT1000 sensor (soil temperature)
  pt1000Temperature = pt1000Sensor.readPT1000();
  // Read the watermark 2 sensor 
  watermark2 = wmSensor2.readWatermark();
  // Read the solar radiation sensor
  valueRad = radSensor.readRadiation();
  // Conversion from voltage into umol·m-2·s-1
  radiation = valueRad / 0.0002;
  // Read the anemometer sensor
  anemometer = weather.readAnemometer();
  // Read the pluviometer sensor
  pluviometer1 = weather.readPluviometerCurrent();
  pluviometer2 = weather.readPluviometerHour();
  pluviometer3 = weather.readPluviometerDay();
  // Read the vane sensor
  vane = weather.readVaneDirection();
  USB.println(F("------------------------------------"));  
  USB.println(F("Finished the Measurement Process")); 
  USB.println(F("------------------------------------"));      
}


void printData()
{
  USB.print(F("Temperature: "));
  USB.print(temperature);
  USB.println(F(" ºC"));
  USB.print(F("Humidity: "));
  USB.print(humdity);
  USB.println(F(" %"));  
  USB.print(F("Pressure: "));
  USB.print(pressure);
  USB.println(F(" kPa")); 
  USB.print(F("Radiation: "));
  USB.print(radiation);  
  USB.println(F(" umol·m-2·s-1"));
  USB.print(F("Soil temperature: "));
  USB.print(pt1000Temperature);
  USB.println(F(" ºC"));
  USB.println(F("Watermark 2 - Frequency: "));  
   USB.print(watermark2); 
  USB.println(F(" Hz"));
  USB.print(F("Anemometer: "));
  USB.print(anemometer);
  USB.println(F(" km/h"));
  USB.print(F("Current hour accumulated rainfall (mm/h): "));
  USB.println(pluviometer1);
  USB.print(F("Previous hour accumulated rainfall (mm/h): "));
  USB.println(pluviometer2);
  USB.print(F("Last 24h accumulated rainfall (mm/day): "));
  USB.println(pluviometer3); 

  USB.print(F("Direction: "));
  switch(vane)
  {
    case  SENS_AGR_VANE_N   :  USB.println("N");
                               break;
    case  SENS_AGR_VANE_NNE :  USB.println("NNE");
                               break;
    case  SENS_AGR_VANE_NE  :  USB.println("NE");
                               break;
    case  SENS_AGR_VANE_ENE :  USB.println("ENE");
                               break;
    case  SENS_AGR_VANE_E   :  USB.println("E");
                               break;
    case  SENS_AGR_VANE_ESE :  USB.println("ESE");
                               break;
    case  SENS_AGR_VANE_SE  :  USB.println("SE");
                               break;
    case  SENS_AGR_VANE_SSE :  USB.println("SSE");
                               break;
    case  SENS_AGR_VANE_S   :  USB.println("S");
                               break;
    case  SENS_AGR_VANE_SSW :  USB.println("SSW");
                               break;
    case  SENS_AGR_VANE_SW  :  USB.println("SW");
                               break;
    case  SENS_AGR_VANE_WSW :  USB.println("WSW");
                               break;
    case  SENS_AGR_VANE_W   :  USB.println("W");
                               break;
    case  SENS_AGR_VANE_WNW :  USB.println("WNW");
                               break;
    case  SENS_AGR_VANE_NW  :  USB.println("WN");
                               break;
    case  SENS_AGR_VANE_NNW :  USB.println("NNW");
                               break;
  }
}


void generateFrame()
{  
  // Creating a new frame
  USB.println(F("------------------------------------"));  
  USB.println(F("Creating a BINARY frame"));
  USB.println(F("------------------------------------"));  
  // Create new frame (BINARY)
  frame.createFrame(BINARY); 
  // Add battery level
  frame.addSensor(SENSOR_BAT, PWR.getBatteryLevel());
  // Add temperature
  frame.addSensor(SENSOR_AGR_TC, temperature);
  // Add humidity
  frame.addSensor(SENSOR_AGR_HUM, humdity);
  // Add pressure
  frame.addSensor(SENSOR_AGR_PRES, pressure);
  // Add radiation
  frame.addSensor(SENSOR_AGR_PAR, radiation); 
  // Add soil temperature
  frame.addSensor(SENSOR_AGR_SOILTC, pt1000Temperature);
  // Add watermark 2
  frame.addSensor(SENSOR_AGR_SOIL2, watermark2); 
  // Add anemometer
  frame.addSensor(SENSOR_AGR_ANE, anemometer);  
  // Add pluviometer 1
  frame.addSensor(SENSOR_AGR_PLV1, pluviometer1);
  // Add pluviometer 2
  frame.addSensor(SENSOR_AGR_PLV2, pluviometer2);
  // Add pluviometer 3
  frame.addSensor(SENSOR_AGR_PLV3, pluviometer3);     
  // Add vane value
  frame.addSensor(SENSOR_AGR_WV, vane);
  USB.println(F("------------------------------------"));  
  USB.println(F("Finished the Creating a BINARY Frame\n"));    
  USB.println(F("------------------------------------"));   
}


void sendLoRaWAN()
{  
  USB.println(F("------------------------------------"));  
  USB.println(F("Sending Data")); 
  USB.println(F("------------------------------------")); 
  //////////////////////////////////////////////
  // 1. Switch on
  //////////////////////////////////////////////
  error = LoRaWAN.ON(socket);
  // Check status
  if( error == 0 ) 
  {
    USB.println(F("1. Switch ON OK"));     
  }
  else 
  {
    USB.print(F("1. Switch ON error = ")); 
    USB.println(error, DEC);
  }

  //////////////////////////////////////////////
  // 2. Join network
  //////////////////////////////////////////////
  error = LoRaWAN.joinABP();
  // Check status
  if ( error == 0 )
  {
    USB.println(F("2. Join network OK"));

    error = LoRaWAN.getMaxPayload();

    if (error == 0)
    {
      //////////////////////////////////////////////
      // 3. Generate tiny frame
      //////////////////////////////////////////////
      USB.print(F("3.1. LoRaWAN maximum payload OK\n"));
      //USB.println(LoRaWAN._maxPayload, DEC);

      // Set maximum payload
      frame.setTinyLength(LoRaWAN._maxPayload);

      boolean end = false;
      uint8_t pending_fields = 0;

      while (end == false)
      {
        pending_fields = frame.generateTinyFrame();

        USB.print(F("3.2. Tiny frame generated OK\n"));
        //USB.printHexln(frame.bufferTiny, frame.lengthTiny);

        //////////////////////////////////////////////
        // 4. Send unconfirmed packet
        //////////////////////////////////////////////
        USB.println(F("4. LoRaWAN confirmed sending..."));
        error = LoRaWAN.sendUnconfirmed( PORT, frame.bufferTiny, frame.lengthTiny);
        // Check status
        if (error == 0)
        {
          USB.println(F("4.1. LoRaWAN send packet OK"));
          if (LoRaWAN._dataReceived == true)
          {
            USB.print(F("  There's data on port number: "));
            USB.print(LoRaWAN._port, DEC);
            USB.print(F("\r\n  Data: "));
            USB.println(LoRaWAN._data);
          }
        }
        else
        {
          USB.print(F("4.1. LoRaWAN send packet error = "));
          USB.println(error, DEC);
        }

        if (pending_fields > 0)
        {
          end = false;
          delay(10000);
        }
        else
        {
          end = true;
        }
      }
    }
    else
    {
      USB.println(F("3. LoRaWAN error getting the maximum payload"));
    }
  }
  else
  {
    USB.print(F("2. Join network error = "));
    USB.println(error, DEC);
  }

  //////////////////////////////////////////////
  // 5. Switch off
  //////////////////////////////////////////////
  error = LoRaWAN.OFF(socket);
  // Check status
  if( error == 0 ) 
  {
    USB.println(F("5. Switch OFF OK"));     
  }
  else 
  {
    USB.print(F("5. Switch OFF error = ")); 
    USB.println(error, DEC);
  }
  USB.println(F("------------------------------------"));  
  USB.println(F("Finished the Sending Data"));
  USB.println(F("------------------------------------"));  
}


void loop()
{
  /////////////////////////////////////////////
  // 1. Enter sleep mode
  ///////////////////////////////////////////// 
  USB.println(F("------------------------------------"));
  USB.println(F("Hibernate Mode"));  
  USB.println(F("------------------------------------"));
  Agriculture.sleepAgr("00:00:30:00", RTC_OFFSET, RTC_ALM1_MODE4, SENSOR_ON, SENS_AGR_PLUVIOMETER);  
  
  /////////////////////////////////////////////
  // 2.1. Check pluviometer interruption
  /////////////////////////////////////////////
  if( intFlag & PLV_INT)
  {
    USB.println(F("+++ PLV interruption +++"));

    pendingPulses = intArray[PLV_POS];

    USB.print(F("Number of pending pulses: "));
    USB.println(pendingPulses);

    for(int i=0 ; i<pendingPulses; i++)
    {
      // Enter pulse information inside class structure
      weather.storePulse();
      
      // Decrease number of pulses
      intArray[PLV_POS]--;
    }
    
    // Clear flag
    intFlag &= ~(PLV_INT);    
  }
  
  /////////////////////////////////////////////
  // 2.2. Check RTC interruption
  /////////////////////////////////////////////
  if(intFlag & RTC_INT)
  {
    USB.println(F("+++ RTC interruption +++"));
    
    // Turn on sensor board
    Agriculture.ON();
    
    // Init RTC    
    RTC.ON();
    USB.print(F("Time: "));
    USB.println(RTC.getTime());
           
    // Measure sensors
    measureSensors();
    // Print Data Sensors
    printData();
    // Create Frame
    generateFrame();
    // Send Frame
    sendLoRaWAN();

    // Clear flag
    intFlag &= ~(RTC_INT); 
  }
  USB.println(F("------------------------------------"));
  USB.println(F("Finished the Hibernate Mode"));  
  USB.println(F("------------------------------------"));
  delay(5000);
}
