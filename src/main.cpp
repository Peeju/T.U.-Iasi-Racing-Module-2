#include <Arduino.h>
#include "ArduinoLog.h"
#include "adcObj/adcObj.hpp"
#include "driver/can.h"
#include <TinyGPSPlus.h>
#include "Aditional/aditional.hpp"
#include <HardwareSerial.h>


#define TX_GPIO_NUM   GPIO_NUM_16 //inversate??
#define RX_GPIO_NUM   GPIO_NUM_17
#define LOG_LEVEL LOG_LEVEL_VERBOSE
#define GPS_BAUDRATE 9600 
#define SPIN0 GPIO_NUM_33 //a0
#define SPIN1 GPIO_NUM_12//a1
#define SPIN2 GPIO_NUM_13 //a2
HardwareSerial GPS_Serial(1);

TinyGPSPlus gps;
u_int16_t status;
adcObj BSPD(ADC1_CHANNEL_4);
adcObj brakePressure(ADC1_CHANNEL_4);
adcObj steeringAngle(ADC1_CHANNEL_4);
adcObj oilPressure(ADC1_CHANNEL_4);

//todo: ADC: BSPD, brake pressure, steering angle, oil pressure
//todo: GPS: lat, long, speed

static const can_general_config_t g_config = {.mode = TWAI_MODE_NO_ACK, .tx_io = TX_GPIO_NUM, .rx_io = RX_GPIO_NUM,        
                                                                    .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED,      
                                                                    .tx_queue_len = 1, .rx_queue_len = 5,                           
                                                                    .alerts_enabled = TWAI_ALERT_ALL,  .clkout_divider = 0,        
                                                                    .intr_flags = ESP_INTR_FLAG_LEVEL1};
static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();


void setup() {
  Serial.begin(9600);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17 (adjust if needed)

  pinMode(SPIN1, INPUT);
  pinMode(SPIN2, INPUT);
  pinMode(SPIN0, INPUT); 

  Log.begin(LOG_LEVEL, &Serial);

  status = can_driver_install(&g_config, &t_config, &f_config);
    if(status==ESP_OK){
      Log.noticeln("Can driver installed");
    }
    else {
      Log.errorln("Can driver installation failed with error: %s", esp_err_to_name(status));
    }

    status=can_start();
    if(status==ESP_OK){
      Log.noticeln("Can started");
    }
    else {
      Log.errorln("Can starting procedure failed with error: %s", esp_err_to_name(status));
    }
}

void loop() {

  twai_message_t tx_msg_adc;
  tx_msg_adc.data_length_code=8;
  tx_msg_adc.identifier=0x115;

  
  twai_message_t tx_msg_gps;
  tx_msg_gps.data_length_code=8;
  tx_msg_gps.identifier=0x116;
  /*
  lat=46.52566262797154
  lng=26.9430779276816
  */
  double lat=0;
  double lng=0;
  double spd = 150;
  

  int sb2 = !digitalRead(SPIN2);
  int sb1 = !digitalRead(SPIN1);
  int sb0 = !digitalRead(SPIN0);
  //Serial.print("   ");
  //Serial.print(sb2);
  //Serial.print("   ");
  //Serial.print(sb1);
  //Serial.print("   ");
  //Serial.println(sb0);
  int gear = sb0 * 4 + sb1 * 2 + sb2;
  tx_msg_gps.data[7] = gear;
  
  //Serial.print("Gear: ");
  //Serial.print(gear);
  //Serial.println();
  

   #if LOG_LEVEL==LOG_LEVEL_VERBOSE
   //Serial.println("OK");
  //  while(!Serial2.available()){}
  // if(Serial2.available() > 0){
  //   Serial.print("OK");
  //   if (gps.encode(Serial2.read())){ 
  //     Serial.print("OK");
  //     if (gps.location.isValid()) {
  //       lat=gps.location.lat();
  //       lng=gps.location.lng();
  //       Serial.println(lat);
  //     }
  //     else {
  //       Log.errorln("GPS position is INVALID");
  //     }
  //     if(gps.speed.isValid())
  //       spd=gps.speed.kmph(); 
  //     else Log.errorln("GPS speed is INVALID");
  //   }
  //   else Log.errorln("Data from GPS is invalid");
  // }
  //else Log.errorln("GPS serial is not available");

 //Log.verboseln("Latitude: %.2D, Longitude: %2.D, Speed: %2.D km/h");
   //Log.verboseln("Lat:%D, Long:%D, Spd:%D");

   while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }
  lat = gps.location.lat();
  lng = gps.location.lng();
  spd = gps.speed.kmph();

  
   Serial.print("Lat= ");
   Serial.print(lat,9);
   Serial.print("; Long= ");
   Serial.println(lng,9);
  // Serial.println(gear);
   Serial.println();
  convert(fractional(lat), tx_msg_gps.data);
  convert(fractional(lng), tx_msg_gps.data+3);
  tx_msg_gps.data[6]=uint8_t(spd);


  int d1 = BSPD.getVoltage();
  int d2 = brakePressure.getVoltage();
  int d3 = steeringAngle.getVoltage();
  int d4 = oilPressure.getVoltage();
  //Log.verboseln("BSPD: %d, Brake pressure: %d, Steering Angle: %d, Oil Pressure: %d", d1, d2, d3, d4);

 
  
  tx_msg_adc.data[0]=d1/100;
  tx_msg_adc.data[1]=d1%100;
  tx_msg_adc.data[2]=d2/100;
  tx_msg_adc.data[3]=d2%100;
  tx_msg_adc.data[4]=d3/100;
  tx_msg_adc.data[5]=d3%100;
  tx_msg_adc.data[6]=d4/100;
  tx_msg_adc.data[7]=d4%100;


  #else
  if(Serial2.available() > 0){
    if (gps.encode(Serial2.read())){ 
      if (gps.location.isValid()) {
        convert(fractional(gps.location.lat()), tx_msg_gps.data);
        convert(fractional(gps.location.lng()), tx_msg_gps.data+3);
      }
      else {
        Log.errorln("GPS position is INVALID");
        tx_msg_gps.data[0]=0xff;
        tx_msg_gps.data[1]=0xff;
        tx_msg_gps.data[2]=0xff;
        tx_msg_gps.data[3]=0xff;
        tx_msg_gps.data[4]=0xff;
        tx_msg_gps.data[5]=0xff;
        
      }
      if(gps.speed.isValid())
        tx_msg_gps.data[6]=gps.speed.kmph(); 
      else {
        Log.errorln("GPS speed is INVALID");
        tx_msg_gps.data[6]=0xff;
    }
    }
    else {
      Log.errorln("Data from GPS is invalid");
      tx_msg_gps.data[0]=0xee;
      tx_msg_gps.data[1]=0xee;
      tx_msg_gps.data[2]=0xee;
      tx_msg_gps.data[3]=0xee;
      tx_msg_gps.data[4]=0xee;
      tx_msg_gps.data[5]=0xee;
      tx_msg_gps.data[6]=0xee;
      }
  }
  else {
    Log.errorln("GPS serial is not available");
      tx_msg_gps.data[0]=0xdd;
      tx_msg_gps.data[1]=0xdd;
      tx_msg_gps.data[2]=0xdd;
      tx_msg_gps.data[3]=0xdd;
      tx_msg_gps.data[4]=0xdd;
      tx_msg_gps.data[5]=0xdd;
      tx_msg_gps.data[6]=0xdd;
    }

  tx_msg_adc.data[0]=BSPD.getVoltage()/100;
  tx_msg_adc.data[1]=BSPD.getVoltage()%100;
  tx_msg_adc.data[2]=brakePressure.getVoltage()/100;
  tx_msg_adc.data[3]=brakePressure.getVoltage()%100;
  tx_msg_adc.data[4]=steeringAngle.getVoltage()/100;
  tx_msg_adc.data[5]=steeringAngle.getVoltage()%100;
  tx_msg_adc.data[6]=oilPressure.getVoltage()/100;
  tx_msg_adc.data[7]=oilPressure.getVoltage()%100;
  #endif


    status = can_transmit(&tx_msg_adc, pdMS_TO_TICKS(1000));
  if(status==ESP_OK) {
    //Log.noticeln("Can message sent");
  }
  else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if(status==ESP_OK) Log.errorln("Can driver restarted");
  }
  
  
    status = can_transmit(&tx_msg_gps, pdMS_TO_TICKS(1000));
  if(status==ESP_OK) {
    //Log.noticeln("Can message sent");
  }
  else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if(status==ESP_OK) Log.errorln("Can driver restarted");
  }

}