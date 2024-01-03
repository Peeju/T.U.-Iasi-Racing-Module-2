#include <Arduino.h>
#include "ArduinoLog.h"
#include "adcObj/adcObj.hpp"
#include "driver/can.h"
#include <TinyGPSPlus.h>
#include "Aditional/aditional.hpp"


#define TX_GPIO_NUM   GPIO_NUM_14
#define RX_GPIO_NUM   GPIO_NUM_27
#define LOG_LEVEL LOG_LEVEL_NOTICE
#define GPS_BAUDRATE 9600 

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
  Serial2.begin(GPS_BAUDRATE);


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
  tx_msg_gps.data_length_code=7;
  tx_msg_gps.identifier=0x116;
  //lat=46.52566262797154
  //lng=26.9430779276816
  double lat=46.52566262797154;
  double lng=26.9430779276816;
  double spd = 150;

   #if LOG_LEVEL==LOG_LEVEL_VERBOSE
  // if(Serial2.available() > 0){
  //   if (gps.encode(Serial2.read())){ 
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
  // else Log.errorln("GPS serial is not available");

 //Log.verboseln("Latitude: %.2D, Longitude: %2.D, Speed: %2.D km/h");
   //Log.verboseln("Lat:%D, Long:%D, Spd:%D");
  Serial.print("Lat= ");
  Serial.print(lat);
  Serial.print("; Long= ");
  Serial.println(lng);
  convert(fractional(lat), tx_msg_gps.data);
  convert(fractional(lng), tx_msg_gps.data+3);
  tx_msg_gps.data[6]=uint8_t(spd);


  int d1 = BSPD.getVoltage();
  int d2 = brakePressure.getVoltage();
  int d3 = steeringAngle.getVoltage();
  int d4 = oilPressure.getVoltage();
  Log.verboseln("BSPD: %d, Brake pressure: %d, Steering Angle: %d, Oil Pressure: %d", d1, d2, d3, d4);

 
  
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
    Log.noticeln("Can message sent");
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
    Log.noticeln("Can message sent");
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