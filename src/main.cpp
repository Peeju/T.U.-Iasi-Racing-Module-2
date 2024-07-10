#include <Arduino.h>

#include "ArduinoLog.h"
#include "adcObj/adcObj.hpp"
#include "driver/can.h"
#include <TinyGPS++.h>
#include "Aditional/aditional.hpp"
#include <HardwareSerial.h>


#define TX_GPIO_NUM   GPIO_NUM_14 //inversate??
#define RX_GPIO_NUM   GPIO_NUM_27
#define LOG_LEVEL LOG_LEVEL_VERBOSE
#define GPS_BAUDRATE 9600 
// #define SPIN0 GPIO_NUM_33 //a0
// #define SPIN1 GPIO_NUM_12//a1
// #define SPIN2 GPIO_NUM_2 //a2 

u_int16_t status;
//adcObj BSPD(ADC1_CHANNEL_5);
adcObj brakePressure(ADC1_CHANNEL_4);
adcObj steeringAngle(ADC1_CHANNEL_7);
adcObj oilPressure(ADC1_CHANNEL_4);


double latitude=0;
double longitude=0;
double speed=0;

TinyGPSPlus gps;

twai_message_t tx_msg_gps;
twai_message_t tx_msg_adc;

//todo: ADC: BSPD, brake pressure, steering angle, oil pressure
//todo: GPS: lat, long, speed

static const can_general_config_t g_config = {.mode = TWAI_MODE_NO_ACK, .tx_io = TX_GPIO_NUM, .rx_io = RX_GPIO_NUM,        
                                                                    .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED,      
                                                                    .tx_queue_len = 1000, .rx_queue_len = 5,                           
                                                                    .alerts_enabled = TWAI_ALERT_ALL,  .clkout_divider = 0,        
                                                                    .intr_flags = ESP_INTR_FLAG_LEVEL1};
static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

void setup() {
  delay(2000);
  Serial.begin(9600);
  Serial2.begin(GPS_BAUDRATE);
  // pinMode(SPIN1, INPUT);
  // pinMode(SPIN2, INPUT);
  // pinMode(SPIN0, INPUT); 
  pinMode(2, INPUT_PULLUP);//1
  pinMode(12, INPUT_PULLUP);//2
  pinMode(4, INPUT_PULLUP);//3
  pinMode(32, INPUT_PULLUP);//3
  pinMode(15, INPUT_PULLUP);//3
  pinMode(5, INPUT_PULLUP);//

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

    tx_msg_adc.data_length_code=8;
    tx_msg_adc.identifier=0x115;
    tx_msg_adc.flags=CAN_MSG_FLAG_NONE;
  
    tx_msg_gps.data_length_code=8;
    tx_msg_gps.identifier=0x116;
    tx_msg_gps.flags=CAN_MSG_FLAG_NONE;
    
}

void loop() {

 int gear = 0; 
 int a1 = digitalRead(2);
 int a2 = digitalRead(4);
 int a3 = digitalRead(12);
 int a4 = digitalRead(5);
 int a5 = digitalRead(15);
 int a6 = digitalRead(32);

 if (a1 == 0)
    gear = 1;
 else if (a2 == 0)
    gear = 2;
 else if (a3 == 0)
    gear = 4;
 else if (a4 == 0)
    gear = 3;
 else if (a5 == 0)
    gear = 6;
//  else if (a6 == 0)
//     gear = 5;
 else
    gear = 0;

  tx_msg_adc.data[6] = gear;
  Log.noticeln("Gear: %d", gear);
if (Serial2.available() > 0) {
      if (gps.encode(Serial2.read())) {
        if (gps.location.isValid()) {
          latitude = gps.location.lat();
          longitude = gps.location.lng();
        }
        if (gps.speed.isValid()) {
          speed = gps.speed.kmph();

        } 
      }
    }
   // Log.noticeln("Latitude: %d, Longitude: %d, Speed: %l", latitude, longitude, speed);

  Serial.print("Latitude: ");
  Serial.println(latitude, 6);
  Serial.print("Longitude: ");
  Serial.println(longitude, 6);
  Serial.print("Speed: ");
  Serial.println(speed);
  




  #if LOG_LEVEL==LOG_LEVEL_VERBOSE

  // Serial.print("Gear: ");
  // Serial.print(gear);
  // Serial.println();
  
  // Log.noticeln("Latitude: %d, Longitude: %d, Speed: %l", latitude, longitude, speed);

  convert(fractional(latitude), tx_msg_gps.data);
  convert(fractional(longitude), tx_msg_gps.data+3);
  

  tx_msg_gps.data[7]=uint8_t(speed);

  Serial.print(tx_msg_gps.data[0]);
  Serial.print(tx_msg_gps.data[1]);
  Serial.print(tx_msg_gps.data[2]);
  //int d1 = BSPD.getVoltage();
  int d2 = brakePressure.getVoltage();
  int d3 = steeringAngle.getVoltage();
  int d4 = oilPressure.getVoltage();
  //Log.verboseln("BSPD: %d, Brake pressure: %d, Steering Angle: %d, Oil Pressure: %d", d1, d2, d3, d4);

 
  // tx_msg_adc.data[0]=d1/100;
  // tx_msg_adc.data[1]=d1%100;
  tx_msg_adc.data[0]=d2/100;
  tx_msg_adc.data[1]=d2%100;
  tx_msg_adc.data[2]=d3/100;
  tx_msg_adc.data[3]=d3%100;
  tx_msg_adc.data[4]=d4/100;
  tx_msg_adc.data[5]=d4%100;


  #else
  // if(Serial2.available() > 0){
  //   if (gps.encode(Serial2.read())){ 
  //     if (gps.location.isValid()) {
  //       convert(fractional(gps.location.lat()), tx_msg_gps.data);
  //       convert(fractional(gps.location.lng()), tx_msg_gps.data+3);
  //     }
  //     else {
  //       Log.errorln("GPS position is INVALID");
  //       tx_msg_gps.data[0]=0xff;
  //       tx_msg_gps.data[1]=0xff;
  //       tx_msg_gps.data[2]=0xff;
  //       tx_msg_gps.data[3]=0xff;
  //       tx_msg_gps.data[4]=0xff;
  //       tx_msg_gps.data[5]=0xff;
        
  //     }
  //     if(gps.speed.isValid())
  //       tx_msg_gps.data[6]=gps.speed.kmph(); 
  //     else {
  //       Log.errorln("GPS speed is INVALID");
  //       tx_msg_gps.data[6]=0xff;
  //   }
  //   }
  //   else {
  //     Log.errorln("Data from GPS is invalid");
  //     tx_msg_gps.data[0]=0xee;
  //     tx_msg_gps.data[1]=0xee;
  //     tx_msg_gps.data[2]=0xee;
  //     tx_msg_gps.data[3]=0xee;
  //     tx_msg_gps.data[4]=0xee;
  //     tx_msg_gps.data[5]=0xee;
  //     tx_msg_gps.data[6]=0xee;
  //     }
  // }
  // else {
  //   Log.errorln("GPS serial is not available");
  //     tx_msg_gps.data[0]=0xdd;
  //     tx_msg_gps.data[1]=0xdd;
  //     tx_msg_gps.data[2]=0xdd;
  //     tx_msg_gps.data[3]=0xdd;
  //     tx_msg_gps.data[4]=0xdd;
  //     tx_msg_gps.data[5]=0xdd;
  //     tx_msg_gps.data[6]=0xdd;
  //   }

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
    //Log.noticeln("Can message ADC sent");
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
    //Log.noticeln("Can message GPS sent");
  }
  else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if(status==ESP_OK) Log.errorln("Can driver restarted");
  }
  //vTaskDelay(pdMS_TO_TICKS(100));

}