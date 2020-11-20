#include <Arduino.h>
/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This uses OTAA (Over-the-air activation), where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <list>

#include "sps30.h"
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include "Adafruit_VEML7700.h"
#include "Adafruit_VEML6070.h"
#include <Adafruit_ADS1015.h>
#include <Adafruit_BME280.h>
#include "config.h"


//Power Management mit AXP1992 Chip
#include <esp_sleep.h>
#include <axp20x.h>
AXP20X_Class pmu;

SPS30 sps30;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
// This should also be in little endian format, see above.
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

TinyGPSPlus gps;

HardwareSerial GPSSerial(1);

struct __attribute__((__packed__)) DataPayload {
  uint16_t pressure;
  uint16_t wind_direction;
  uint16_t light_intensity;
  uint16_t CO2value;
  int8_t temperature;
  uint8_t humidity;
  uint8_t uv_index;
  uint8_t wind_speed;
  uint8_t light_parameters;
  uint8_t wind_peak;
};

struct DataPayloadGPS {
  int32_t gps_lng;
  int32_t gps_lat;
  int16_t gps_alt;
};

struct __attribute__((__packed__)) DataPayloadSPS30 {
  uint16_t mass_pm1;
  uint16_t mass_pm2;
  uint16_t mass_pm10;
};

static DataPayload data;
static DataPayloadGPS dataGPS;
static DataPayloadSPS30 dataSPS30;
static osjob_t preparesendjob;
static osjob_t sendjob;
static osjob_t sendjobaftergps;
static osjob_t luminationjob;
static osjob_t uvjob;
static osjob_t bmejob;
static osjob_t adsjob;
static osjob_t airsensorco2job;
static osjob_t sps30job;
static std::list<uint8_t> wind_mean;
static uint8_t wind_peak;
static SemaphoreHandle_t wind_speed_mtx = xSemaphoreCreateMutex();

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 23,  // war 5 bei v_0_7 - soll 23 sein bei > v_1_1
  .dio = {26, 33, 32},
};

// BME 280 foo
Adafruit_BME280 bme;

// UV Sensor
Adafruit_VEML6070 uv = Adafruit_VEML6070();

// Light sensor
Adafruit_VEML7700 light = Adafruit_VEML7700();

// ADS1115 ADC
Adafruit_ADS1115 ads;

//SCD30 Co2 Sensor
SCD30 airsensorCO2;

// Only send the GPS coordinates once after having a lock.
bool gpsWasSent = false;

uint8_t read_light_gain_from_data() {
  return (uint8_t)(data.light_parameters & 0x03);
}

uint8_t read_light_it_from_data() {
  return (uint8_t)((data.light_parameters >> 2) & 0x0F);
}

void set_lumination_reader_parameters() {
  light.setGain(read_light_gain_from_data());
  light.setIntegrationTime(read_light_it_from_data());
}

void write_light_gain_to_data(uint8_t gain) {
  data.light_parameters = (0xFC & data.light_parameters) + gain;
  set_lumination_reader_parameters();
}

void write_light_it_to_data(uint8_t it) {
  data.light_parameters = (0x03 & data.light_parameters) + (it<<2);
  set_lumination_reader_parameters();
}

void read_lumination_job(osjob_t* j) {
  // printf("Light: 0x%x\n", data.light_parameters);
  while (true) {
    data.light_intensity = (int16_t)(light.readALS());
    uint8_t gain = read_light_gain_from_data();
    uint8_t it = read_light_it_from_data();
    // printf("Gain: 0x%x, it: 0x%x\n", gain, it);
    if (gain == VEML7700_GAIN_1_8 && data.light_intensity < LIGHT_THRESHHOLD_DOWN_HIGH) {
      write_light_gain_to_data(VEML7700_GAIN_2);
    } else if (gain == VEML7700_GAIN_2 && it == VEML7700_IT_25MS && data.light_intensity < LIGHT_THRESHHOLD_DOWN_LOW) {
      write_light_it_to_data(VEML7700_IT_800MS);
    } else if (gain == VEML7700_GAIN_2 && it == VEML7700_IT_800MS && data.light_intensity > LIGHT_THREADHHOLD_UP_LOW) {
      write_light_it_to_data(VEML7700_IT_25MS);
    } else if (gain == VEML7700_GAIN_2 && it == VEML7700_IT_25MS && data.light_intensity > LIGHT_THRESHHOLD_UP_HIGH) {
      write_light_gain_to_data(VEML7700_GAIN_1_8);
    } else {
      break;
    }
    os_setTimedCallback(&luminationjob, os_getTime()+sec2osticks(1), read_lumination_job);
    return;
  }
  // printf("New Gain: 0x%x, it: 0x%x, Light: %d\n", read_light_gain_from_data(), read_light_it_from_data(), data.light_intensity);
  light.powerSaveEnable(true);
}

void read_lumination() {
  light.powerSaveEnable(false);
  os_setTimedCallback(&luminationjob, os_getTime()+sec2osticks(1), read_lumination_job);
}

void read_uv_job(osjob_t* j) {
  uint16_t uv_value = uv.readUV();
  float it_factor[4] = {0.5, 1, 2, 4};
  data.uv_index = (uint8_t)round(uv_value / 280 * (UV_R_SET_KOHM / 270) / it_factor[UV_IT]);
  uv.sleep(true);
}

void read_uv() {
  uv.sleep(false);
  os_setTimedCallback(&uvjob, os_getTime()+sec2osticks(4), read_uv_job);
}

void read_bme_job(osjob_t* j) {
  data.temperature = (int8_t)((bme.readTemperature() + 50) * 2);
  data.pressure = (uint16_t)(bme.readPressure() / 10);
  data.humidity = (uint8_t)(bme.readHumidity());
}

void read_bme() {
  bme.takeForcedMeasurement();
  os_setTimedCallback(&bmejob, os_getTime() + sec2osticks(1), read_bme_job);
}

void read_ads_job(osjob_t* j) {
  xSemaphoreTake(wind_speed_mtx, portMAX_DELAY);
  data.wind_direction = (uint16_t) (ads.readADC_SingleEnded(0)*360)/24500;
  uint8_t wind_speed = (uint8_t)(((ads.readADC_SingleEnded(1)*3.3/1023.0*6.0)-0.2516)*6.0); 
  wind_mean.push_back(wind_speed);
  wind_peak = max(wind_peak, wind_speed);
  xSemaphoreGive(wind_speed_mtx);
  os_setTimedCallback(&adsjob, os_getTime() + sec2osticks(10), read_ads_job);
}

void read_ads() {
  xSemaphoreTake(wind_speed_mtx, portMAX_DELAY);
  uint64_t wind_speed = 0;
  wind_mean.sort();
  if (wind_mean.size() > 0) {
    int five_percent = (int)floor(wind_mean.size() / 100 * 5);
    int i = 0;
    for (std::list<uint8_t>::iterator wm = wind_mean.begin(); wm != wind_mean.end(); wm++) {
      if (i >= five_percent && i <= wind_mean.size() - five_percent) {
        wind_speed += *wm;
      }
    }
    data.wind_speed = wind_speed / (wind_mean.size() - (five_percent * 2));
    data.wind_peak = wind_peak;

    wind_mean.clear();
    wind_peak = 0;
  }
  xSemaphoreGive(wind_speed_mtx);
}

void read_airsensorco2_job(osjob_t* j) {
  data.CO2value = airsensorCO2.getCO2();
  airsensorCO2.setForcedRecalibrationFactor(airsensorCO2.getCO2());   //wenn der aktuelle Messwert <400 o. >2000 , dann wird Kalibrierung angestoßen
}

void read_airsensorco2() {
  os_setTimedCallback(&airsensorco2job, os_getTime() + sec2osticks(3), read_airsensorco2_job);
}

void read_sps30_job(osjob_t* j) {
  struct sps_values v;
  sps30.GetValues(&v);
  dataSPS30.mass_pm1 = v.MassPM1 > 10 ? 65535 : (uint16_t)(v.MassPM1 * 6000);
  dataSPS30.mass_pm2 = v.MassPM2 > 10 ? 65535 : (uint16_t)(v.MassPM2 * 6000);
  dataSPS30.mass_pm10 = v.MassPM10 > 10 ? 65535 : (uint16_t)(v.MassPM10 * 6000);
  sps30.stop();
  // sps30.sleep();
}

void read_sps30() {
  // sps30.wakeup();
  if (!sps30.probe()) {
    Serial.println("Could not probe SPS30");
  }
  sps30.start();
  os_setTimedCallback(&sps30job, os_getTime() + sec2osticks(30), read_sps30_job);
}

int dataToSend = 1;

void do_send(osjob_t* j){
  // GPS sending
  Serial.print("GPS valid: ");
  Serial.println(gps.satellites.value());
  
  if (gps.location.isValid() && gps.altitude.isValid() && gps.satellites.isValid() && gps.satellites.value() > 5 && !gpsWasSent) {   
    dataGPS.gps_lng = (int32_t)(gps.location.lng() / 180 * INT32_MAX);
    dataGPS.gps_lat = (int32_t)(gps.location.lat() / 90 * INT32_MAX);
    dataGPS.gps_alt = (int16_t)(gps.altitude.meters());
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("GPS OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        int result = LMIC_setTxData2(1, (byte*)&dataGPS, sizeof(dataGPS), 0);
        if (result) {
          Serial.printf("Packet queuing failed: %d\n", result);
        }
        Serial.println(F("GPS Packet queued"));
        gpsWasSent = true;
    }
    if (!pmu.begin(Wire, AXP192_SLAVE_ADDRESS)) {
      pmu.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    }
    os_setTimedCallback(&sendjobaftergps, os_getTime()+sec2osticks(60), do_send);
    return;
  }

  byte* send_data;
  uint8_t send_data_len;
  uint8_t send_port;

  switch (dataToSend) {
    case 0:
      send_port = 2;
      Serial.println("Sending wheather data.");
      send_data = (byte*)&data;
      send_data_len = sizeof(data);
      break;
    case 1:
      send_port = 3;
      Serial.println("Sending air ppm data.");
      send_data = (byte*)&dataSPS30;
      send_data_len = sizeof(dataSPS30);
      break;
    default:
      Serial.println("Not sending any data - invalid dataToSend");
      dataToSend = 0;
      return;
  }
  dataToSend++;
  if (dataToSend > 1) {
    dataToSend = 0;
  }
  
  Serial.printf("Data size: %d\n", send_data_len);
  for(int i=0;i<send_data_len;i++)
  {
    Serial.println(send_data[i]);
  }

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
      // Prepare upstream data transmission at the next possible time.
      int result = LMIC_setTxData2(send_port, send_data, send_data_len, 0);
      if (result) {
        Serial.printf("Packet queuing failed: %d\n", result);
      }
      Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void prepare_send(osjob_t* j) {
  switch (dataToSend) {
    case 0:
      read_lumination();
      read_uv();
      read_bme();
      read_ads();
      read_airsensorco2();
      break;
    case 1:
      read_sps30();
      Serial.println("Lesen SPS30");
      break;
  }
  os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(35), do_send);
  os_setTimedCallback(&preparesendjob, os_getTime()+sec2osticks(TX_INTERVAL), prepare_send);
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.printf("Unknown event 0x%x (%d)\n", ev, ev);
      break;
  }
}

void AXP192_power(bool on) {
  if (on) {
    pmu.setPowerOutPut(AXP192_LDO2, AXP202_ON);  // Lora on T-Beam V1.0
    pmu.setPowerOutPut(AXP192_LDO3, AXP202_ON);  // Gps on T-Beam V1.0
    pmu.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // OLED on T-Beam v1.0
    pmu.setChgLEDMode(AXP20X_LED_OFF);
    // pmu.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    //pmu.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
  } else {
    pmu.setChgLEDMode(AXP20X_LED_OFF);
    pmu.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
    pmu.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    pmu.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    // pmu.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
  }
}

void AXP192_init() {      
  if (pmu.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 PMU initialization failed");
  } else {
    Serial.println("Initialzation...");
    // configure AXP192
    pmu.setDCDC1Voltage(3500);              // for external OLED display
    pmu.setTimeOutShutdown(false);          // no automatic shutdown
    pmu.setTSmode(AXP_TS_PIN_MODE_DISABLE); // TS pin mode off to save power

    // switch ADCs on
    pmu.adc1Enable(AXP202_BATT_VOL_ADC1, true);
    pmu.adc1Enable(AXP202_BATT_CUR_ADC1, true);
    pmu.adc1Enable(AXP202_VBUS_VOL_ADC1, true);
    pmu.adc1Enable(AXP202_VBUS_CUR_ADC1, true);

    pmu.setLDO2Voltage(3300);
    pmu.setLDO3Voltage(3300);

    // switch power rails on
    AXP192_power(true);

    Serial.println("AXP192 PMU initialized");
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("Starting"));
  //BME280 foo
  Wire.begin(21,22);
  int timeout = 0;

  GPSSerial.begin(GPSBaud, SERIAL_8N1, GPS_TX, GPS_RX);
  GPSSerial.setTimeout(2);

  AXP192_init();
  Serial.println("AXP192 initialisiert");
  delay(2000);
  AXP192_power(true);
  Serial.println("AXP192 powered on peripherals");
  delay(2000);
 

  while(!bme.begin(0x76,&Wire))
  {
    Serial.println("Could not find BME280 sensor!");
    delay(500);
    if (timeout++ == 10) {
      Serial.println("Continuing...");
      break;
    }
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED, Adafruit_BME280::SAMPLING_X16,
    Adafruit_BME280::SAMPLING_X16, Adafruit_BME280::SAMPLING_X16, Adafruit_BME280::FILTER_OFF,
    Adafruit_BME280::STANDBY_MS_500);
  //ENDE BME280 foo

  //UV Sensor initialisieren
  uv.begin(UV_IT);

  // ADS1115 init
  
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  
  ads.begin();


  // VEML7700 light sensor initialisation
  light.begin();
  write_light_gain_to_data(VEML7700_GAIN_1_8);
  write_light_it_to_data(VEML7700_IT_25MS);
  Serial.print("Gain VEML7700: ");
  Serial.println(light.getGain());
  Serial.print("Integration time VEML7700: ");
  Serial.println(light.getIntegrationTime());
  //light.powerSaveEnable(true);
  //light.setPowerSaveMode(VEML7700_POWERSAVE_MODE4);
  light.setLowThreshold(0);
  light.setHighThreshold(260);
  light.interruptEnable(true);
  delay(900);
  Serial.println("VEML7700 Interrupt: ");
  Serial.println(light.interruptStatus());

  // Feinstaubsensor
  sps30.begin(&Wire);

  // CO2 Sensor init
  airsensorCO2.begin();
  airsensorCO2.setMeasurementInterval(20); //Change number of seconds between measurements: 2 to 1800 (30 minutes)
  airsensorCO2.setAutoSelfCalibration(true);
  delay(2000);
  //My desk is ~1600m above sealevel
  airsensorCO2.setAltitudeCompensation(113); //Set altitude of the sensor in m

  //Pressure in Boulder, CO is 24.65inHg or 834.74mBar
  Serial.print("aktueller Luftdruck fuer Kalibrierung SCD30 ");
  Serial.println((int)(bme.readPressure()/100));
  airsensorCO2.setAmbientPressure((int)bme.readPressure()/100); //Current ambient pressure in mBar: 700 to 1200
  airsensorCO2.setTemperatureOffset(1); //Optionally we can set temperature offset to 5°C
  

  // Serial.println("wait for GPS");
  // waiting time for GPS module
  // delay(16000);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Start job (sending automatically starts OTAA too)
  prepare_send(&preparesendjob);
  read_ads_job(&adsjob);
}

int previousMillis = millis();

void loop() {
  if (previousMillis + 1000 < millis()) {
    previousMillis = millis();
    while (GPSSerial.available()) {
      uint8_t data = GPSSerial.read();
      gps.encode(data);
      //printf("%c", data);
    }
  }

  os_runloop_once();
}
