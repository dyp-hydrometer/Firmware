/**
* @file dpy_Hydrometer.ino
* @brief This is source file for DPY Hydrometer. This firmware file will 
* setup the nRF52 based Bluefruit LE and Adafruit Adafruit_BNO055
*
* @author Minh Luong
*
*/
#include <bluefruit.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

/** HYDROMETER Service Definitions
 * Hydrometer Service:  0x2468
 * Gravity Measurement Char: 0x0001
 * Battery Level Measurement Char:   0x0002
 */
#define UUID16_HYDS   0x2468
#define UUID16_GRMC   0x0001
#define UUID16_CMDC   0x0002
#define UUID16_BATC   0x0003

/// Command IDs for user app
#define CMD_SET_INTERVAL            0x01
#define CMD_GET_INTERVAL            0x02
#define CMD_GET_CALIBRATION_STATUS  0x03

///Sampling every 100 ms
#define BNO055_SAMPLERATE_DELAY_MS (100)

///Adafruit batter pin
#define VBAT_PIN          (A7)

/// 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_MV_PER_LSB   (0.73242188F)

/// 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER      (0.71275837F)

/// Compensation factor for the VBAT divider
#define VBAT_DIVIDER_COMP (1.403F)

/// Create the BLE service and characteristics
BLEService        hyds = BLEService(UUID16_HYDS);
BLECharacteristic grmc = BLECharacteristic(UUID16_GRMC);
BLECharacteristic cmdc = BLECharacteristic(UUID16_CMDC);
BLECharacteristic batc = BLECharacteristic(UUID16_BATC);

BLEDis bledis;                // DIS (Device Information Service) helper class instance
BLEBas blebas;                // BAS (Battery Service) helper class instance

uint32_t grmInterval = 5;
uint8_t  notifyEnable = 0;
uint8_t  toggleRedLED = 0;
uint8_t  cmdData[5];

/// Variable to determine the tilt from the 3 axis accelerometer for angle measurement
/// Accelerometer variables
float thetaM; 
float phiM;
float thetaFold = 0;
float thetaFnew;
float phiFold = 0;
float phiFnew;

/// Gyrospoce variables for angle measurement
float thetaG = 0;
float phiG = 0;
float dt;
unsigned long millisOld;

/// Final overall measurement after combining accelerometer and gyrospoce data
float theta;
float phi;

/// Angles in radians
float thetaRad;
float phiRad;

/// Variables to calculate Z axis angle 
float Xm;
float Ym;
float psi;

/// Variables used for calibration 
uint8_t cal_system, cal_gyroo, cal_accel, cal_mg = 0;

/// Creating an object
Adafruit_BNO055 myIMU = Adafruit_BNO055();

int       vbat_raw;
uint8_t   vbat_per;
float     vbat_mv;
uint32_t  countDown;;                            /// countdown interval to get data      
int8_t    tempFahrenheit;

/**
 * This method will setup all devices
 * @param None 
 */
void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);                  /// for nrf52840 with native usb

  Serial.println("DYP HYDROMETER");

  /// Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  /// Set the advertised device name (keep it short!)
  Serial.println("Setting Device Name to 'DYP Hydrometer'");
  Bluefruit.setName("DYP Hydrometer");

  /// Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  /// Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  /// Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  /// Setup the Heart Rate Monitor service using
  /// BLEService and BLECharacteristic classes
  Serial.println("Configuring the Hydrometer Service");
  setupHYDS();

  /// Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();
  
  Serial.println("\nAdvertising");

  myIMU.begin();

  myIMU.setExtCrystalUse(true);
  millisOld = millis();

  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);               /// Can be 8, 10, 12 or 14

  delay(1000);
}

/**
 * This method will setup and start the BLE advertising
 * @param None 
 */
void startAdv(void)
{
  /// Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  /// Include HRM Service UUID
  Bluefruit.Advertising.addService(hyds);

  /// Include Name
  Bluefruit.Advertising.addName();
  
  /** Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    /// in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      /// number of seconds in fast mode
  Bluefruit.Advertising.start(0);                /// 0 = Don't stop advertising after n seconds  
}

/**
 * This method will setup all characteristics
 * @param None 
 */
void setupHYDS(void)
{
  hyds.begin();

  /// Setup for Gravity Measurement Characteristic
  grmc.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  grmc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  grmc.setFixedLen(13);
  grmc.setCccdWriteCallback(cccd_callback);     /// Optionally capture CCCD updates
  grmc.begin();

  /// Setup for Command Characteristic
  cmdc.setProperties(CHR_PROPS_WRITE | CHR_PROPS_READ);
  cmdc.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  cmdc.setFixedLen(5);
  cmdc.setWriteCallback(write_callback);
  cmdc.begin();

  /// Set up battery level characterristics
  batc.setProperties(CHR_PROPS_READ);
  batc.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  batc.setFixedLen(1);
  batc.begin();
}

/**
 * Callback invoked when a connection is established
 * @param conn_handle connection where this event happens
 */
void connect_callback(uint16_t conn_handle)
{
  /// Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  Serial.println("Advertising!");
}

/**
 * Callback invoked when user subscribed to the gravity characteristic
 * @param conn_hdl connection where this event happens
 * @param chr the characteristic pointer
 * @param cccd_value the cccd value
 */
void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
    /// Display the raw request packet
    Serial.print("CCCD Updated: ");
    Serial.print(cccd_value);
    Serial.println("");

    /// Check the characteristic this CCCD update is associated with in case
    /// this handler is used for multiple CCCD records.
    if (chr->uuid == grmc.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
            Serial.println("Gravity Measurement 'Notify' enabled");
            notifyEnable = 1;
        } else {
            Serial.println("Gravity Measurement 'Notify' disabled");
            notifyEnable = 0;
        }
    }
}

/**
 * Callback invoked when user wrote to command characteristic
 * @param conn_hdl connection where this event happens
 * @param chr the characteristic pointer
 * @param data the pointer to the input data
 * @param len the input data length
 */
void write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  if (chr->uuid == cmdc.uuid) 
  {
    Serial.print("Received data: ");      
    Serial.println((char *)data);
    switch (data[0])
    {
      case CMD_SET_INTERVAL:                    /// Set the interval for the notification characteristic
          toggleRedLED = 0;
          digitalWrite(LED_RED, 1);
          grmInterval = ((uint32_t)data[1] << 24) & 0xFF000000;
          grmInterval |= ((uint32_t)data[2] << 16) & 0x00FF0000;
          grmInterval |= ((uint32_t)data[3] << 8) & 0x0000FF00;
          grmInterval |= (uint32_t)data[4];
          grmInterval = grmInterval;       
          Serial.print(grmInterval, DEC);  
          break;
      case CMD_GET_INTERVAL:                  /// Get the previous interval for the notification characteristic
          toggleRedLED = 1;
          cmdData[0] = 0x20;
          cmdData[1] = (grmInterval >> 24);
          cmdData[2] = (grmInterval >> 16) & 0x000000FF;
          cmdData[3] = (grmInterval >> 8) & 0x000000FF;
          cmdData[4] = grmInterval & 0x000000FF;
          cmdc.write(cmdData, 5);
          digitalToggle(LED_RED);
          break;
      case CMD_GET_CALIBRATION_STATUS:        /// Get the hardware calibration status
          toggleRedLED = 0;
          cmdData[0] = 0x30;
          cmdData[1] = cal_accel;
          cmdData[2] = cal_gyroo;
          cmdData[3] = cal_mg;
          cmdData[4] = cal_system;
          cmdc.write(cmdData, 5);     
          break;
    }
  }
}

/**
 * This method will print out the data in Hex format
 * @param str the array of byte data
 * @param len the input data length
 */
void print_bytes(byte *str, uint8_t len)
{
    for(int i=0; i<len; i++)
        Serial.print(str[i], HEX);
    Serial.println();
}

/**
 * This method will convert the float value to array of bytes
 * @param bytes_temp the array of output bytes
 * @param float_variable the float value
 */
void float2Bytes(byte bytes_temp[4],float float_variable){ 
  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = float_variable;
  memcpy(bytes_temp, thing.bytes, 4);
}

/**
 * This method will convert battery voltage to level
 * @param mvolts the current voltage of battery  
 * @return the battery level
 */
uint8_t mvToPercent(float mvolts) {
  uint8_t battery_level;

  if (mvolts >= 3000) {
    battery_level = 100;
  } else if (mvolts > 2900) {
    battery_level = 100 - ((3000 - mvolts) * 58) / 100;
  } else if (mvolts > 2740) {
    battery_level = 42 - ((2900 - mvolts) * 24) / 160;
  } else if (mvolts > 2440) {
    battery_level = 18 - ((2740 - mvolts) * 12) / 300;
  } else if (mvolts > 2100) {
    battery_level = 6 - ((2440 - mvolts) * 6) / 340;
  } else {
    battery_level = 0;
  }

  return battery_level;
}

/**
 * This method will read the battery voltage from output pin
 * @param None
 */
void readBatteryLevel()
{
  vbat_raw = analogRead(VBAT_PIN);            /// Read battery voltage from output pin
  vbat_per = mvToPercent(vbat_raw * VBAT_MV_PER_LSB);

  /// FIXME: Using C-style cast.  Use static_cast<float>(...)
  /// instead  [readability/casting] [4]
  /// Remove [readability/casting] ignore from Makefile
  vbat_mv = (float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;

  Serial.print("ADC = ");
  Serial.print(vbat_raw * VBAT_MV_PER_LSB);
  Serial.print(" mV (");
  Serial.print(vbat_raw);
  Serial.print(") ");
  Serial.print("LIPO = ");
  Serial.print(vbat_mv);
  Serial.print(" mV (");
  Serial.print(vbat_per);
  Serial.println("%)");
}

/**
 * This method will read X, Y, and Z values from Accelerometer, Gyroscope, and Magnetometer
 * @param None
 */
void getXYZ()
{
  /// Compact datatype = int8_t
  int8_t tempCelsius = myIMU.getTemp();
    
  ///Using the external temperature sensor that is on the board and not the one on the chip.
  myIMU.setExtCrystalUse(true);
  
  tempFahrenheit = (tempCelsius*1.8) + 32;

  /// Ranging from level 0-3 = CALIBRATION
  myIMU.getCalibration(&cal_system,&cal_gyroo,&cal_accel, &cal_mg);
  
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  /// Calculating theta in degrees = Tilt (accelerometer)
  thetaM = -atan2(acc.x()/9.8,acc.z()/9.8)/2/3.14*360;
  phiM = -atan2(acc.y()/9.8,acc.z()/9.8)/2/3.14*360;

  /// Low pass fiter data
  thetaFnew = .75*thetaFold + .25*thetaM;
  phiFnew = .75*phiFold + .25*phiM;

  dt = (millis() - millisOld)/1000.;
  millisOld = millis();

  /// Final measurement for theta(x) and phi(y)
  theta = (theta + gyro.y()*dt)*.95 + thetaM*.05;
  phi = (phi - gyro.x()*dt)*.95 + phiM*.05;

  /// Finding angle using gyroscope measurements
  thetaG = thetaG + gyro.y()*dt;
  phiG = phiG - gyro.x()*dt;

  /// Converting the angle in radians
  phiRad = phi/360*(2*3.14);
  thetaRad = theta/360*(2*3.14);

  /// Using magnetometer data, finding the z-axis angle
  Xm = mag.x()*cos(thetaRad) - mag.y()*sin(phiRad)*sin(thetaRad) + mag.z()*cos(phiRad)*sin(thetaRad);
  Ym = mag.y()*cos(phiRad) + mag.z()*sin(phiRad);

  psi = ((atan2(Ym,Xm)/(2*3.14))*360);

  /// Print Statements
  Serial.print("Temperature in Celsius:    ");
  Serial.println(tempCelsius);

  Serial.print("Temperature in Fahrenheit: ");
  Serial.println(tempFahrenheit);
  
  Serial.print("Accelerometer  X, ");
  Serial.print("Y, ");
  Serial.print("Z: ");
  Serial.print(acc.x()/9.8);
  Serial.print(", ");
  Serial.print(acc.y()/9.8);
  Serial.print(", ");
  Serial.println(acc.z()/9.8);

  Serial.print("Gyroscope      X, ");
  Serial.print("Y, ");
  Serial.print("Z: ");
  Serial.print(gyro.x()/9.8);
  Serial.print(",");
  Serial.print(gyro.y()/9.8);
  Serial.print(",");
  Serial.println(gyro.z()/9.8); 

  Serial.print("Magnetometer   X, ");
  Serial.print("Y, ");
  Serial.print("Z: ");
  Serial.print(mag.x()/9.8);
  Serial.print(",");
  Serial.print(mag.y()/9.8);
  Serial.print(",");
  Serial.println(mag.z()/9.8);
  
  Serial.print("Calibration level(0-3) of Accelerometer: ");
  Serial.println(cal_accel);
  Serial.print("Calibration level(0-3) of Gyroscope:     "); 
  Serial.println(cal_gyroo);
  Serial.print("Calibration level(0-3) of Magnetometer:  ");
  Serial.println(cal_mg);
  Serial.print("Calibration level(0-3) of the system:    ");
  Serial.println(cal_system);

  Serial.print("X angle(acce filtered): ");
  Serial.println(thetaFnew);
  Serial.print("Y angle(acce filtered): ");
  Serial.println(phiFnew);
  Serial.print("Z angle: ");
  Serial.println(psi);
  Serial.println();
  Serial.println();
  
  /// Setting back the value to new value(0) while going back to loop
  thetaFold = thetaFnew;
  phiFold = phiFnew;
}

/**
 * This method will send the gravity data to user side
 * @param None
 */
void sendData()
{
  uint8_t grmdata[14];                            
  uint8_t grmdata1[4];
  uint8_t grmdata2[4];
  uint8_t grmdata3[4];

  if ( Bluefruit.connected() ) 
  {
    if(toggleRedLED)
        digitalToggle(LED_RED);
        
    /// Note: We use .notify instead of .write!
    /// If it is connected but CCCD is not enabled
    /// The characteristic's value is still updated although notification is not sent
    if(notifyEnable == 1)
    {
      float2Bytes(grmdata1, thetaFnew);
      float2Bytes(grmdata2, phiFnew);
      float2Bytes(grmdata3, psi);
      
      memcpy(grmdata, grmdata1, 4);
      memcpy(&grmdata[4], grmdata2, 4);
      memcpy(&grmdata[8], grmdata3, 4);
      grmdata[12] = tempFahrenheit;
      
      if ( grmc.notify(grmdata, sizeof(grmdata)) ){
          Serial.print("Sent angle data");
          print_bytes(grmdata, 13);
      }else{
          Serial.println("ERROR: Notify not set in the CCCD or not connected!");
      }
    } 
  }
  
}

/**
 * This is main loop of firmware
 * @param None
 */
void loop()
{
  readBatteryLevel();

  if(countDown == 0)
  {
    Serial.print("Timeout: ");
    Serial.println(grmInterval);
    countDown = grmInterval;
    getXYZ();
    sendData();
  }
  else
    countDown--;
  
  batc.write8(vbat_per);
  
  /// Only send update once per second
  delay(1000);
}
