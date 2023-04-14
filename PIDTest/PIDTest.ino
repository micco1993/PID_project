/*
  pid

  This example creates a Bluetooth® Low Energy peripheral with service that contains a
  characteristic to control an pid.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic Bluetooth® Low Energy central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
#include "BLEReadAndWrite.h"


BLEService pidService("9A48ECBA-2E92-082F-C079-9E75AAE428B1"); // Bluetooth® Low Energy pid Service

// Bluetooth® Low Energy pid Characteristics - custom 128-bit UUID, read and writable by central
BLEFloatCharacteristic setpointCharacteristic("C8F88594-2217-0CA6-8F06-A4270B675D69", BLERead | BLEWrite);
BLEFloatCharacteristic proportionalCharacteristic("E3ADBF53-950E-DC1D-9B44-076BE52760D6", BLERead | BLEWrite | BLENotify);
BLEDescriptor proportionalCharDesc("2901", "proportionalGain"); //characteristics with notify needs a descriptor
BLEFloatCharacteristic integralCharacteristic("3e1a4868-144a-4052-8228-4bfecaf270d3", BLERead | BLEWrite | BLENotify);
BLEDescriptor integralCharDesc("3500", "integTime");
BLEFloatCharacteristic derivativeCharacteristic("2f422d7e-732f-4cfc-86fa-c8a5eb9df81e", BLERead | BLEWrite | BLENotify);
BLEDescriptor derivativeCharDesc("6999", "derivTime");
BLEFloatCharacteristic offsetCharacteristic("a5a2cc51-df41-402f-8e7e-92bf6dda1df0", BLERead | BLEWrite);
BLEFloatCharacteristic filterCharacteristic("a238ae45-aee6-4f99-93eb-e440099e1f1d", BLERead | BLEWrite);
BLEIntCharacteristic sampleTimeCharacteristic("e731f69f-95c3-48d4-85f2-dfc3a8365d2f", BLERead | BLEWrite);

BLEFloatCharacteristic processValueCharacteristic("8b8bdfdb-c4e0-44fc-af53-63ae40168421", BLERead | BLEWrite | BLENotify);
BLEDescriptor pvCharDesc("4520", "processValueDescriptor");
BLEFloatCharacteristic controlSignalCharacteristic("ac02c1e3-22b4-4de9-9938-47be1c53773c", BLERead | BLEWrite | BLENotify);
BLEDescriptor csCharDesc("7531", "controlSignalDescriptor");
BLEIntCharacteristic autoTuneCharacteristic("c73ad3eb-0f52-4299-8630-0243f43be20a", BLERead | BLEWrite);

const int pvPin = A0; //process signal connected to pin A0
const int csPin = 2; //control signal connected to pin 2

float setpoint = 0, proportionalGain = 0, integralTime = 99999, integralGain, derivativeTime = 0, derivativeGain, offset = 0,
      controlSignal = 0, limitedCs = 0, processValue = 0, error, integral = 0, derivative = 0, errorOld = 0, filterWeight = 0.7, lastPv = 0;
float processValues[100];

int sampleTime = 100, csMax = 1023, csMin = 0;
int sumRead = 0, readCount = 0, filteredValue, lastFilteredValue = 0, pvCounter = 0;
bool turnOn, autoTune = false, startReading = false;

void writeControlSignal(float cs) {
  int csToBits = cs * 3199/ 1023; //Have to write a value between 0 and 255 (0 - 3,3 V) to analog output
  REG_TCC0_CCB0 = csToBits;                               // TCC0 CC0 - 50% duty cycle on D2
  while (TCC0->SYNCBUSY.bit.CCB0);                 // Wait for synchronization
}

float limitCs(float cs) { //limit control signal to be between 0 and 3,3 V
  if (cs > csMax) {
    cs = csMax;
  }
  else if (cs < csMin) {
    cs = csMin;
  }
  return cs;
}

void calculateControlSignal() {

  processValue = sumRead/readCount;

  if(derivativeTime > 0) { //if derivation is used more filtering is needed
    filteredValue = filterWeight * processValue + (1 - filterWeight) * lastFilteredValue;
    lastFilteredValue = filteredValue;
    processValue = filteredValue;
  }
  //Serial.println(processValue);
  
  error = setpoint - processValue + offset;
  

  controlSignal = proportionalGain * error + integral + derivative;
  limitedCs = limitCs(controlSignal);
  writeControlSignal(limitedCs);

  if (integralTime < 99999 && limitedCs == controlSignal) { //integral is counted if integration is active and control signal is in allowed range
    integral += integralGain * error;
  }
  else {
    integral = 0;
  }

  if (derivativeTime > 0) { //derivative is counted if derivation is active
    derivative = derivativeGain * (error - errorOld);
  }
  else {
    derivative = 0;
  }
  errorOld = error;
}




void setup() {
  Serial.begin(9600);

  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 2: 48MHz/2=24MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;         // Setup single slope PWM on TCC0
  while (TCC1->SYNCBUSY.bit.WAVE);                 // Wait for synchronization

  TCC0->PER.reg = 3199;                            // Set the frequency of the PWM on TCC0 to 15kHz: 48MHz / (3199 + 1) = 15kHz
  while (TCC1->SYNCBUSY.bit.PER);                  // Wait for synchronization

  REG_TCC0_CCB0 = 0;                               // TCC0 CC0 - 50% duty cycle on D2
  while (TCC0->SYNCBUSY.bit.CCB0);                 // Wait for synchronization
  
  TCC0->CTRLA.bit.ENABLE = 1;                     // Enable the TCC0 counter
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module!");

    while (1);
  }
  

  // set advertised local name and service UUID:
  BLE.setLocalName("PID");
  BLE.setAdvertisedService(pidService);

  //add descriptors for the characteristics that use notify
  proportionalCharacteristic.addDescriptor(proportionalCharDesc);
  integralCharacteristic.addDescriptor(integralCharDesc);
  derivativeCharacteristic.addDescriptor(derivativeCharDesc);
  processValueCharacteristic.addDescriptor(pvCharDesc);
  controlSignalCharacteristic.addDescriptor(csCharDesc);

  // add the characteristic to the service
  pidService.addCharacteristic(setpointCharacteristic);
  pidService.addCharacteristic(proportionalCharacteristic);
  pidService.addCharacteristic(integralCharacteristic);
  pidService.addCharacteristic(derivativeCharacteristic);
  pidService.addCharacteristic(offsetCharacteristic);
  pidService.addCharacteristic(sampleTimeCharacteristic);
  pidService.addCharacteristic(filterCharacteristic);
  pidService.addCharacteristic(controlSignalCharacteristic);
  pidService.addCharacteristic(processValueCharacteristic);
  pidService.addCharacteristic(autoTuneCharacteristic);

  // add service
  BLE.addService(pidService);

  // start advertising
  BLE.advertise();
  Serial.println("BLE PID Peripheral");
}

void loop() {
  //Serial.println(REG_TCC2_PER);
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    if (central.connected()) { //write the values that are in the pid at this moment to the central
      writeFloatValue(setpointCharacteristic, setpoint / 10.23);
      writeFloatValue(proportionalCharacteristic, proportionalGain);
      writeFloatValue(integralCharacteristic, integralTime);
      writeFloatValue(derivativeCharacteristic, derivativeTime);
      writeFloatValue(offsetCharacteristic, offset / 10.23);
      writeFloatValue(filterCharacteristic, filterWeight);
      sampleTimeCharacteristic.writeValue(sampleTime);
    }

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // the values will be updated
      if (setpointCharacteristic.written()) {
        setpoint = readFloatCharacteristic(setpointCharacteristic) * 10.23;
        Serial.println(setpoint);
      }
      if (proportionalCharacteristic.written()) {
        proportionalGain = readFloatCharacteristic(proportionalCharacteristic);
        Serial.println(proportionalGain);
      }
      if (integralCharacteristic.written()) {
        integralTime = readFloatCharacteristic(integralCharacteristic);
        Serial.println(integralTime);
      }
      if (derivativeCharacteristic.written()) {
        derivativeTime = readFloatCharacteristic(derivativeCharacteristic);
        Serial.println(derivativeTime);
      }
      if (offsetCharacteristic.written()) {
        offset = 10.23 * readFloatCharacteristic(offsetCharacteristic);
        Serial.println(offset);
      }
      if (filterCharacteristic.written()) {
        filterWeight = readFloatCharacteristic(filterCharacteristic);
        Serial.println(filterWeight);
      }
      if (sampleTimeCharacteristic.written()) {
        sampleTime = readIntCharacteristic(sampleTimeCharacteristic);
        Serial.println(sampleTime);
        
        turnOn = true;
        autoTune = false;

        //when all values have been written the gains are calculated
        if (integralTime < 99999 && derivativeTime > 0) { // integration and derivation active
          integralGain = proportionalGain / integralTime * sampleTime;
          derivativeGain = proportionalGain * derivativeTime / sampleTime;
        }
        else if (integralTime < 99999 && derivativeTime == 0) { //integration active
          integralGain = proportionalGain / integralTime * sampleTime;
          derivativeGain = 0;
        }
        else if (integralTime == 99999 && derivativeTime > 0) { //derivation active
          integralGain = 0;
          derivativeGain = proportionalGain * derivativeTime / sampleTime;
        }
        else { //integration and derivation inactive
          integralGain = 0;
          derivativeGain = 0;
        }
      }

      if (turnOn && setpoint != 0) { //read all process values for avg calculation
         sumRead = sumRead + analogRead(pvPin);
         Serial.println(analogRead(pvPin));
         readCount++;
      }
      if (turnOn && setpoint != 0 && millis() % sampleTime == 0) { //calculate a new control signal at each sample time interval
        calculateControlSignal();
        
        sumRead = 0;
        readCount = 0;

        float csInPercent = limitedCs * 100.0 / 1023; //cs from 0 - 1023 to 0 - 100
        float pvInPercent = processValue * 100.0 / 1023; //pv from 0 - 1023 to 0 - 100

        //write process value and control signal to central so that they can be plotted
        processValueCharacteristic.writeValue(pvInPercent);
        controlSignalCharacteristic.writeValue(csInPercent);
      }

      //stop controlling if set point is written to 0
      if (setpoint == 0) {
        writeControlSignal(0);
        turnOn = false;
        integral = 0;
        derivative = 0;
        sumRead = 0;
        errorOld = 0;
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
    writeControlSignal(0); //stop controlling if connection to central breaks
    turnOn = false;
  }
}
