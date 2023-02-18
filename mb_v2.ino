/* SDM630 Parameters Address 
 * Doc: https://xn--stromzhler-v5a.eu/media/pdf/cf/8f/92/SDM630-Modbus-V2-manual-incl-protocoll.pdf
 * 
 * 
 * L1N     0x00   L2N      0x02  L3N     0x04  
 * I1      0x06   I2       0x08  I3      0x0A 
 * P1      0x0C   P2       0x0E  P3      0x10
 * eTotal  0x56   qTotal   0x5C
 * netkW   0x8C   netkVarh 0x8E
 * 
 * Created by Ion Butnarciuc
 * This code is in the public domain.
*/

#include "ModbusMaster.h"
#include "SoftwareSerial.h"

#define SLAVE_ADDRESS 4         // Device ID
#define SLAVE_BAUDRATE 19200    // Device BAUD
#define POLL_INTERVAL 1000      // Interval to poll in ms
#define POLL_COOLDOWN 5         // Interval between two poll

// Storing all measurements in one big structure

struct Measurement{
  float V1N, V2N, V3N, I1, I2, I3, P1, P2, P3, eTotal, qTotal, netkW, netkVarh;
};

unsigned long previousMillis = 0;

// Initialization of Modbus Node
ModbusMaster node_meter;

// Initialization of Serial Software
SoftwareSerial rs485serial(0,2);  // RX,TX

// Creating struct Measurement

Measurement measurement;

void setup() {

  // Serial initialization for debugging
  Serial.begin(115200);

  // Call function to enable communication

  initCommunication();
  
}

void loop() {

  // Get the current time in ms
  unsigned long currentMillis = millis();

  // If elapsed time is bigger than our poll interval, read data
  if (currentMillis - previousMillis >= POLL_INTERVAL) {

      pollDevice();
      Serial.print("V1N: ");
      Serial.print(measurement.V1N);
      Serial.print("|| ");
      Serial.print("I1: ");
      Serial.print(measurement.I1);
      Serial.print("|| ");
      Serial.print("P1: ");
      Serial.print(measurement.P1);
      Serial.println("|| ");
      Serial.print("EnergyTotal: ");
      Serial.print(measurement.eTotal);
      Serial.print("|| ");
      Serial.print("QTotal: ");
      Serial.print(measurement.qTotal);
      Serial.print("|| ");
      Serial.print("EnergyNet: ");
      Serial.print(measurement.netkW);
      Serial.print("|| ");
      Serial.print("QNet: ");
      Serial.println(measurement.netkVarh);
  }
}


void initCommunication(){
  
  // Delay in order to stabilize
  delay(10);
  
  // Set Baudarate for communication
  rs485serial.begin(SLAVE_BAUDRATE);

  // Delay in order to stabilize
  delay(10);

  // Initialize communication with Device
  node_meter.begin(SLAVE_ADDRESS,rs485serial);
}

void pollDevice(){

  bool success1 = false, success2 = false, success3 = false;
  // Defining a variable to store result data
  uint32_t result_data;

  // 04 function to read Input Registers of Device 
  // Starting from Address 0, we read 9 parameters [V1N to P3], so the length is 18.

  result_data = node_meter.readInputRegisters(0x0000,18);
  
  // If querry succeded
  if (result_data == node_meter.ku8MBSuccess){
     // Typecast two 16-bit Registers into 32-bit
     uint32_t V1N = (static_cast<uint32_t>(node_meter.getResponseBuffer(0)) << 16) + static_cast<uint32_t>(node_meter.getResponseBuffer(1));
     // Store the formated value in struct V1N
     measurement.V1N = *(float*) &V1N;

     // Typecast two 16-bit Registers into 32-bit
     uint32_t I1 = (static_cast<uint32_t>(node_meter.getResponseBuffer(6)) << 16) + static_cast<uint32_t>(node_meter.getResponseBuffer(7));
     // Store the formated value in struct I1
     measurement.I1 = *(float*) &I1;

     // Typecast two 16-bit Registers into 32-bit
     uint32_t P1 = (static_cast<uint32_t>(node_meter.getResponseBuffer(16)) << 16) + static_cast<uint32_t>(node_meter.getResponseBuffer(17));
     // Store the formated value in struct P1
     measurement.P1 = *(float*) &P1;
    
  } 
  else{
      Serial.println("First Poll Failed");

       // set to NaN if failed
      measurement.V1N = 1.0 / 0.0;
      measurement.I1 = 1.0 / 0.0;
      measurement.P1 = 1.0 / 0.0;
  }

  delay(POLL_COOLDOWN);

  // Reading Total kWh( Import + Export) and kVarh( Import + Export)
  
  result_data = node_meter.readInputRegisters(0x56,4);
  
  // If querry succeded
  if (result_data == node_meter.ku8MBSuccess){
     // Typecast two 16-bit Registers into 32-bit
     uint32_t eTotal = (static_cast<uint32_t>(node_meter.getResponseBuffer(0)) << 16) + static_cast<uint32_t>(node_meter.getResponseBuffer(1));
     // Store the formated value in struct eTotal
     measurement.eTotal = *(float*) &eTotal;

     // Typecast two 16-bit Registers into 32-bit
     uint32_t qTotal = (static_cast<uint32_t>(node_meter.getResponseBuffer(2)) << 16) + static_cast<uint32_t>(node_meter.getResponseBuffer(3));
     // Store the formated value in struct qTotal
     measurement.qTotal = *(float*) &qTotal;
     
  } 
  else {
      Serial.println("Second Poll Failed");

      // set to NaN if failed
      measurement.eTotal = 1.0 / 0.0;
      measurement.qTotal = 1.0 / 0.0;
  }

  delay(POLL_COOLDOWN);

   // Reading net kWh( Import - Export) and kVarh( Import - Export)
  
  result_data = node_meter.readInputRegisters(0x56,4);
  
  // If querry succeded
  if (result_data == node_meter.ku8MBSuccess){
     // Typecast two 16-bit Registers into 32-bit
     uint32_t netkW = (static_cast<uint32_t>(node_meter.getResponseBuffer(0)) << 16) + static_cast<uint32_t>(node_meter.getResponseBuffer(1));
     // Store the formated value in struct eTotal
     measurement.netkW = *(float*) &netkW;

     // Typecast two 16-bit Registers into 32-bit
     uint32_t netkVarh = (static_cast<uint32_t>(node_meter.getResponseBuffer(2)) << 16) + static_cast<uint32_t>(node_meter.getResponseBuffer(3));
     // Store the formated value in struct qTotal
     measurement.netkVarh = *(float*) &netkVarh;

  } 
  else{
    Serial.println("Third Poll Failed");
    
    // set to NaN if failed
    measurement.netkW = 1.0 / 0.0; 
    measurement.netkVarh = 1.0 / 0.0;
  }
}
