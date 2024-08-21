// Adapted from MPU6050_raw example file to generate offsets required to calibrate the MPU6050. 

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

/* Raw values array output
 * 
 * ax - accelerometer value in x xis
 * ay - accelerometer value in y xis
 * az - accelerometer value in z xis
 * gx - gyroscope value in x xis
 * gy - gyroscope value in y xis
 * gz - gyroscope value in z xis
 */

// Offset variables
int16_t rawValue[6];               // Raw sensor values for accelerometer and gyroscope for each axis: {ax, ay, az, gx, gy, gz}
int16_t totalRawValue[6];          // Total raw values over a number of sensor readings 
int16_t averageRawValue[6];        // Average raw values over a number of sensor readings
int16_t calculatedOffset[6] = {0}; // Calculated offset values from average raw sensor values; initialized to zero
int16_t counter = 1;               // Loop variable for calculating respective offset values
bool isOffsetCalculated = false;   // Set true once offsets have been calculated

MPU6050 accelgyro;                 // Instantiate an MPU6050 object

void setup() {
  
  // Initialize Wire library and join i2c bus.
  Wire.begin();

  // Initialize serial communication
  Serial.begin(38400);

  // Initialize device
  accelgyro.initialize();

  // Minor delay to allow the device to be fully setup.
  delay(500);

  // Verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Set the offset values of ax, ay, az, gx, gy and gz to the initial values of the calculatedOffset array.
  setOffsetValues();

  // Calculate sensor offsets
  calculateOffsets();
}

void loop(){
  
}

// Set sensor offset values
void setOffsetValues(){
  
  // Set sensor offset values to those of the calculatedOffset array.
  accelgyro.setXAccelOffset(calculatedOffset[0]);
  accelgyro.setYAccelOffset(calculatedOffset[1]);
  accelgyro.setZAccelOffset(calculatedOffset[2]);
  accelgyro.setXGyroOffset(calculatedOffset[3]);
  accelgyro.setYGyroOffset(calculatedOffset[4]); 
  accelgyro.setZGyroOffset(calculatedOffset[5]); 

  // Output offsets to be used in the arduino_gimbal.ino file.
  if(isOffsetCalculated == true){
    Serial.println("\nSensor offsets");
    Serial.println("--------------");
    Serial.print("ax: ");
    Serial.print(calculatedOffset[0]); Serial.print("\n");
    Serial.print("ay: ");
    Serial.print(calculatedOffset[1]); Serial.print("\n");
    Serial.print("az: ");
    Serial.print(calculatedOffset[2]); Serial.print("\n");
    Serial.print("gx: ");
    Serial.print(calculatedOffset[3]); Serial.print("\n");
    Serial.print("gy: ");
    Serial.print(calculatedOffset[4]); Serial.print("\n");
    Serial.print("gz: ");
    Serial.print(calculatedOffset[5]); Serial.print("\n"); 
  }
}

// Calculate sensor offsets
void calculateOffsets(){  
  
  // Read raw accel/gyro measurements from device
  accelgyro.getMotion6(&rawValue[0], &rawValue[1], &rawValue[2], &rawValue[3], &rawValue[4], &rawValue[5]);

  // Sum raw sensor values over 30 readings; 30 was chosen so that the total obtained is in the short int (int16_t) range.
  if(counter < 30){
    for(int i = 0; i<6; i++){
      totalRawValue[i] = totalRawValue[i] + rawValue[i];
     }   
      counter ++;
      return calculateOffsets();
  }
    
  if(counter == 30) {
    for(int i = 0; i<6; i++){
      averageRawValue[i] = totalRawValue[i] / counter;    
     }
       // Calculate offsets for each sensor axis: ax, ay, az, gx, gy, gz respectively.
       calculatedOffset[1] = -averageRawValue[0] / 8;
       calculatedOffset[0] = -averageRawValue[1] / 8;
       calculatedOffset[2] = (16384 - averageRawValue[2])/ 8;
       calculatedOffset[3] = -averageRawValue[3] / 4;
       calculatedOffset[4] = -averageRawValue[4] / 4;
       calculatedOffset[5] = -averageRawValue[5] / 4;
  
       isOffsetCalculated = true;
       setOffsetValues();
       return;
    }
}
