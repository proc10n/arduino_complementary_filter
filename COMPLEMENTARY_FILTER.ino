#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Scale
int scale;


// Gyroscope and accelerometer variables
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t accX_prev, accY_prev, accZ_prev; // Previous readings
int16_t gyroX_prev, gyroY_prev, gyroZ_prev;
int16_t accX_f, accY_f, accZ_f; // Filtered accelerometer readings
int16_t gyroX_f, gyroY_f, gyroZ_f; // Filtered gyroscope readings


// Initial estimates (Sensor flat on a surface!)
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

// Constants
float g = 9.81;           // Gravity 
float alpha = 0.92;       // Complementary filter (Adjust based on test)
float lpf = 0.3;
unsigned long prevTime;  

void setup() {
    Serial.begin(9600);  // Initialize serial communication
    Wire.begin();        // Initialize I2C communication

    int scale = 60;

    // Initialize MPU6050
    Serial.println("Initializing MPU.");
    mpu.initialize();
    
    // Check MPU6050 connection
    Serial.println("Testing connection.");
    if (!mpu.testConnection()) {
        Serial.println("Connection failed.");
        while (true);  // Stop the program if connection fails
    } else {
        Serial.println("Connection succesful.");
    }

     //**Set Offsets Here*************
     mpu.setXAccelOffset(-677.0); //Set your accelerometer offset for axis X
     mpu.setYAccelOffset(-3957.0); //Set your accelerometer offset for axis Y
     mpu.setZAccelOffset(941.0); //Set your accelerometer offset for axis Z
     mpu.setXGyroOffset(134.0);  //Set your gyro offset for axis X
     mpu.setYGyroOffset(-4.0);  //Set your gyro offset for axis Y
     mpu.setZGyroOffset(70);  //Set your gyro offset for axis Z  
     //*******************************

     
   prevTime = millis();  // Initialize previous time for delta time calculation
}

void loop() {
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;  // Calculate delta time in seconds
    prevTime = currentTime;
    gyroX_prev = gyroX, gyroY_prev = gyroY, gyroZ_prev = gyroZ;
    accX_prev = accX, accY_prev = accY, accZ_prev = accZ;


    // Update sensor data
    mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

    // Filtering values with LPF
    gyroX = lpf * gyroX + (1 - lpf) * gyroX_prev;
    gyroY = lpf * gyroY + (1 - lpf) * gyroY_prev;
    gyroZ = lpf * gyroZ + (1 - lpf) * gyroZ_prev;

    accX = lpf * accX + (1 - lpf) * accX_prev;
    accY = lpf * accY + (1 - lpf) * accY_prev;
    accZ = lpf * accZ + (1 - lpf) * accZ_prev;


    // Converte accel to angle estimate in rad (Adjust based on sensitivity!)
    float accXf = accX / 16384.0; 
    float accYf = accY / 16384.0;
    float accZf = accZ / 16384.0;

    float roll_acc = atan2(accYf, accZf);
    float pitch_acc = atan(-accXf / sqrt(accYf * accYf + accZf * accZf));

    // Convert gyroscope values to degrees per second (Adjust based on range!)
    float gyroX_dps = gyroX / 131.0;
    float gyroY_dps = gyroY / 131.0;
    float gyroZ_dps = gyroZ / 131.0;

    // Integrate gyroscope data 
    roll += gyroX_dps * dt;
    pitch += gyroY_dps * dt;
    yaw += gyroZ_dps * dt;

    // Complementary filter + conversion do degrees
    roll = alpha * roll + (1 - alpha) * roll_acc * 180 / PI;
    pitch = alpha * pitch + (1 - alpha) * pitch_acc * 180 / PI;

    // Print euler angle estimates
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" | Pitch: ");
    Serial.print(pitch);
    Serial.print(" | Yaw: ");
    Serial.print(yaw);
    Serial.println(scale);

    // Sample Time
    delay(100);
}
