#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define SERIAL_DIGITS 4

#define SERVOPIN 9

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo servo;
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails() {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);

  
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("WebSerial 3D Firmware"); Serial.println("");

    imuINIT()
    servoINIT()
}


void loop() {
    receiveMessage();
}






// ------- COMM FUNCTIONS -----------
void receiveMessage() {
    while (Serial.available() != 0) {
        int c = Serial.read();
        Serial.println(c);
        if (c == int('c')) {
            servoClose();
        } else if (c == int('o')) {
            servoOpen();
        }
    }
}

void blink(int n, int d) {
    for (int i = 0; i < n; i++) {
        digitalWrite(13,HIGH);
        delay(d);
        digitalWrite(13,LOW);
        delay(d);
    }
}
// ------- SERVO FUNCTIONS ----------
void servoINIT() {
    servo.attach(SERVOPIN);
    pinMode(SERVOPIN, OUTPUT);
}

void servoOpen() {
    servo.write(opendegree);
    blink(10, 100);
}

void servoClose() {
    servo.write(closedegree);
    blink(5, 200);
}




// -------- IMU FUNCTIONS -----------
void imuINIT() {
    /* Initialise the sensor */
    if(!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    
    delay(1000);

    /* Use external crystal for better accuracy */
    bno.setExtCrystalUse(true);
    

    // function headers to do and set calibration
    // (see harrison's scripts for how to use these)
    // bool getSensorOffsets(uint8_t *calibData);
    // bool getSensorOffsets(adafruit_bno055_offsets_t &offsets_type);
    // void setSensorOffsets(const uint8_t *calibData);
    // void setSensorOffsets(const adafruit_bno055_offsets_t &offsets_type);


    /* Display some basic information on this sensor */
    displaySensorDetails();
}


void sendIMUData() {
    // /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);
    // event.orientation.x,y,z is orientation in euler angles
    // event.acceleration.x,y,z is acceleration
    // event.magnetic.x,y,z is magnetic field
    // event.gyro.x,y,z is gyroscopic measurements in euler angles
    
    // bno.getQuat() is orientation in quaternion

    // bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER)
    // bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE)
    // bno.getVector(Adafruit_BNO055::VECTOR_EULER)
    // bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER) is total acceleration. frame?
    // bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL) is acceleration minus gravity. frame?
    // bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY) is experienced gravity. in imu frame

    // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    // bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    // bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    // bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    // /* The WebSerial 3D Model Viewer expects data as heading, pitch, roll */
    // Serial.print(F("Orientation: "));
    // Serial.print(360 - (float)event.orientation.x);
    // Serial.print(F(", "));
    // Serial.print((float)event.orientation.y);
    // Serial.print(F(", "));
    // Serial.print((float)event.orientation.z);
    // Serial.println(F(""));

    // imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);


    imu::Quaternion quat = bno.getQuat();
    sendQuat(quat);
    sendCalibrationStatus();
}



void sendAcceleration(sensors_event_t event) {

}
void sendOrientation(sensors_event_t event) {

}


void sendQuaternion(imu::Quaternion quat) {
    Serial.print(F("Quaternion: "));
    Serial.print((float)quat.w(), SERIAL_DIGITS);
    Serial.print(F(", "));
    Serial.print((float)quat.x(), SERIAL_DIGITS);
    Serial.print(F(", "));
    Serial.print((float)quat.y(), SERIAL_DIGITS);
    Serial.print(F(", "));
    Serial.print((float)quat.z(), SERIAL_DIGITS);
    Serial.println(F(""));
}

void sendCalibrationStatus() {
    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print(F("Calibration: "));
    Serial.print(sys, DEC);
    Serial.print(F(", "));
    Serial.print(gyro, DEC);
    Serial.print(F(", "));
    Serial.print(accel, DEC);
    Serial.print(F(", "));
    Serial.print(mag, DEC);
    Serial.println(F(""));
}