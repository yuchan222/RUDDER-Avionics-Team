#include "SensorManager.h"

Adafruit_BMP3XX bmp;
MPU9250 imu;
MPU9250Setting setting;

#define QUAT_FIX 10000

void initSensors() {
  Wire.end();
  delay(10);
  Wire.begin();
  // BMP388 설정 
  if (!bmp.begin_I2C(0x76)) {
    Serial.println("BMP388 fail");
    //while (1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  //bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  //BMP3_IIR_FILTER_DISABLE
  // MPU9250 설정
  setting.accel_fs_sel = ACCEL_FS_SEL::A8G;
  setting.gyro_fs_sel  = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;

  if (!imu.setup(0x68, setting)) {
    Serial.println("MPU9250 init fail");
    //while (1);
  }
  //imu.calibrateAccelGyro();
  //imu.calibrateMag();
  imu.setAccBias(440, 300, 0);   // 직접 지정
  imu.setGyroBias(185, 63, 0); //500DPS (90, 30, 10) // 1000DPS

  imu.selectFilter(QuatFilterSel::MAHONY);
  imu.setFilterIterations(30);
  Serial.println("Init Sensor Done");
}

void readSensors(DataPacket &p) {
  imu.update();
  p.acc[0] = imu.getAccX() * 1000 ;
  p.acc[1] = imu.getAccY() * 1000 ;
  p.acc[2] = imu.getAccZ() * 1000 ;
  p.gyro[0] = imu.getGyroX() * 100 ;
  p.gyro[1] = imu.getGyroY() * 100 ;
  p.gyro[2] = imu.getGyroZ() * 100 ;
  p.mag[0] = imu.getMagX() * 1000 ;
  p.mag[1] = imu.getMagY() * 1000;
  p.mag[2] = imu.getMagZ() * 1000;
  p.quat[0] = imu.getQuaternionW() * QUAT_FIX;
  p.quat[1] = imu.getQuaternionX() * QUAT_FIX;
  p.quat[2] = imu.getQuaternionY() * QUAT_FIX;
  p.quat[3] = imu.getQuaternionZ() * QUAT_FIX;

  if (bmp.performReading()) {
    p.pressure = bmp.pressure;
    p.bmp_temp = bmp.temperature *100 ;
  } else {
    p.pressure = -1;
    p.bmp_temp = -1;
  }
}

void clearI2CBus(uint8_t sdaPin, uint8_t sclPin) {
  pinMode(sdaPin, OUTPUT);
  pinMode(sclPin, OUTPUT);
  digitalWrite(sdaPin, HIGH);

  for (int i = 0; i < 9; i++) {
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(sclPin, LOW);
    delayMicroseconds(5);
  }

  digitalWrite(sclPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(sdaPin, HIGH);
  delayMicroseconds(5);
}

