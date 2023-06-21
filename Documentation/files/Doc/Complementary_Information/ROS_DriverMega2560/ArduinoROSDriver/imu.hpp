
#include<Wire.h>

constexpr uint8_t kAddrAccel = 0x18;
constexpr uint8_t kAddrGyro = 0x68;

constexpr float kPI = 3.1415;
constexpr float kDeg2Rad = kPI / 180.0;


class BMX055{
  private:
    void InitAccel() {
        // [-2,+2] [g]
        Send(kAddrAccel, 0x0F, 0x03);
        Send(kAddrAccel, 0x10, 0x08);
        Send(kAddrAccel, 0x11, 0x00);
    }

    void InitGyro() {
        // [-125,+125] [deg/s]
        Send(kAddrGyro, 0x0F, 0x04);
        Send(kAddrGyro, 0x10, 0x07);
        Send(kAddrGyro, 0x11, 0x00);
    }


    void Read(int addr, uint8_t start_reg, uint8_t data[6]) {
        for (int i = 0; i < 6; i++) {
            Wire.beginTransmission(addr);
            Wire.write((start_reg + i));
            Wire.endTransmission();
            Wire.requestFrom(addr, 1);
            if (Wire.available() == 1) {
                data[i] = Wire.read();
            }
        }
    }

    float AccelToFloat(uint8_t data[2]) {
        int val = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
        if (val > 2047) {
            val -= 4096;
        }
        return val * 0.0098; 
    }
    float GyroToFloat(uint8_t data[2]) {
  int val = ((data[1] * 256) + data[0]);
  if (val > 32767) {
    val -= 65536;
  }
  // [-125, +125] [degree/s] --> [rad/s]
  return (val * 0.0038) * kDeg2Rad;
}

    void Send(uint8_t addr, uint8_t reg, uint8_t val) {
          Wire.beginTransmission(addr);
          Wire.write(reg);
          Wire.write(val);
          Wire.endTransmission();
          delay(100);
    }

    public:
    void SetupDevice() {
      Wire.begin();
        InitAccel();
        InitGyro();
        
    }
    void getGyro(float &gx,float &gy,float &gz){
      uint8_t data[6];
      Read(kAddrAccel, 2, data);
      gx=GyroToFloat(&data[0]);
      gy=GyroToFloat(&data[2]);
      gz=GyroToFloat(&data[4]);
    }

    void getAcc(float &ax,float &ay,float &az){
      uint8_t data[6];
      Read(kAddrGyro, 2, data);
      ax = AccelToFloat(&data[0]);
      ay = AccelToFloat(&data[2]);
      az = AccelToFloat(&data[4]);
    }
};
