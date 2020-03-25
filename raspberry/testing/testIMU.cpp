#include <cstdio>
#include <cstdlib>
#include "../src/Logger.hpp"
#include "../src/IMU.hpp"

double millis() {
    struct timespec time;
    clock_gettime(CLOCK_REALTIME, &time);
    return (double) (time.tv_sec * 1000.0) + ((double) time.tv_nsec / 1000000.0);
}

int main(int argc, char const *argv[]) {

  printf("Initializing...\n");
  Logger::init(DEBUG);

  IMU imu = IMU();
  imu.init(RANGE_2G, RANGE_245DPS, RANGE_1_3GAUSS,  0.9, 0.2);
  imu.calibrate();
  imu.initAngles();

  double dt;
  int count = 0;
  double time = 0;
  double t;
  bool loop = true;
  IMU::vector<float> angle;

  time = millis();

  while(loop) {
    t = millis();
    dt = (t- time);
    if(dt >= 0.002) {
      count++;
      time = t;

      imu.read();
      imu.processMag();
      imu.processAcc();
      imu.processGyro(dt);
      angle = imu.filter();

      Logger::log(DEBUG, "%f, %f, %f, %f, %f, %f, %f, %f, %f", imu.accAngle.x, imu.accAngle.y, imu.accAngle.z, imu.gyroData.x, imu.gyroData.y, imu.gyroData.z, angle.x, angle.y, angle.z);

      if((count % 20) == 1) {
        Logger::flush();
      }

      if (count >= 30000) {
        loop = false;
      }
    }
  }
  Logger::close();
  printf("Exiting...\n");
  return 0;
}
