#include <cstdio>
#include <cstdlib>
#include "../src/Logger.hpp"
#include "../src/IMU.hpp"

#define INTERVAL_MSEC			2
#define RUNTIME_SEC			600
#define RUNTIME				RUNTIME_SEC * 1000 / INTERVAL_MSEC

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
  Logger::log(DEBUG, "AccAngle x, AccAngle y, AccAngle z, Angle x, Angle y, Angle z, AccData x, AccData y, AccData z, GyroData x, GyroData y, GyroData z, MagData x, MagData y, MagData z");
  double dt = 0;
  int count = 0;
  double time = 0;
  double t0 = millis();
  double t;
  bool loop = true;
  IMU::vector<float> angle;
  printf("You have %d seconds to move the Quad to 6 positions, "
          "so that all axes are exposed to positive and negative gravity.\n"
          "Waiting for clearance... (ENTER)",
         RUNTIME_SEC);
  getchar();

  time = millis();

  while(loop) {
    t = millis();
    dt = (t- time);
    if(dt >= INTERVAL_MSEC) {
      count++;
      time = t;

      imu.read();
      imu.processMag();
      imu.processAcc();
      imu.processGyro(dt);
      angle = imu.filter();

      // Logger::log(DEBUG, "%f, %f, %f, %f, %f, %f", imu.accData.x, imu.accData.y, imu.accData.z, imu.gyroData.x, imu.gyroData.y, imu.gyroData.z);
      Logger::log(DEBUG, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
                  imu.accAngle.x, imu.accAngle.y, imu.accAngle.z,
                  angle.x, angle.y, angle.z,
                  imu.accData.x, imu.accData.y, imu.accData.z,
                  imu.gyroData.x, imu.gyroData.y, imu.gyroData.z,
                  imu.magData.x, imu.magData.y, imu.magData.z);
      if((count % 20) == 1) {
        Logger::flush();
      }

      if (count >= RUNTIME) {
        loop = false;
      }
    }
  }
  Logger::close();
  printf("Exiting...\nLoop took %f seconds, should have taken %d seconds", (millis() - t0)/1000, RUNTIME_SEC);
  return 0;
}
