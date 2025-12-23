#include <Eigen/Dense>
#include <eskf.h>

#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);

  EsEkf ekf;

  ekf.initialize(Eigen::Vector3d(0,0,0),
                 Eigen::Vector3d(0,0,0),
                 Eigen::Quaterniond(1,0,0,0),
                 Eigen::Vector3d(0,0,0),
                 Eigen::Vector3d(0,0,0),
                 0.0,
                 Eigen::Matrix<double, DIM_ERROR, DIM_ERROR>::Identity() * 0.1);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}