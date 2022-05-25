
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>
#include <sstream>
#include <string>
#include <fstream>

#include "SystemModel_X.hpp"
#include "PositionMeasurementModel_X.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>
#include <stdio.h>

#include <iostream>
#include <random>
#include <chrono>
#include <cmath>


using namespace KalmanExamples;

typedef float T;

// Some type shortcuts
typedef Leg::State<T> State;
typedef Leg::Control<T> Control;
typedef Leg::SystemModel<T> SystemModel;

typedef Leg::PositionMeasurement<T> PositionMeasurement;
typedef Leg::PositionMeasurementModel<T> PositionModel;


int main(int argc, char** argv)
{
    // input data
    std::ifstream infile("laser_data.csv");
    std::string line; 

    // read variables
    double laser_x, laser_y, dt;
    bool is_valid = false;
 
    // System initial state
    State x0;

    x0.d() = 0.01;   // meters
    x0.a() = 0.15;   // meters
    x0.f() = 0.75;  // hertzs
    x0.p() = 0;     // rads
    
    // Control input
    Control u;

    // System
    SystemModel sys;
    
    // Measurement 
    PositionModel pm;
    
    // Extended Kalman Filter
    Kalman::ExtendedKalmanFilter<State> ekf;

    // Unscented Kalman Filter
    Kalman::UnscentedKalmanFilter<State> ukf(1);

    // Init filter with system state
    ukf.init(x0);       
    ekf.init(x0);

    State x;
    x = sys.f(x0, u);

    // discard first line
    std::getline(infile, line);

    std::cout << "d_e_x" << "," << "a_e_x" << "," << "f_e_x" << "," << "p_e_x" << ","
              << "d_u_x" << "," << "a_u_x" << "," << "f_u_x" << "," << "p_u_x" << ","
              << "z_x"   << "," << "z_e_x" << "," << "z_u_x" << "," << "dt"    << std::endl;

    while (std::getline(infile, line)) {
      is_valid = false;
      std::stringstream ss(line); 
      std::string token;

      std::getline(ss, token, ',');
      dt = std::stod(token);

      std::getline(ss, token, ',');
      if (token.compare("nan")!=0){
        laser_x = std::stod(token);
        is_valid = true;
      }

      std::getline(ss, token, ',');
      if (token.compare("nan")!=0){
        laser_y = std::stod(token);
        // is_valid stays as per previous comparison
      }
      
      // Control input
      u.dt() = dt; //seconds
      
      // Predict state for current time-step using the filters
      auto x_ukf = ukf.predict(sys, u);        
      auto x_ekf = ekf.predict(sys, u);

      // Position measurement
      PositionMeasurement measured_position;
      
      if (is_valid){
        measured_position.pos() = laser_x;
        // Update UKF
        x_ukf = ukf.update(pm, measured_position);    
        // Update EKF
        x_ekf = ekf.update(pm, measured_position);

      } 

      PositionMeasurement estimated_position_u = pm.h(x_ukf);
      PositionMeasurement estimated_position_e = pm.h(x_ekf);

      // Print to stdout as csv format
      std::cout << x_ekf.d() << "," << x_ekf.a() << "," << x_ekf.f() << "," << x_ekf.p() << ","
                << x_ukf.d() << "," << x_ukf.a() << "," << x_ukf.f() << "," << x_ukf.p() << ",";


      // print readings 
      if (is_valid){
        std::cout   << measured_position.pos();
      } else{
        std::cout   << "NaN";
      }
      
      std::cout  <<  "," << estimated_position_e.pos()   << "," << estimated_position_u.pos() << "," << u.dt() << std::endl;
                        
    }
    
    return 0;
}
