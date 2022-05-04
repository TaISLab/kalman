
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>


#include "SystemModel_X.hpp"
#include "PositionMeasurementModel_X.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>
#include <csv.h>
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

  io::CSVReader<3> in("laser_data.csv");
  in.read_header(io::ignore_extra_column, "laser_x", "laser_y", "time" );
  double laser_x, laser_y, time;
  while(in.read_row(laser_x, laser_y, time)){
    // do stuff with the data
    printf ("floats: %4.2f %4.2f %4.8f \n", laser_x, laser_y, time);
  }

  printf ("Done reading");
  return 0;

    // Simulated (true) system initial state
    State x0;
    x0.a() = 0.5;   // meters
    x0.f() = 0.75;  // hertzs
    x0.p() = 0;     // rads
    
    // simulation parameters 
    T dosPi = 2.0 * M_PI;
    // Standard-Deviation of noise added to amplitude during state transition
    T amplitudeNoise = 0.1;
    // Standard-Deviation of noise added to frequency during state transition
    T frequencyNoise = 0.01;
    // Standard-Deviation of noise added to phase during state transition
    T phaseNoise = 0.001;

    // Standard-Deviation of noise added to measurement 
    T measureNoise = 0.1;

    // Control input
    Control u;

    // System
    SystemModel sys;
    
    // Measurement 
    PositionModel pm;
    
    // Random number generation (for noise simulation)
    std::default_random_engine generator;
    generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
    std::normal_distribution<T> noise(0, 1);
    
    // Unscented Kalman Filter
    Kalman::UnscentedKalmanFilter<State> ukf(1);
    
    // Extended Kalman Filter
    Kalman::ExtendedKalmanFilter<State> ekf;

    // Init filter with true system state
    ukf.init(x0);       
    ekf.init(x0);

    State x;
    x = sys.f(x0, u);

    const size_t N = 500;
    const size_t V = 20;

    std::cout   << "a_r" << "," << "f_r" << "," << "p_r" << "," << "z" << ","
                << "a_e" << "," << "f_e" << "," << "p_e" << ","
                << "a_u" << "," << "f_u" << "," << "p_u"
                    << std::endl;

    for(size_t i = 1; i <= N; i++)
    {
        // Control input
        u.dt() = T(V/(N*x0.f())); //seconds
        
        // Simulate system
        x = sys.f(x, u);
        
        // Add noise: Our robot move is affected by noise (due to actuator failures)
        x.a() += amplitudeNoise*noise(generator);
        x.f() += frequencyNoise*noise(generator);
        // wrapping phase ...
        auto angle = x.p() + phaseNoise*noise(generator);
        angle = fmod(angle, dosPi);
        if ( angle < 0)
            angle += dosPi;
        x.p() = angle;


        // Predict state for current time-step using the filters
        auto x_ukf = ukf.predict(sys, u);        
        auto x_ekf = ekf.predict(sys, u);

        // Position measurement
        PositionMeasurement position = pm.h(x);
        
        // Measurement is affected by noise as well
        position.pos() += measureNoise * noise(generator);
                    
        // Update UKF
        x_ukf = ukf.update(pm, position);    
        // Update EKF
        x_ekf = ekf.update(pm, position);

        // Print to stdout as csv format
        std::cout   << x.a()     << "," << x.f()     << "," << x.p() << "," << position.pos() << ","
                    << x_ekf.x() << "," << x_ekf.f() << "," << x_ekf.p()  << ","
                    << x_ukf.a() << "," << x_ukf.f() << "," << x_ukf.p()
                    << std::endl;
    }
    
    return 0;
}
