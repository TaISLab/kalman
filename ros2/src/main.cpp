
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>


#include "SystemModelLeg.hpp"
#include "PositionMeasurementModelLeg.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>
#include <cmath>

typedef float T;

// Some type shortcuts
typedef Leg::State<T> State;
typedef Leg::Control<T> Control;
typedef Leg::SystemModel<T> SystemModel;

typedef Leg::PositionMeasurement<T> PositionMeasurement;
typedef Leg::PositionMeasurementModel<T> PositionModel;


int main(int argc, char** argv)
{
    // Simulated (true) system initial state
    State x0;
    x0.a_x() = 0.5;   // meters
    x0.f_x() = 0.75;  // hertzs
    x0.p_x() = 0;     // rads

    x0.a_y() = 0.1;   // meters
    x0.f_y() = 0.05;  // hertzs
    x0.p_y() = 0;     // rads
    
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

    std::cout   << "a_r_x" << "," << "f_r_x" << "," << "p_r_x" << "," << "z_x" << ","
                << "a_e_x" << "," << "f_e_x" << "," << "p_e_x" << ","
                << "a_u_x" << "," << "f_u_x" << "," << "p_u_x" << ","
                << "a_r_y" << "," << "f_r_y" << "," << "p_r_y" << "," << "z_y" << ","
                << "a_e_y" << "," << "f_e_y" << "," << "p_e_y" << ","
                << "a_u_y" << "," << "f_u_y" << "," << "p_u_y"                
                << std::endl;

    for(size_t i = 1; i <= N; i++)
    {
        // Control input
        u.dt() = T(V/(N*x0.f_x())); //seconds
        
        // Simulate system
        x = sys.f(x, u);
        
        // Add noise: Our robot move is affected by noise (due to actuator failures)
        x.a_x() += amplitudeNoise*noise(generator);
        x.f_x() += frequencyNoise*noise(generator);
        x.a_y() += amplitudeNoise*noise(generator);
        x.f_y() += frequencyNoise*noise(generator);


        // wrapping phase ...
        auto angle = x.p_x() + phaseNoise*noise(generator);
        angle = fmod(angle, dosPi);
        if ( angle < 0)
            angle += dosPi;
        x.p_x() = angle;

        angle = x.p_y() + phaseNoise*noise(generator);
        angle = fmod(angle, dosPi);
        if ( angle < 0)
            angle += dosPi;
        x.p_y() = angle;

        // Predict state for current time-step using the filters
        auto x_ukf = ukf.predict(sys, u);        
        auto x_ekf = ekf.predict(sys, u);

        // Position measurement
        PositionMeasurement position = pm.h(x);
        
        // Measurement is affected by noise as well
        position.pos_x() += measureNoise * noise(generator);
        position.pos_y() += measureNoise * noise(generator);        
                    
        // Update UKF
        x_ukf = ukf.update(pm, position);    
        // Update EKF
        x_ekf = ekf.update(pm, position);

        // Print to stdout as csv format
        std::cout   <<     x.a_x() << "," <<     x.f_x() << "," <<     x.p_x()  << "," << position.pos_x() << ","
                    << x_ekf.a_x() << "," << x_ekf.f_x() << "," << x_ekf.p_x()  << ","
                    << x_ukf.a_x() << "," << x_ukf.f_x() << "," << x_ukf.p_x()  << ","
                    <<     x.a_y() << "," <<     x.f_y() << "," <<     x.p_y()  << "," << position.pos_y() << ","
                    << x_ekf.a_y() << "," << x_ekf.f_y() << "," << x_ekf.p_y()  << ","
                    << x_ukf.a_y() << "," << x_ukf.f_y() << "," << x_ukf.p_y()                    
                    << std::endl;
    }
    
    return 0;
}
