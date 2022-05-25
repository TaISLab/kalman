
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>


#include "SystemModel.hpp"
#include "ForceMeasurementModel.hpp"
#include "SpeedMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>
#include <cmath>

typedef float T;

// Some type shortcuts
typedef KalmanExamples::Step::State<T> State;
typedef KalmanExamples::Step::Control<T> Control;
typedef KalmanExamples::Step::SystemModel<T> SystemModel;
typedef KalmanExamples::Step::ForceMeasurement<T> ForceMeasurement;
typedef KalmanExamples::Step::ForceMeasurementModel<T> ForceModel;
typedef KalmanExamples::Step::SpeedMeasurement<T> SpeedMeasurement;
typedef KalmanExamples::Step::SpeedMeasurementModel<T> SpeedModel;


int main(int argc, char** argv)
{
    // Simulated (true) system initial state
    State x0;
    
    x0.v0()  = 0.01;   // meters/s
    x0.v1()  = 1.0;    // meters/s
    x0.f0()  = 4.0;    // Newton
    x0.f1()  = 15.0;   // Newton
    x0.w()   = 0.1;    // rads/s
    x0.d()   = 0.50;   // rads
    x0.vp()  = 0.20;   // rads
    
    // simulation parameters 
    T dosPi = 2.0 * M_PI;

    // Standard-Deviation of noises added to state vars during transition
    T v0_noise = 0.1;
    T v1_noise = 0.1;
    T f0_noise = 0.1;
    T f1_noise = 0.1;
    T w_noise  = 0.01;
    T d_noise  = 0.001;
    T vp_noise = 0.001;

    // Standard-Deviation of noise added to measurements
    T dv_measureNoise = 0.1;
    T df_measureNoise = 0.1;

    // Control input
    Control u;
    u.dt() = 0.001;

    // System
    SystemModel sys;
    
    // Measurements
    ForceModel fm;
    SpeedModel sm;

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

    const size_t N = 500;  // number of samples
    const size_t V = 20;  // number of simulated periods
    // Uniform Control inputs (could be non-uniform ...)
    u.dt() = T(V/(N*x0.w()/dosPi)); //seconds

    std::cout   << "t_v0" << ","  << "t_v1" << ","  << "t_f0" << ","  << "t_f1" << ","  << "t_w"  << ","  << "t_d"  << ","  << "t_vp" << ","            // STATE (simulated)
                << "dv"   << ","  << "df"   << ","                                                                                                           // Measurements (speed and force)
                << "e_v0" << ","  << "e_v1" << ","  << "e_f0" << ","  << "e_f1" << ","  << "e_w"  << ","  << "e_d"  << ","  << "e_vp" << ","           // Filter predicted states ( extended, unscented)
                << "u_v0" << ","  << "u_v1" << ","  << "u_f0" << ","  << "u_f1" << ","  << "u_w"  << ","  << "u_d"  << ","  << "u_vp"
                << std::endl;

    for(size_t i = 1; i <= N; i++)
    {
        
        // Simulate system ............................................
        //std::cout   << "Calling sys" << std::endl;        
        x = sys.f(x, u);
        
        // Add noise to state
        x.v0()  += v0_noise * noise(generator);
        x.v1()  += v1_noise * noise(generator);
        x.f0()  += f0_noise * noise(generator);
        x.f1()  += f1_noise * noise(generator);
        x.w()   += w_noise  * noise(generator);
        x.d()   += d_noise  * noise(generator);
        x.vp()  += vp_noise * noise(generator);

        // wrapping phases ...
        x.d() = fmod(x.d(), dosPi);
        if ( x.d() < 0)
            x.d() += dosPi;

        x.vp() = fmod(x.vp(), dosPi);
        if ( x.vp() < 0)
            x.vp() += dosPi;

        // Predict state for current time-step using the filters
        //std::cout   << "Calling predict" << std::endl;        
        auto x_ukf = ukf.predict(sys, u);        
        //std::cout   << "Calling predict" << std::endl;        
        auto x_ekf = ekf.predict(sys, u);

        // Measurements        
        SpeedMeasurement speed = sm.h(x);
        ForceMeasurement force = fm.h(x);

        // Add noise to measurements
        speed.dv() += dv_measureNoise * noise(generator);
        force.df() += df_measureNoise * noise(generator);
                    
        // Update filters with measurements
        // unscented
        //std::cout   << "Calling update" << std::endl;        
        x_ukf = ukf.update(sm, speed);
        //std::cout   << "Calling update" << std::endl;                
        x_ukf = ukf.update(fm, force);
        // extended
        //std::cout   << "Calling update" << std::endl;                
        x_ekf = ekf.update(sm, speed);
        //std::cout   << "Calling update" << std::endl;                
        x_ekf = ekf.update(fm, force);


        // Print to stdout as csv format
        std::cout << x.v0()     << "," << x.v1()     << "," << x.f0()     << "," << x.f1()     << "," << x.w()      << "," << x.d()      << "," << x.vp()     << ","  // STATE (simulated)
                  << speed.dv() << "," << force.df() << ","                                                                                                           // Measurements (speed and force)
                  << x_ekf.v0() << "," << x_ekf.v1() << "," << x_ekf.f0() << "," << x_ekf.f1() << "," << x_ekf.w()  << "," << x_ekf.d()  << "," << x_ekf.vp() << ","  // Filter predicted states ( extended, unscented)
                  << x_ukf.v0() << "," << x_ukf.v1() << "," << x_ukf.f0() << "," << x_ukf.f1() << "," << x_ukf.w()  << "," << x_ukf.d()  << "," << x_ukf.vp()            
                  << std::endl;
    
    }
    
    return 0;
}
