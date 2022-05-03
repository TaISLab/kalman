#ifndef KALMAN_EXAMPLES1_LEG_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_LEG_SYSTEMMODEL_HPP_

#include <kalman/SystemModel.hpp>
#include <cmath>

namespace KalmanExamples
{
namespace Leg
{

/**
 * @brief System state vector-type for a leg in swinging in 1D (a sine, basically)
 *
 * State is characterized by its swing amplitude, angular speed and phase:
 *          z_k = a_k * sin(2*pi* f_k*t_k + phi) = a_k sin(p_k)
 *                        state = [a_k  f_k  p_k]
 *                        control = t_k
 *                        measurement = z_k    
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(State, T, 3)
    
    //! Amplitude
    static constexpr size_t A = 0;
    //! Frequency
    static constexpr size_t F = 1;
    //! Absolute phase
    static constexpr size_t P = 2;
    
    T a()       const { return (*this)[ A ]; }
    T f()       const { return (*this)[ F ]; }
    T p()       const { return (*this)[ P ]; }
    
    T& a()      { return (*this)[ A ]; }
    T& f()      { return (*this)[ F ]; }
    T& p()      { return (*this)[ P ]; }    

};

/**
 * @brief System control-input vector-type for a leg in 1D
 *
 * This is the system control-input defined by time.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(Control, T, 1)
    
    //! time 
    static constexpr size_t DT = 0;
    
    T  dt()  const { return (*this)[ DT ]; }    
    T& dt() { return (*this)[ DT ]; }
};

/**
 * @brief System model for a leg swinging in 1D
 *
 * This is the system model defining how our leg moves from one 
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::SystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef KalmanExamples::Leg::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanExamples::Leg::Control<T> C;
    
    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_new_;
                
        x_new_.a() = x.a();
        x_new_.f() = x.f();

        // Only absolute phase changes in new state
        auto angle = x.p() + ( dosPi * x.f() * u.dt() );
        angle = fmod(angle, dosPi);
        if ( angle < 0)
            angle += dosPi;
        x_new_.p() = angle;

        // Return transitioned state vector
        return x_new_;
    }

    static constexpr T dosPi = 2.0 * M_PI;
    
};

} // namespace Leg
} // namespace KalmanExamples

#endif