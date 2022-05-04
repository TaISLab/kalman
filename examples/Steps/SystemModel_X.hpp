#ifndef KALMAN_EXAMPLES1_LEG_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_LEG_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>
#include <cmath>

namespace KalmanExamples
{
namespace Leg
{

/**
 * @brief System state vector-type for a leg in swinging in 1D (a sine with unknown freq, amp and phase, basically)
 *
 * System is characterized by its swing amplitude, angular speed and phase:
 *            z_k = a_k * sin(2*pi* f_k*t_k + phi) = a_k sin(p_k)
 *                     measurement = z_k  = h(k) 
 *                           state = x_k  = [a_k  f_k  p_k] = f(x_k-1,u_k-1)
 *                         control = u_k  = t_k+1 - t_k
 *                    f(x_k,u_k)   = [a_k  f_k  p_k + 2*pi*u_k]
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
 * This is the system control-input defined by time increment.
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
 * This is the system model defining how our leg changes state with
 * control input, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
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
     * @param [in] x Current system state 
     * @param [in] u Control input
     * @returns The (predicted) system state given control input and states
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_new_;
                
        x_new_.a() = x.a();
        x_new_.f() = x.f();

        // Only absolute phase changes in new state and non-lineally
        auto angle = x.p() + ( dosPi * x.f() * u.dt() );
        angle = fmod(angle, dosPi);
        if ( angle < 0)
            angle += dosPi;
        x_new_.p() = angle;

        // Return transitioned state vector
        return x_new_;
    }

    // just to save obtaining it several times ...
    static constexpr T dosPi = 2.0 * M_PI;
    

protected:
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians( const S& x, const C& u )
    {
        this->F.setZero();
        
        // f(a,f,p) = [f_a, f_f, f_p]
        // F(a,f,p) = [ [df_a/da, df_a/df, df_a/dp ], [df_f/da, df_f/df, df_f/dp ], [df_p/da, df_p/df, df_p/dp ] ]

        this->F( S::A, S::A ) = 1;
        //this->F( S::A, S::F ) = 0;
        //this->F( S::A, S::P ) = 0;

        //this->F( S::F, S::A ) = 0;
        this->F( S::F, S::F ) = 1;
        //this->F( S::F, S::P ) = 0;

        //this->F( S::P, S::A ) = 0;
        this->F( S::P, S::F ) = dosPi * u.dt();
        this->F( S::P, S::P ) = 1;


        // W = df/dw (Jacobian of state transition w.r.t. the noise)

        this->W.setIdentity();
        // TODO: more sophisticated noise modelling
        //       i.e. The noise affects the the direction in which we move as 
        //       well as the velocity (i.e. the distance we move)
    }



};

} // namespace Leg
} // namespace KalmanExamples

#endif
