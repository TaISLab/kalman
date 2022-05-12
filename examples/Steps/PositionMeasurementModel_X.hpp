#ifndef KALMAN_EXAMPLES_LEG_POSITIONMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_LEG_POSITIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include <cmath>

namespace KalmanExamples
{
namespace Leg
{

/**
 * @brief Measurement vector measuring leg position
 *
 * @param T Numeric scalar type
 */
template<typename T>
class PositionMeasurement : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(PositionMeasurement, T, 1)
    
    //! current measured position
    static constexpr size_t POS = 0;
        
    T  pos()       const { return (*this)[ POS ]; }
    T& pos()      { return (*this)[ POS ]; }
};

/**
 * @brief Measurement model for measuring the position of the robot
 *        using two beacon-landmarks
 *
 * This is the measurement model for measuring the position of the robot.
 * The measurement is given by two landmarks in the space, whose positions are known.
 * The robot can measure the direct distance to both the landmarks, for instance
 * through visual localization techniques.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class PositionMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, PositionMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  KalmanExamples::Leg::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  KalmanExamples::Leg::PositionMeasurement<T> M;
    
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;        
        measurement.pos() = x.d() + x.a() * std::sin(x.p());
      
        return measurement;
    }

protected:
    
    static constexpr T dosPi = 2.0 * M_PI;
    
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear measurement function \f$h(x)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize 
     */
    void updateJacobians( const S& x )
    {
        // H = d/ds * h(s) = [dh/da, dh/df, dh/dp, dh/dd] (Jacobian of measurement function w.r.t. the state)
        this->H.setZero();
        
        this->H( M::POS, S::A ) = std::sin(x.p());
        this->H( M::POS, S::F ) = dosPi * x.a() * std::cos(x.p()) ;
        this->H( M::POS, S::P ) = x.a() * std::cos(x.p());
        this->H( M::POS, S::D ) = 1;
        
    }    

};

} // namespace Leg
} // namespace KalmanExamples

#endif