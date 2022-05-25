// The MIT License (MIT)
//
// Copyright (c) 2015 Markus Herb
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#ifndef KALMAN_UNSCENTEDKALMANFILTER_HPP_
#define KALMAN_UNSCENTEDKALMANFILTER_HPP_

#include "UnscentedKalmanFilterBase.hpp"
#include "StandardFilterBase.hpp"

#include <iostream>
#include <eigen3/Eigen/Eigenvalues> 
#include <eigen3/Eigen/Cholesky>

namespace Kalman {
    
    /**
     * @brief Unscented Kalman Filter (UKF)
     *
     * @note It is recommended to use the square-root implementation SquareRootUnscentedKalmanFilter of this filter
     *
     * This implementation is based upon [The square-root unscented Kalman filter for state and parameter-estimation](http://dx.doi.org/10.1109/ICASSP.2001.940586) by Rudolph van der Merwe and Eric A. Wan.
     * Whenever "the paper" is referenced within this file then this paper is meant.
     * 
     * @param StateType The vector-type of the system state (usually some type derived from Kalman::Vector)
     */
    template<class StateType>
    class UnscentedKalmanFilter : public UnscentedKalmanFilterBase<StateType>,
                                  public StandardFilterBase<StateType>
    {
    public:
        //! Unscented Kalman Filter base type
        typedef UnscentedKalmanFilterBase<StateType> UnscentedBase;
        
        //! Standard Filter base type
        typedef StandardFilterBase<StateType> StandardBase;
        
        //! Numeric Scalar Type inherited from base
        using typename UnscentedBase::T;
        
        //! State Type inherited from base
        using typename UnscentedBase::State;
        
        //! Measurement Model Type
        template<class Measurement, template<class> class CovarianceBase>
        using MeasurementModelType = typename UnscentedBase::template MeasurementModelType<Measurement, CovarianceBase>;

        //! System Model Type
        template<class Control, template<class> class CovarianceBase>
        using SystemModelType = typename UnscentedBase::template SystemModelType<Control, CovarianceBase>;
        
    protected:
        //! The number of sigma points (depending on state dimensionality)
        using UnscentedBase::SigmaPointCount;
        
        //! Matrix type containing the sigma state or measurement points
        template<class Type>
        using SigmaPoints = typename UnscentedBase::template SigmaPoints<Type>;
        
        //! Kalman Gain Matrix Type
        template<class Measurement>
        using KalmanGain = Kalman::KalmanGain<State, Measurement>;
        
    protected:
        // Member variables
        
        //! State Estimate
        using UnscentedBase::x;
        
        //! State Covariance
        using StandardBase::P;
        
        //! Sigma points (state)
        using UnscentedBase::sigmaStatePoints;
        
    public:
        /**
         * Constructor
         * 
         * See paper for detailed parameter explanation
         * 
         * @param alpha Scaling parameter for spread of sigma points (usually \f$ 1E-4 \leq \alpha \leq 1 \f$)
         * @param beta Parameter for prior knowledge about the distribution (\f$ \beta = 2 \f$ is optimal for Gaussian)
         * @param kappa Secondary scaling parameter (usually 0)
         */
        UnscentedKalmanFilter(T alpha = T(1), T beta = T(2), T kappa = T(0))
            : UnscentedKalmanFilterBase<StateType>(alpha, beta, kappa)
        {
            // Init covariance to identity
            P.setIdentity();
        }
       
        /**
         * @brief Perform filter prediction step using system model and no control input (i.e. \f$ u = 0 \f$)
         *
         * @param [in] s The System model
         * @return The updated state estimate
         */
        template<class Control, template<class> class CovarianceBase>
        const State& predict( const SystemModelType<Control, CovarianceBase>& s )
        {
            // predict state (without control)
            Control u;
            u.setZero();
            return predict( s, u );
        }
        
        /**
         * @brief Perform filter prediction step using control input \f$u\f$ and corresponding system model
         *
         * @param [in] s The System model
         * @param [in] u The Control input vector
         * @return The updated state estimate
         */
        template<class Control, template<class> class CovarianceBase>
        const State& predict( const SystemModelType<Control, CovarianceBase>& s, const Control& u )
        {
            // Compute sigma points
            if(!computeSigmaPoints())
            {
                //std::cout << "Can't compute sigma points for prediction in unscented kalman filter" << std::endl;
                // A well known trick of forcing to be zero the negative eigenvalues, as suggested in
                //           GREWAL, M. S.; ANDREWS, A. P. Kalman Filtering: Theory and Practice: Using Matlab. New York, NY: John Wiley & Sons, 2001. 401 p. ISBN 0471392545.
                //std::cout << "covariance" << std::endl;                
                //std::cout << P << std::endl;
                Eigen::EigenSolver<SquareMatrix<T, State::RowsAtCompileTime>> eigensolver(P);
                Eigen::Matrix<std::complex<T>, State::RowsAtCompileTime, 1> eival = eigensolver.eigenvalues();                
                Eigen::Matrix<std::complex<T>, State::RowsAtCompileTime, State::RowsAtCompileTime> eivec = eigensolver.eigenvectors();
                Eigen::Matrix<std::complex<T>, State::RowsAtCompileTime, State::RowsAtCompileTime> eivecT = eivec.adjoint();

                //std::cout << "Current eigenvalues" << std::endl;
                //std::cout << eival << std::endl;
                //std::cout << "Current eigenvectors" << std::endl;
                //std::cout << eivec << std::endl;

                // "correct" negative eigenvalues
                for (int i = 0; i < eival.size(); ++i) {
                    if (eival(i).real()<0)
                        eival(i) =1e-6; // stupid numeric errors ...
                }
                Eigen::Matrix<std::complex<T>, State::RowsAtCompileTime, State::RowsAtCompileTime> eivalM = eival.matrix().asDiagonal();                
                Eigen::Matrix<float, State::RowsAtCompileTime, State::RowsAtCompileTime> P2 = (eivec * eivalM *eivecT).real();

                //std::cout << "New covariance" << std::endl;
                //std::cout << P2 << std::endl;

                //eigensolver.compute(P2);
                //eival = eigensolver.eigenvalues();
                //eivec = eigensolver.eigenvectors();
                //std::cout << "new eigenvalues" << std::endl;
                //std::cout << eival << std::endl;
                //std::cout << "new eigenvectors" << std::endl;
                //std::cout << eivec << std::endl;

                P = P2;
                if(!computeSigmaPoints()){
                    std::cout << "Cholseky falla again" << std::endl;
                    assert(false);
                }

                //Eigen::LDLT<SquareMatrix<T, State::RowsAtCompileTime>> ldlt;
                //ldlt.compute(P);
                //if(ldlt.info() != Eigen::Success){
                //    std::cout << "ldlt fallo" << std::endl;
                //}else {
                    //std::cout << "ldlt conseguida:" << std::endl;
                    //Eigen::Transpositions<State::RowsAtCompileTime,State::RowsAtCompileTime,int> P = ldlt.transpositionsP();
                    //Eigen::Transpose<Eigen::TranspositionsBase<Eigen::Transpositions<State::RowsAtCompileTime, State::RowsAtCompileTime, int> > > Pt = P.inverse();
                    //SquareMatrix<T, State::RowsAtCompileTime> L = ldlt.matrixL();
                    //SquareMatrix<T, State::RowsAtCompileTime> D = ldlt.vectorD().asDiagonal();
                    //SquareMatrix<T, State::RowsAtCompileTime> Lt = ldlt.matrixL().adjoint();
                    //std::cout << "L:" << std::endl;
                    //std::cout << L << std::endl<< std::endl;
                    //std::cout << "D:" << std::endl;
                    //Eigen::Diagonal<const Eigen::Matrix<T, State::RowsAtCompileTime, State::RowsAtCompileTime>, 0> vD = ldlt.vectorD();
                    //for (int i = 0; i < vD.size(); ++i) {
                    //    std::cout << vD(i) << std::endl;
                    //}
                    //std::cout << D << std::endl<< std::endl;
                    //std::cout << "Cov == Pt*L*D*Lt*Pt " << std::endl;
                    //std::cout << (Pt*L*D*Lt*Pt)<< std::endl<< std::endl;

                
                //} 

            }
            
            // Compute predicted state
            x = this->template computeStatePrediction<Control, CovarianceBase>(s, u);
            
            // Compute predicted covariance
            computeCovarianceFromSigmaPoints(x, sigmaStatePoints, s.getCovariance(), P);
            
            // Return predicted state
            return this->getState();
        }
        
        /**
         * @brief Perform filter update step using measurement \f$z\f$ and corresponding measurement model
         *
         * @param [in] m The Measurement model
         * @param [in] z The measurement vector
         * @return The updated state estimate
         */
        template<class Measurement, template<class> class CovarianceBase>
        const State& update( const MeasurementModelType<Measurement, CovarianceBase>& m, const Measurement& z )
        {
            SigmaPoints<Measurement> sigmaMeasurementPoints;
            
            // Predict measurement (and corresponding sigma points)
            Measurement y = this->template computeMeasurementPrediction<Measurement, CovarianceBase>(m, sigmaMeasurementPoints);
            
            // Compute innovation covariance
            Covariance<Measurement> P_yy;
            computeCovarianceFromSigmaPoints(y, sigmaMeasurementPoints, m.getCovariance(), P_yy);
            
            KalmanGain<Measurement> K;
            computeKalmanGain(y, sigmaMeasurementPoints, P_yy, K);
            
            // Update state
            x += K * ( z - y );
            
            // Update state covariance
            updateStateCovariance<Measurement>(K, P_yy);
            
            return this->getState();
        }
        
    protected:
        /**
         * @brief Compute sigma points from current state estimate and state covariance
         * 
         * @note This covers equations (6) and (9) of Algorithm 2.1 in the Paper
         */
        bool computeSigmaPoints()
        {
            // Get square root of covariance
            CovarianceSquareRoot<State> llt;
            llt.compute(P);
            if(llt.info() != Eigen::Success)
            {
                return false;
            }
            
            SquareMatrix<T, State::RowsAtCompileTime> _S = llt.matrixL().toDenseMatrix();
            
            // Set left "block" (first column)
            sigmaStatePoints.template leftCols<1>() = x;
            // Set center block with x + gamma * S
            sigmaStatePoints.template block<State::RowsAtCompileTime, State::RowsAtCompileTime>(0,1)
                    = ( this->gamma * _S).colwise() + x;
            // Set right block with x - gamma * S
            sigmaStatePoints.template rightCols<State::RowsAtCompileTime>()
                    = (-this->gamma * _S).colwise() + x;
            
            return true;
        }
        
        /**
         * @brief Compute the Covariance from sigma points and noise covariance
         * 
         * @param [in] mean The mean predicted state or measurement
         * @param [in] sigmaPoints the predicted sigma state or measurement points
         * @param [in] noiseCov The system or measurement noise covariance
         * @param [out] cov The propagated state or innovation covariance
         *
         * @return True on success, false if a numerical error is encountered when updating the matrix
         */
        template<class Type>
        bool computeCovarianceFromSigmaPoints( const Type& mean, const SigmaPoints<Type>& sigmaPoints, 
                                               const Covariance<Type>& noiseCov, Covariance<Type>& cov) const
        {
            decltype(sigmaPoints) W = this->sigmaWeights_c.transpose().template replicate<Type::RowsAtCompileTime,1>();
            decltype(sigmaPoints) tmp = (sigmaPoints.colwise() - mean);
            cov = tmp.cwiseProduct(W) * tmp.transpose() + noiseCov;
            
            return true;
        }
        
        /**
         * @brief Compute the Kalman Gain from predicted measurement and sigma points and the innovation covariance.
         * 
         * @note This covers equations (11) and (12) of Algorithm 2.1 in the Paper
         *
         * @param [in] y The predicted measurement
         * @param [in] sigmaMeasurementPoints The predicted sigma measurement points
         * @param [in] P_yy The innovation covariance
         * @param [out] K The computed Kalman Gain matrix \f$ K \f$
         */
        template<class Measurement>
        bool computeKalmanGain( const Measurement& y,
                                const SigmaPoints<Measurement>& sigmaMeasurementPoints,
                                const Covariance<Measurement>& P_yy,
                                KalmanGain<Measurement>& K) const
        {
            // Note: The intermediate eval() is needed here (for now) due to a bug in Eigen that occurs
            // when Measurement::RowsAtCompileTime == 1 AND State::RowsAtCompileTime >= 8
            decltype(sigmaStatePoints) W = this->sigmaWeights_c.transpose().template replicate<State::RowsAtCompileTime,1>();
            Matrix<T, State::RowsAtCompileTime, Measurement::RowsAtCompileTime> P_xy
                    = (sigmaStatePoints.colwise() - x).cwiseProduct( W ).eval()
                    * (sigmaMeasurementPoints.colwise() - y).transpose();
            
            K = P_xy * P_yy.inverse();
            return true;
        }
        
        /**
         * @brief Update the state covariance matrix using the Kalman Gain and the Innovation Covariance
         * 
         * @note This covers equation (14) of Algorithm 2.1 in the Paper
         *
         * @param [in] K The computed Kalman Gain matrix
         * @param [in] P_yy The innovation covariance
         * @return True on success, false if a numerical error is encountered when updating the matrix
         */
        template<class Measurement>
        bool updateStateCovariance(const KalmanGain<Measurement>& K, const Covariance<Measurement>& P_yy)
        {
            P -= K * P_yy * K.transpose();
            return true;
        }
    };
}

#endif
