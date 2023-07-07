#ifndef SMOOTH_TRAJECTORY_H_
#define SMOOTH_TRAJECTORY_H_

#include "Eigen/Dense"


template<class T>
class SmoothTrajectory
{
public:
    SmoothTrajectory(const T tStart, const T tEnd,
                     const Eigen::Vector<T, Eigen::Dynamic> pStart, const Eigen::Vector<T, Eigen::Dynamic> pEnd);
    ~SmoothTrajectory();

    /**
     * Calculate the position value on the trajectory for a given time
     * 
     * @param t Time for which the trajectory to be evaluated
     * @return Position of the point on the trajectory
     * @see K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning and
     *      Control," Cambridge University Press, 2017.
    */
    Eigen::Vector<T, Eigen::Dynamic> jointTrajectoryPos(const T t);
    
    /**
     * Calculate the velocity value on the trajectory for a given time
     * 
     * @param t Time for which the trajectory to be evaluated
     * @return Velocity of the point on the trajectory
     * @see K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning and
     *      Control," Cambridge University Press, 2017.
    */
    Eigen::Vector<T, Eigen::Dynamic> jointTrajectoryVel(const T t);

    

private:
    // Coefficients of the smooting polynomial
    Eigen::Vector<T, 4> m_polyCoeffs;

    // Start position of the trajectory
    Eigen::Vector<T, Eigen::Dynamic> m_pStart;

    // End position of the trajectory
    Eigen::Vector<T, Eigen::Dynamic > m_pEnd;

    // Start time of the trajectory
    T m_tStart;

    // End time of the trajectory
    T m_tEnd;

    /**
     * Calculate the coefficients of the third order smoothing polynomial used to
     *  map the time duration to [0, 1] interval
     * 
     * Smoothing polynomial is an efficient way to adjust the speed on the
     *  trajectory while keeping the positions intact.
     * The polynomial is of the form
     *  s(t) = a0 + a1*t + a2*t^2 + a3*t^3
     * 
     * @return Coefficients of the polynomial in ascending order
     * @see K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning and
     *      Control," Cambridge University Press, 2017.
    */
    void cubicSmoothingPolynomial();

    /**
     * Calculate the value of the smoothing polynomial for a given time t
     * 
     * @param t Time for which the trajectory to be evaluated
     * @return Time mapped to [0, 1] closed interval
     * @see K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning and
     *      Control," Cambridge University Press, 2017.
    */
    T smoothingPolynomial(const T t);

    /**
     * Calculate the derivative of the smoothing polynomial for a given time t
     * 
     * @param t Time for which the trajectory to be evaluated
     * @return Time derivative of the smoothing polynomial
     * @see K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning and
     *      Control," Cambridge University Press, 2017.
    */
    T smoothingPolynomialDerivative(const T t);
};

#endif // SMOOTH_TRAJECTORY_H_