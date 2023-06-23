#ifndef SMOOTH_TRAJECTORY_H_
#define SMOOTH_TRAJECTORY_H_

#include "Eigen/Dense"


class SmoothTrajectory
{
public:
    SmoothTrajectory(const double tStart, const double tEnd,
                     const double pStart, const double pEnd);
    ~SmoothTrajectory();

    /**
     * Calculate the position value on the trajectory for a given time
     * 
     * @param t Time for which the trajectory to be evaluated
     * @return Position of the point on the trajectory
     * @see K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning and
     *      Control," Cambridge University Press, 2017.
    */
    double jointTrajectoryPos(const double t);
    
    /**
     * Calculate the velocity value on the trajectory for a given time
     * 
     * @param t Time for which the trajectory to be evaluated
     * @return Velocity of the point on the trajectory
     * @see K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning and
     *      Control," Cambridge University Press, 2017.
    */
    double jointTrajectoryVel(const double t);

    

private:
    // Order of the smoothing polynomial, only cubic polynomial is implemented
    int m_polyOrder;

    // Coefficients of the smooting polynomial
    Eigen::Vector4d m_polyCoeffs;

    // Start position of the trajectory
    double m_pStart;

    // End position of the trajectory
    double m_pEnd;

    // Start time of the trajectory
    double m_tStart;

    // End time of the trajectory
    double m_tEnd;

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
    double smoothingPolynomial(const double t);

    /**
     * Calculate the derivative of the smoothing polynomial for a given time t
     * 
     * @param t Time for which the trajectory to be evaluated
     * @return Time derivative of the smoothing polynomial
     * @see K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning and
     *      Control," Cambridge University Press, 2017.
    */
    double smoothingPolynomialDerivative(const double t);
};

#endif // SMOOTH_TRAJECTORY_H_