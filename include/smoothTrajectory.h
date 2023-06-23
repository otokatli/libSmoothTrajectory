#include "Eigen/Geometry"
#include <array>

class SmoothTrajectory
{
public:
    SmoothTrajectory(const double tStart, const double tEnd,
                     const double pStart, const double pEnd);
    ~SmoothTrajectory();

    /**
     * @brief Calculate the position on the desired straight trajectory for a given time
     * @param t Time [s]
     * @return Point on the trajectory
    */
    double jointTrajectoryPos(const double t);
    
    /**
     * @brief Calculate a velocity on the desired straight trajectory for a given time
     * @param t Time [s]
     * @return Velocity on the trajectory
    */
    double jointTrajectoryVel(const double t);

    

private:
    int m_polyOrder;
    double m_pStart;
    double m_pEnd;
    double m_tStart;
    double m_tEnd;
    Eigen::Vector4d m_polyCoeffs;

    /**
     * @brief Calculate the 3rd order polynomial
     * @return Polynomial coefficients in increasing order
    */
    void cubicSmoothingPolynomial();

    /**
     * @brief Evaluate the smooting polynomial
     * @param t Time [s]
     * @return Value of the smooting polynomial at time t
    */
    double smoothingPolynomial(const double t);
    double smoothingPolynomialDerivative(const double t);


    //void linearSmoothingPolynomial(Eigen::Vector2d& coeffs);
    //void quinticSmoothingPolynomial(Eigen::Vector<double, 6>& coeffs);
};