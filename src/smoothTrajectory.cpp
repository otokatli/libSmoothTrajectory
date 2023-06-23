#include "smoothTrajectory.h"
#include <iostream>

SmoothTrajectory::SmoothTrajectory(const double tStart, const double tEnd,
                                   const double pStart, const double pEnd) :
                                   m_polyOrder{3}, m_pStart{pStart}, m_pEnd{pEnd},
                                   m_tStart{tStart}, m_tEnd{tEnd}
{
    // Calculate the smooting polynomial coefficients
    cubicSmoothingPolynomial();
}

SmoothTrajectory::~SmoothTrajectory()
{
}

double SmoothTrajectory::jointTrajectoryPos(const double t)
{
    return m_pStart + smoothingPolynomial(t) * (m_pEnd - m_pStart);
}

double SmoothTrajectory::jointTrajectoryVel(const double t)
{
    return smoothingPolynomialDerivative(t) * (m_pEnd - m_pStart);
}

void SmoothTrajectory::cubicSmoothingPolynomial()
{
    Eigen::Matrix4d A;
    Eigen::Vector4d b;

    A << 1.0, m_tStart, std::pow(m_tStart, 2), std::pow(m_tStart, 3),
         0.0, 1.0, 2.0 * m_tStart, 3.0 * std::pow(m_tStart, 2),
         1.0, m_tEnd, std::pow(m_tEnd, 2), std::pow(m_tEnd, 3),
         0.0, 1.0, 2.0 * m_tEnd, 3.0 * std::pow(m_tEnd, 2);
    b << 0.0, 0.0, 1.0, 0.0;

    m_polyCoeffs = A.inverse() * b;
}

double SmoothTrajectory::smoothingPolynomial(const double t)
{
    double smoothingPolyVal = 0.0;

    for (int order = 0; order < 4; ++order)
        smoothingPolyVal += m_polyCoeffs(order) * std::pow(t, order);
    
    return smoothingPolyVal;
}

double SmoothTrajectory::smoothingPolynomialDerivative(const double t)
{    
    return m_polyCoeffs(1) + 2*m_polyCoeffs(2)*t + 3*m_polyCoeffs(3)*std::pow(t, 2);
}
