#include "smoothTrajectory.h"
#include <iostream>


template class SmoothTrajectory<float>;

template<class T>
SmoothTrajectory<T>::SmoothTrajectory(const T tStart, const T tEnd,
    const Eigen::Vector<T, Eigen::Dynamic> pStart, const Eigen::Vector<T, Eigen::Dynamic> pEnd) :
                                   m_pStart{pStart}, m_pEnd{pEnd},
                                   m_tStart{tStart}, m_tEnd{tEnd},
                                   m_polyCoeffs{Eigen::Vector<T, 4>::Zero()}
{
    // Calculate the smooting polynomial coefficients
    cubicSmoothingPolynomial();
}


template<class T>
SmoothTrajectory<T>::~SmoothTrajectory()
{
}

template<class T>
Eigen::Vector<T, Eigen::Dynamic> SmoothTrajectory<T>::jointTrajectoryPos(const T t)
{
    return m_pStart + smoothingPolynomial(t) * (m_pEnd - m_pStart);
}

template<class T>
Eigen::Vector<T, Eigen::Dynamic> SmoothTrajectory<T>::jointTrajectoryVel(const T t)
{
    return smoothingPolynomialDerivative(t) * (m_pEnd - m_pStart);
}

template<class T>
void SmoothTrajectory<T>::cubicSmoothingPolynomial()
{
    Eigen::Matrix<T, 4, 4> A;
    Eigen::Vector<T, 4> b;

    A << 1.0, m_tStart, std::pow(m_tStart, 2), std::pow(m_tStart, 3),
         0.0, 1.0, 2.0 * m_tStart, 3.0 * std::pow(m_tStart, 2),
         1.0, m_tEnd, std::pow(m_tEnd, 2), std::pow(m_tEnd, 3),
         0.0, 1.0, 2.0 * m_tEnd, 3.0 * std::pow(m_tEnd, 2);
    b << 0.0, 0.0, 1.0, 0.0;

    m_polyCoeffs = A.inverse() * b;
}

template<class T>
T SmoothTrajectory<T>::smoothingPolynomial(const T t)
{
    return m_polyCoeffs(0) + m_polyCoeffs(1) * t + m_polyCoeffs(2) * std::pow(t, 2) + m_polyCoeffs(3) * std::pow(t, 3);
}

template<class T>
T SmoothTrajectory<T>::smoothingPolynomialDerivative(const T t)
{    
    return m_polyCoeffs(1) + 2*m_polyCoeffs(2)*t + 3*m_polyCoeffs(3)*std::pow(t, 2);
}