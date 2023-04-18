#include "smoothTrajectory.h"

SmoothTrajectory::SmoothTrajectory(const double tInitial, const double tFinal,
                                   const Eigen::Affine3d TInitial, const Eigen::Affine3d TFinal)
{
    m_polyOrder = 3;
    m_TInitial = TInitial;
    m_TFinal = TFinal;
    m_tInitial = tInitial;
    m_tFinal = tFinal;
}

SmoothTrajectory::SmoothTrajectory(const double tInitial, const double tFinal,
                                   const Eigen::Affine3d TInitial, const Eigen::Affine3d TFinal,
                                   const int polyOrder)
{
    m_polyOrder = m_polyOrder;
    m_TInitial = TInitial;
    m_TFinal = TFinal;
    m_tInitial = tInitial;
    m_tFinal = tFinal;
}

SmoothTrajectory::~SmoothTrajectory()
{
}

double SmoothTrajectory::screwTrajectory(const double t)
{
    // Smoothing polynomial evaluated at time t
    const double s = smoothingPolynomial(t);

    return 0.0;
}

void SmoothTrajectory::linearSmoothingPolynomial(Eigen::Vector2d& coeffs)
{
    Eigen::Matrix2d A;
    Eigen::Vector2d b;

    A << 1.0, m_tInitial,
         1.0, m_tFinal;
    b << 0.0, 1.0;
}

void SmoothTrajectory::cubicSmoothingPolynomial(Eigen::Vector4d& coeffs)
{
    Eigen::Matrix4d A;
    Eigen::Vector4d b;

    A << 1.0, m_tInitial, std::pow(m_tInitial, 2), std::pow(m_tInitial, 3),
         0.0, 1.0, 2.0 * m_tInitial, 3.0 * std::pow(m_tInitial, 2),
         1.0, m_tFinal, std::pow(m_tFinal, 2), std::pow(m_tFinal, 3),
         0.0, 1.0, 2.0 * m_tFinal, 3.0 * std::pow(m_tFinal, 2);
    b << 0.0, 0.0, 1.0, 0.0;
}

void SmoothTrajectory::quinticSmoothingPolynomial(Eigen::Vector<double, 6>& coeffs)
{
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Vector<double, 6> b;

    A << 1.0, m_tInitial, std::pow(m_tInitial, 2), std::pow(m_tInitial, 3), std::pow(m_tInitial, 4), std::pow(m_tInitial, 5),
         0.0, 1.0, 2.0 * m_tInitial, 3.0 * std::pow(m_tInitial, 2), 4 * std::pow(m_tInitial, 3), 5 * std::pow(m_tInitial, 4),
         0.0, 0.0, 2.0, 6.0 * m_tInitial, 12.0 * std::pow(m_tInitial, 2), 20.0 * std::pow(m_tInitial, 3),
         1.0, m_tFinal, std::pow(m_tFinal, 2), std::pow(m_tFinal, 3), std::pow(m_tFinal, 4), std::pow(m_tFinal, 5),
         0.0, 1.0, 2.0 * m_tFinal, 3.0 * std::pow(m_tFinal, 2), 4.0 * std::pow(m_tFinal, 3), 5.0 * std::pow(m_tFinal, 4),
         0.0, 0.0, 2.0, 6.0 * m_tFinal, 12.0 * std::pow(m_tFinal, 2), 20.0 * std::pow(m_tFinal, 3);
    b << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
}

double SmoothTrajectory::smoothingPolynomial()
{
    if (m_polyOrder == 1)
    {
        linearSmoothingPolynomial();
    }
    else if (m_polyOrder == 3)
    {
        cubicSmoothingPolynomial();
    }
    else if (m_polyOrder == 5)
    {
        quinticSmoothingPolynomial();
    }
}
