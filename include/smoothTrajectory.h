#include "Eigen/Geometry"

class SmoothTrajectory
{
public:
    SmoothTrajectory(const double tInitial, const double tFinal,
                     const Eigen::Affine3d TInitial, const Eigen::Affine3d TFinal);
    SmoothTrajectory(const double tInitial, const double tFinal,
                     const Eigen::Affine3d TInitial, const Eigen::Affine3d TFinal,
                     const int polyOrder);
    ~SmoothTrajectory();

    double screwTrajectory(const double t);

    double m_polyCoeffs[5] = {0.0};

private:
    int m_polyOrder;
    Eigen::Affine3d m_TInitial;
    Eigen::Affine3d m_TFinal;
    double m_tInitial;
    double m_tFinal;

    void linearSmoothingPolynomial(Eigen::Vector2d& coeffs);
    void cubicSmoothingPolynomial(Eigen::Vector4d& coeffs);
    void quinticSmoothingPolynomial(Eigen::Vector<double, 6>& coeffs);
    double smoothingPolynomial(const double t);
};