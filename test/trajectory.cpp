#include "smoothTrajectory.h"
#include <iostream>

int main()
{
    std::cout << "Testing smooth trajectory library..." << std::endl;

    const double tInitial = 0.0;
    const double tFinal = 10.0;
    //Eigen::Affine3d TInitial = Eigen::Affine3d::Identity();
    //Eigen::Affine3d TFinal = Eigen::Translation3d(1.0, 2.0, 3.0) * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());

    //SmoothTrajectory T(tInitial, tFinal, TInitial, TFinal);

    //std::cout << "Initial translation:" << std::endl << TInitial.translation() << std::endl;
    //std::cout << "Initial rotation:" << std::endl << TInitial.rotation() << std::endl;

    //std::cout << "Final translation:" << std::endl << TFinal.translation() << std::endl;
    //std::cout << "Final rotation:" << std::endl << TFinal.rotation() << std::endl;
    
    return 0;
}