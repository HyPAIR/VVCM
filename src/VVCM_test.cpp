#include <iostream>
#include <Eigen/Dense>
#include "VVCM_FK.hpp"

using namespace VVCM;

int main()
{
    int N = 4;              // number of robots
    float zr = 1000.0;      // height of the holding points (unit: mm)
    MatrixXf Rn(4, 2);      // robot formation (x, y) coordinates (unit: mm)
    Rn << 213.7, 122.7,
          804.6,  37.2,
          904.0, 550.0,
          439.3, 715.9;
    MatrixXf Vn(4, 2);      // shape of the deformable sheet (x, y) coordinates (unit: mm)
    Vn << -316.1, -421.9,
           803.4, -384.1,
           746.1,  712.8,
          -367.3,  664.2;

    std::cout << "----------------------" << std::endl;

    VVCM_FK a(N, zr, Vn);            // initialize the VVCM_FK object
    a.update_stable_solutions(Rn);   // compute the stable forward kinematics

    // M: the number of stable solutions found
    std::cout << "M: " << a.M << std::endl;

    if (a.M == 0)
    {
        std::cout << "No stable solution found." << std::endl;
        return 0;
    }

    // Po: object pose (x, y, z) coordinates for each stable solution (unit: mm)
    std::cout << "Po: " << std::endl;
    for (const auto &po : a.Po)
    {
        std::cout << po.transpose() << std::endl;
    }

    // Vo: deformable sheet vertex (x, y) coordinates for each stable solution (unit: mm)
    std::cout << "Vo: " << std::endl;
    for (const auto &vo : a.Vo)
    {
        std::cout << vo.transpose() << std::endl;
    }

    std::cout << "----------------------" << std::endl;

    return 0;
}