#include <traj_gen2/TrajGen.hpp>
#include <iostream>
using namespace trajgen;
using DataType = double;
int main(int argc, char *argv[])
{
    const int dim = 3;
    uint poly_order = 8;
    uint max_conti = 4;

    trajgen::time_knots<DataType> ts{0,2,4,7};
    trajgen::PolyParam pp(poly_order, max_conti, trajgen::ALGORITHM::POLY_COEFF);
    Vector<DataType, dim> objWeights(0.0f, 19.0f, 1.0f);
    trajgen::PolyTrajGen<DataType, dim> pTraj(ts, pp);

    // pins
    trajgen::FixPin<DataType,dim> x1(0.0f,0,Vector<DataType,dim>(0.,0,0));
    trajgen::FixPin<DataType,dim> x2(2.0f,0,Vector<DataType,dim>(2.,-1,2));
    trajgen::FixPin<DataType,dim> x3(4.0f,0,Vector<DataType,dim>(5.,3,4));
    trajgen::FixPin<DataType,dim> x4(5.0f,0,Vector<DataType,dim>(7, -5, 4));
    std::vector<trajgen::Pin<DataType, dim> *> pinSet{&x1, &x2, &x3, &x4};
    pTraj.addPinSet(pinSet);

    trajgen::FixPin<DataType, dim> xdot0(0.0f, 1, Vector<DataType, dim>(0, 0, 0));
    trajgen::FixPin<DataType, dim> xddot0(0.0f, 2, Vector<DataType, dim>(0, 0, 0));
    pTraj.addPin(&xdot0);
    pTraj.addPin(&xddot0);

    trajgen::LoosePin<DataType, dim> passCube(3.0, 0, Vector<DataType, dim>(3, -3, 1), Vector<DataType, dim>(4.2, -2, 2));
    pTraj.addPin(&passCube);

    pTraj.setDerivativeObj(objWeights);
    bool _verbose = false;
    bool _isSolved = pTraj.solve(_verbose);
    if (_isSolved)
    {
        auto res = pTraj.eval(2, 2);
        std::cout << res << std::endl;
    }
    else
    {
        std::cout<<"unsolve this problem"<<std::endl;
    }

}