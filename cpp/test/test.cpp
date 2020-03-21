#include <traj_gen2/TrajGen.hpp>
#include <iostream>
#include <chrono>

using namespace trajgen;
using namespace Eigen;
using Type = double ;

int main(){
    auto begin = std::chrono::steady_clock::now();
    // 1. Prameter setting
    const int dim = 3; uint poly_order = 8, max_conti = 4;
    time_knots<Type> ts{0,2,4,7};
//     PolyParam pp(poly_order,max_conti,ALGORITHM::POLY_COEFF);
    PolyParam pp(poly_order,max_conti,ALGORITHM::END_DERIVATIVE);
    Vector<Type,dim> objWeights(0,1,1);
    PolyTrajGen<Type,dim> pTraj(ts,pp);
    // 2. Pin
    // 2.1 FixPin
    FixPin<Type,dim> x1(0.0f,0,Vector<Type,dim>(0,0,0));
    FixPin<Type,dim> x2(2.0f,0,Vector<Type,dim>(2,-1,2));
    FixPin<Type,dim> x3(4.0f,0,Vector<Type,dim>(5,3,4));
    FixPin<Type,dim> x4(7.0f,0,Vector<Type,dim>(7,-5,5));
    std::vector<Pin<Type,dim>*> pinSet{&x1,&x2,&x3,&x4}; // to prevent downcasting slicing, vector of pointers
    pTraj.addPinSet(pinSet);

    FixPin<Type,dim> xdot0(0.0f,1,Vector<Type,dim>(0,0,0));
    FixPin<Type,dim> xddot0(0.0f,2,Vector<Type,dim>(0,0,0));
    pTraj.addPin(&xdot0); pTraj.addPin(&xddot0);

    // 2.2 LoosePin
    LoosePin<Type,dim> passCube(3.0f,0,Vector<Type,dim>(3,-3,1),Vector<Type,dim>(4.2,-2,2));
    pTraj.addPin(&passCube);

    // 3. Solve
    pTraj.setDerivativeObj(objWeights); bool verbose = false;
    bool isSolved = pTraj.solve(verbose);

    // 4. Evaulate the curve
    Type t_eval = 3,d_eval = 1;
    Vector<Type,dim> xdot_eval = pTraj.eval(t_eval,d_eval);
    if (isSolved)
        cout << xdot_eval << endl;

    // optimTrajGen
    time_knots <Type> tsPair{0,7}; Type pntDensity = 4;
    OptimTrajGen<Type,dim> oTraj(tsPair,pntDensity);


    auto end= std::chrono::steady_clock::now();
    std::cout << "Terminated with " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()*1e-3 << "[ms]" << std::endl;
    return 0;

}