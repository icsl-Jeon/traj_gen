#include <traj_gen2/TrajGen.hpp>
#include <iostream>
#include <chrono>

using namespace trajgen;
using namespace Eigen;
using Type = double ;

int main(){
    // 1. Prameter setting
    // common
    const int dim = 3;
    trajgen::time_knots<Type> ts{0,2,4,7};
    trajgen::Vector<Type,3> objWeights(0,1,1);

    // polyTrajGen
    uint poly_order = 8, max_conti = 4;
    trajgen::PolyParam pp(poly_order,max_conti,ALGORITHM::END_DERIVATIVE); // or ALGORITHM::POLY_COEFF
    // optimTrajGen
    Type pntDensity = 5;

    // 2. Pin
    // 2.1 FixPin
    FixPin<Type,dim> x1(0.0f,0,Vector<Type,dim>(0,0,0));
    FixPin<Type,dim> x2(2.0f,0,Vector<Type,dim>(2,-1,2));
    FixPin<Type,dim> x3(4.0f,0,Vector<Type,dim>(5,3,4));
    FixPin<Type,dim> x4(7.0f,0,Vector<Type,dim>(7,-5,5));

    FixPin<Type,dim> xdot0(0.0f,1,Vector<Type,dim>(0,0,0));
    FixPin<Type,dim> xddot0(0.0f,2,Vector<Type,dim>(0,0,0));

    // 2.2 LoosePin
    LoosePin<Type,dim> passCube(3.0f,0,Vector<Type,dim>(3,-3,1),Vector<Type,dim>(4.2,-2,2));

    std::vector<Pin<Type,dim>*> pinSet{&x1,&x2,&x3,&x4,&xdot0,&xddot0,&passCube}; // to prevent downcasting slicing, vector of pointers

    // Let's test the two trajGen class
    TrajGen<Type,dim>** pTrajGenSet = new TrajGen<Type,dim>*[1]; //2
    pTrajGenSet[0] = new PolyTrajGen<Type,dim>(ts,pp);
    pTrajGenSet[1] = new OptimTrajGen<Type,dim>(ts,pntDensity);
    bool verbose = false;
    bool isSolved = false;
    string TrajGenClass[2] = {"PolyTraj","OptimTraj"};

    Type t_eval = 3; d_order d_eval = 1;

    for (uint i = 0 ; i <2 ; i++) {
        auto begin = std::chrono::steady_clock::now();
        pTrajGenSet[i]->setDerivativeObj(objWeights);
        pTrajGenSet[i]->addPinSet(pinSet);
        isSolved = pTrajGenSet[i]->solve(verbose);
        printf("For %s, The evaluated value for (t=%.2f, d=%d) : ",TrajGenClass[i].c_str(),t_eval,d_eval);
        if (isSolved)
            cout << pTrajGenSet[i]->eval(t_eval,d_eval).transpose() << endl;
        else
            cout << "Failed. " << endl;

        auto end= std::chrono::steady_clock::now();

        std::cout << "Total time :  " <<
        std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()*1e-3 << "[ms]" << std::endl;
    }
    return 0;

}