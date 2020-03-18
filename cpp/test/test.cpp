#include <traj_gen2/TrajGen.hpp>
#include <iostream>
using namespace trajgen;
using namespace Eigen;

int main(){
    // 1. Prameter setting
    const int dim = 3; uint poly_order = 8, max_conti = 4;
    time_knots ts{0,2,4,7};
    PolyParam pp(poly_order,max_conti,ALGORITHM::POLY_COEFF);
    Vector3f objWeights(0,1,1);
    PolyTrajGen<dim> pTraj(ts,pp);

    // 2. Pin
    // 2.1 FixPin
    FixPin<dim> x1(0.0f,0,Vector3f(0,0,0));
    FixPin<dim> x2(2.0f,0,Vector3f(2,-1,2));
    FixPin<dim> x3(4.0f,0,Vector3f(5,3,4));
    FixPin<dim> x4(7.0f,0,Vector3f(7,-5,5));
    std::vector<Pin<dim>*> pinSet{&x1,&x2,&x3,&x4}; // to prevent downcasting slicing, vector of pointers
    pTraj.addPinSet(pinSet);

    FixPin<dim> xdot0(0.0f,1,Vector3f(0,0,0));
    FixPin<dim> xddot0(0.0f,2,Vector3f(0,0,0));
    pTraj.addPin(&xdot0); pTraj.addPin(&xddot0);

    // 2.2 LoosePin
    LoosePin<dim> passCube(3.0f,0,Vector3f(3,-3,1),Vector3f(4.2,-2,2));
    pTraj.addPin(&passCube);

    // 3. Solve
    pTraj.setDerivativeObj(objWeights); bool verbose = false;
    bool isSolved = pTraj.solve(verbose);

    // 4. Evaulate the curve
    float t_eval = 3,d_eval = 1;
    Vector3f xdot_eval = pTraj.eval(t_eval,d_eval);
    if (isSolved)
        cout << xdot_eval << endl;
    return 0; 

}