#include <iostream>
#include "Manipulator.h"
#include "flaccoController.h"

using namespace std;

int main() {
    MatrixXf DH(4,4);
    DH << 0, 1, 0, 1,
          0, 1, 0, 1,
          0, 1, 0, 1,
          0, 1, 0, 1;
    Vector4f q_in{0,0,0,0};
    Manipulator man{DH,q_in};
    Vector3f obst{3,2,0};
    FlaccoController contr{0,0,0,q_in}; // We need it just to check projectJ & projectP
    contr.newObst(obst);
    cout << "\n-----------------------------------\n";
    cout << "Jacobian:\n" << man.jacobian() << endl;
    cout << "______________\n";
    cout << "Vector n:\n" << -contr.eeDisVec(man.dKin())/contr.eeDis(man.dKin())<< endl;
    cout << "______________\n";
    cout << "Projected Jacobian:\n" << contr.projectJ(man.jacobian(),man.dKin())<< endl;
    cout << "\n-----------------------------------\n";
    cout << "Projection of p_dot\n________________________\n"; /*TODO: computation of p_dot*/
    Vector3f test_vel{10,10,10};
    cout << "Vector n:\n" << -contr.eeDisVec(man.dKin())/contr.eeDis(man.dKin())<< endl;
    cout << "______________\n";
    cout << "Velocity test_vel:\n" << test_vel << endl;
    cout << "______________\n";
    cout << "Velocity projection:\n" << contr.projectP(test_vel,man.dKin()) << endl;
    return 0;
}