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
    Vector3f obst{4,1,0};
    FlaccoController contr{0,0,0,q_in}; // We need it just to check projectJ & projectP
    contr.newObst(obst);
    cout << "\n-----------------------------------\n";
    cout << "Jacobian:\n" << man.jacobian() << endl;
    cout << "______________\n";
    cout << "Vector n:\n" << -contr.eeDisVec(man.dKin())/contr.eeDis(man.dKin())<< endl;
    cout << "______________\n";
    cout << "Projected Jacobian:\n" << contr.projectJ(man.jacobian(),man.dKin())<< endl;
    cout << "\n-----------------------------------\n";
    cout << "TODO: projection of p_dot\n"; /*TODO: projection of p_dot*/
    return 0;
}