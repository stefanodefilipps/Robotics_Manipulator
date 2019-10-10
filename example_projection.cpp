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
    FlaccoController contr{1,1,10,q_in}; // We need it just to check projectJ & projectP
    contr.newObst(obst);
    cout << "\n-----------------------------------\n";
    cout << "Jacobian:\n" << man.jacobian() << endl;
    cout << "______________\n";
    cout << "Vector n:\n" << -contr.eeDisVec(man.dKin())/contr.eeDis(man.dKin())<< endl;
    cout << "______________\n";
    cout << "Projected Jacobian:\n" << contr.projectJ(man.jacobian(),man.dKin())<< endl;
    cout << "\n-----------------------------------\n";
    cout << "Projection of p_dot\n________________________\n";
    for (int i = 1; i <= 4; ++i) {
        Vector3f pos_contrI{man.dKin(q_in,i)};
        cout << "\n------------\n" << "| JOINT " << i << ": |\n" << "------------\n" << endl;
        cout << "Vector n:\n" << -contr.eeDisVec(pos_contrI)/contr.eeDis(pos_contrI)<< endl;
        cout << "______________\n";
        cout << "Repulsive Velocity:\n" << contr.eeRepulsiveVelocity(pos_contrI) << endl;
        cout << "______________\n";
        cout << "Velocity projection:\n" << contr.projectP(pos_contrI) << endl;
    }
    return 0;
}