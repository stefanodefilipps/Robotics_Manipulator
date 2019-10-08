#include<iostream>
#include "Manipulator.h"

using namespace std;

int main() {

    MatrixXf DH(3, 4);
    DH << 0, 1, 0, 1,
            0, 1, 0, 1,
            0, 1, 0, 1;
    Vector3f q_in{0,0,0};
    Manipulator man{DH,q_in};
    cout << man.dKin(q_in,2) << endl;
    for (int i = 0; i < 3; ++i) {
        cout << "----------------------------------------\n";
        cout << "Jacobian at " << i << "-th joint is:\n";
        cout << man.jacobian(q_in,i) << endl;
    }

    return 0;
}