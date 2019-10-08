#include<iostream>
#include "Manipulator.h"

using namespace std;

int main() {

    MatrixXf DH(3, 4);
    DH << 0, 1, 0, 1,
            0, 1, 0, 1,
            0, 1, 0, 1;
    Vector3f q0{0,0,0};
    Manipulator man{DH,q0};
    for (int i = 0; i < 3; ++i) {
        cout << "----------------------------------------\n";
        cout << "Jacobian at " << i+1 << "-th joint is:\n";
        cout << man.jacobian(q0,0.5,1) << endl;
    }

    return 0;
}