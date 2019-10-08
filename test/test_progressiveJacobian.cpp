#include<iostream>
#include "Manipulator.h"

using namespace std;

int main() {

    MatrixXf DH(4, 4);
    DH << 0, 1, 0, 1,
            0, 1, 0, 1,
            0, 1, 0, 1,
            0, 1, 0, 1;
    Vector4f q_in{0,0,0,0};
    Manipulator man{DH,q_in};
    for (int i{1}; i <= 4; ++i) {
        cout << "----------------------------------------\n";
        cout << "Jacobian at " << i << "-th joint is:\n";
        cout << man.jacobian(q_in,i) << endl;
        cout << "dKin at " << i << "-th joint is:\n";
        cout << man.dKin(q_in,i) << endl;
    }

    return 0;
}