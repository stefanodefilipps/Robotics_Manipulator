#include<iostream>
#include "Manipulator.h"

using namespace std;

int main() {

    MatrixXf DH(5, 4);
    DH << 0, 1, 0, 1,
            0, 1, 0, 1,
            0, 1, 0, 1,
            0, 1, 0, 1,
            0, 1, 0, 1;
    VectorXf q_in(4);
    cout << "Type 4 joint values for initial configuration: ";
    for (int j{0}; j < static_cast<int>(q_in.size()); ++j) {
        cin >> q_in[j];
    }
    cout << "Joint configuration is: \nq_in = \n" << q_in << endl;
    Manipulator man{DH,q_in};
    for (int i{1}; i <= static_cast<int>(q_in.size()); ++i) {
        cout << "----------------------------------------\n";
        cout << "Jacobian at " << i << "-th joint is:\n";
        cout << man.jacobian(q_in,i) << endl;
        cout << "dKin at " << i << "-th joint is:\n";
        cout << man.dKin(q_in,i) << endl;
    }

    return 0;
}