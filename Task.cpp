#include "Task.h"

std::vector<MatrixXf> Task::get() const {
    return stack;
}

void Task::set(const std::vector<MatrixXf>& stack_) {

}

void Task::swapTask(int i, int j) {
    int temp{indices(i)};
    indices(j) = indices(i);
    indices(i) = temp;
}

Eigen::MatrixXf& Task::operator()(int k) const {
    return stack[inces(k)];
}
