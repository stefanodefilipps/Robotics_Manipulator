#ifndef __TASK__
#define __TASK__

#include <Eigen/Dense>

class Task {
public:
    Task(std::vector<Eigen::MatrixXf>& stack_) :stack{stack_}, indices{0,1,2,3}
    std::vector<MatrixXf> get() const;
    void set(const std::vector<MatrixXf>& stack_);
    void swapTask(int i, int j);
    Eigen::MatrixXf& operator()(int k) const;
private:
    std::vector<Eigen::MatrixXf> stack;
    std::vecor<int> indices;
};

#endif