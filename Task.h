#ifndef __TASK__
#define __TASK__

#include <Eigen/Dense>
#include <vector>

class Task {
public:
	// CONSTRUCTOR
    Task(std::vector<Eigen::MatrixXf>& stack_) :stack{stack_} {
		for (int i = 0; i < static_cast<int>(stack.size()); ++i) {
			indices.push_back(i);
		}
	}
    // GET FUNCTIONS
    std::vector<Eigen::MatrixXf> get() const; /*GET THE STACK, NON PRIORITIZED*/
    std::vector<Eigen::MatrixXf> getStack() const; /*GET THE STACK, PRIORITIZED*/
	std::vector<int> getInd() const; /*GET PRIORITY INDICES*/
	// SET FUNCTIONS
    void set(const std::vector<Eigen::MatrixXf>& stack_);
    // METHODS
    void swapTask(int i, int j);
    // OPERATORS
    Eigen::MatrixXf operator()(int k) const; // ACCESS i-th VALUE OF NON PRIORITIZED TASK
    Eigen::MatrixXf operator[](int k) const; // ACCESS i-th VALE OF PRIORITIZED TASK
private:
    std::vector<Eigen::MatrixXf> stack;
    std::vector<int> indices;
};

#endif