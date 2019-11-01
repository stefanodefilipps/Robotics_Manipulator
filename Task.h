#ifndef __TASK__
#define __TASK__

#include <vector>

template <typename T>
class Task {
public:
	// CONSTRUCTOR
    Task(std::vector<T>& stack_) :stack{stack_} {
		for (int i = 0; i < static_cast<int>(stack.size()); ++i) {
			indices.push_back(i);
		}
	}
    // GET FUNCTIONS
    std::vector<T> get() const /*GET THE STACK, NON PRIORITIZED*/ {
        return stack;
	}
    std::vector<T> getStack() const /*GET THE STACK, PRIORITIZED*/ {
	    std::vector<T> temp{stack};
        for (int i = 0; i < static_cast<int>(stack.size()); ++i) {
            temp[i] = stack[indices[i]];
        }
        return temp;
	}
	std::vector<int> getInd() const /*GET PRIORITY INDICES*/ {
        return indices;
	}
	int size() const {
        return static_cast<int>(indices.size());
	}
	// SET FUNCTIONS
	void push(T newElement) {
	    stack.push_back(newElement);
	    int actSize = static_cast<int>( indices.size() );
	    indices.push_back(actSize);
	}
    // METHODS
    void goUpTo(int start, int end) {
        int temp{indices[start]};
        for (int i = start; i > end; --i) {
            indices[i] = indices[i-1];
        }
        indices[end] = temp;
	}
    void swapTask(int i, int j) {
        int temp{indices[i]};
        indices[i] = indices[j];
        indices[j] = temp;
	}
    // OPERATORS
    T operator()(int k) const /* ACCESS i-th VALUE OF NON PRIORITIZED TASK */ {
        return stack[k];
	}
    T operator[](int k) const /* ACCESS i-th VALE OF PRIORITIZED TASK*/ {
        return stack[indices[k]];
    }
private:
    std::vector<T> stack;
    std::vector<int> indices;
};

#endif