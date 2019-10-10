example_swap: example_swap.o Task.o
	g++ -o example_swap example_swap.o Task.o
Task.o: Task.cpp
	g++ -c Task.cpp
example_swap.o: example_swap.cpp
	g++ -c example_swap.cpp
clean:
	rm *.o example_swap
