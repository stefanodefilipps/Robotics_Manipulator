example_reordering: example_reordering.o Task.o Manipulator.o flaccoController.o
	g++ -o example_reordering example_reordering.o Task.o Manipulator.o flaccoController.o
Task.o: Task.cpp
	g++ -c Task.cpp
Manipulator.o: Manipulator.cpp
	g++ -c Manipulator.cpp
flaccoController.o: flaccoController.cpp
	g++ -c flaccoController.cpp
example_reordering.o: example_reordering.cpp
	g++ -c example_reordering.cpp
clean:
	rm *.o example_reorder
