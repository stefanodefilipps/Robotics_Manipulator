example_projection: Manipulator.o flaccoController.o example_projection.o
	g++ -o example_projection Manipulator.o flaccoController.o example_projection.o
Manipulator.o: Manipulator.cpp
	g++ -c Manipulator.cpp
flaccoController.o: flaccoController.cpp
	g++ -c flaccoController.cpp
example_projection.o: example_projection.cpp
	g++ -c example_projection.cpp
clean:
	rm *.o example_projection
