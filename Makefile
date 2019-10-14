example_ctrPoints: example_ctrPoints.o Manipulator.o
	g++ -o example_ctrPoints example_ctrPoints.o Manipulator.o
Manipulator.o: Manipulator.cpp
	g++ -c Manipulator.cpp
example_ctrPoints.o: example_ctrPoints.cpp
	g++ -c example_ctrPoints.cpp
clean:
	rm *.o example_ctrPoints
