#include "Demo.h"

class Demo1 :public Demo {
public:
	void Demo::setup() {
		init();

		simulator.simulateTimestep(.1);
		std::cout << "Euler method, results after a single step by .1:" << endl;
		printPointsInfo();
	}
};