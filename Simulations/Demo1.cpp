#include "Demo1.h"
#include "MassSpringSystemSimulator.h"

void Demo1::init()
{
	simulator.setIntegrator(0);
	elapsed_frames = 0;
	simulator.setMass(10);
	simulator.addMassPoint(Vec3(0, 0, 0) *0.1f , Vec3(-1, 0, 0) * .01f, false);
	simulator.addMassPoint(Vec3(0, 2, 0) * 0.1f, Vec3(1, 0, 0) * .01f, false);
	simulator.setStiffness(40);
	simulator.addSpring(0, 1, 0.2f);
	printPointsInfo();
	simulator.simulateTimestep(0.1);
	printPointsInfo();
		
}

void Demo1::printPointsInfo()
{
	size_t counter = 0;
	for (Point& p : simulator.points) {
		std::cout << "This point " << counter++ << ":" << std::endl;
		std::cout << "\tPosition " << p.position << std::endl;
		std::cout << "\tVelocity " << p.velocity << std::endl;

	}
}

void Demo1::setup()
{
	init();
}
