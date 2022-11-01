#pragma once
#ifndef DEMO_H
#define DEMO_H

#include "MassSpringSystemSimulator.h"



class Demo {
public:
	MassSpringSystemSimulator& simulator;
	int elapsed_frames = 0;
	explicit Demo(MassSpringSystemSimulator& simulator) :simulator(simulator), elapsed_frames(0) {
	}
	virtual void setup() = 0;
	void printPointsInfo() {
		size_t counter = 0;
		for (Point& p : simulator.points) {
			std::cout << "This point " << counter++ << ":" << std::endl;
			std::cout << "\tPosition " << p.position << std::endl;
			std::cout << "\tVelocity " << p.velocity << std::endl;

		}
	}
	void init() {
		simulator.setIntegrator(0);
		simulator.setMass(10);
		simulator.addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		simulator.addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		simulator.setStiffness(40);
		simulator.addSpring(0, 1, 1);
		elapsed_frames = 0;
	}
}; 

#endif