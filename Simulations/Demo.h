#pragma once

class MassSpringSystemSimulator;

class Demo {
public:
	MassSpringSystemSimulator& simulator;
	int elapsed_frames = 0;
	explicit Demo(MassSpringSystemSimulator& sim) :simulator(sim) {
		this->elapsed_frames = 0;
	}
	virtual void printPointsInfo() = 0;
	virtual void setup() = 0;
	virtual void init() = 0;
};