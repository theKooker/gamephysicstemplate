#pragma once
#ifndef POINT_H
#define POINT_H
#include "Simulator.h"

struct Point {
public:
	Point(Vec3 position, Vec3 velocity, float mass, bool isFixed);
	Vec3 position;
	Vec3 velocity;
	Vec3 tempPosition;
	Vec3 tempVelocity;
	Vec3 force;
	float mass;
	bool isFixed;
};
#endif POINT_H