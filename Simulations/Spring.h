#pragma once
#ifndef SPRING
#define SPRING


struct Spring {
public:
	Spring(int point1, int point2, float initialLength, float stiffness);
	int point1;
	int point2;
	float initialLength;
	float stiffness;
};
#endif