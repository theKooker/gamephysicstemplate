#pragma once
#ifndef DEMO1_H
#define DEMO1_H

#include "Demo.h"

class Demo1 :public virtual Demo
{
public:
	Demo1(MassSpringSystemSimulator& s) : Demo(s)
	{
	}

	virtual void init() override;
	virtual void printPointsInfo() override;
	virtual void setup() override;
private:

};

#endif DEMO1_H
