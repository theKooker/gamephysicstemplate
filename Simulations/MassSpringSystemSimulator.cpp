#include "MassSpringSystemSimulator.h"


MassSpringSystemSimulator::MassSpringSystemSimulator():gravity(0,0,0), gravityScale(0)
{
	setIntegrator(EULER);
	m_iTestCase = 3;
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo 1, Demo 2, Demo 3, Demo 4";
}


void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "step=1 min=0");
	switch (m_iTestCase)
	{
	case 0: demo1(); break;
	case 1: demo2(); break;
	case 2: demo3(); break;
	case 3: demo4(); break;
	default:
		break;
	}
}
void MassSpringSystemSimulator::demo1() {
	reset();
	setGravity(Vec3(0, 0, 0));
	TwRemoveVar(DUC->g_pTweakBar, "Timestep");
	std::cout << ">>>>>>>>>>>>>>>>DEMO1<<<<<<<<<<<<<<<<<" << std::endl;
	std::cout << "EULER STEP" << std::endl;
	setMass(10);
	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	setStiffness(40);
	addSpring(0, 1, 1);
	printPointsInfo();
	simulateEuler(0.1);
	printPointsInfo();
	reset();
	std::cout << "MIDPOINT STEP" << std::endl;
	setMass(10);
	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	setStiffness(40);
	addSpring(0, 1, 1);
	printPointsInfo();
	simulateMidpoint(0.1);
	printPointsInfo();
}
void MassSpringSystemSimulator::demo2() {
	reset();
	setGravity(Vec3(0, 0, 0));

	TwRemoveVar(DUC->g_pTweakBar, "Timestep");
	std::cout << ">>>>>>>>>>>>>>>>DEMO2<<<<<<<<<<<<<<<<<" << std::endl;
	std::cout << "(!) We use the Euler Integration" << std::endl;
	setIntegrator(EULER);
	setMass(10);
	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	setStiffness(40);
	addSpring(0, 1, 1);

}
void MassSpringSystemSimulator::demo3() {
	reset();
	setGravity(Vec3(0, 0, 0));
	TwRemoveVar(DUC->g_pTweakBar, "Timestep");
	std::cout << ">>>>>>>>>>>>>>>>DEMO3<<<<<<<<<<<<<<<<<" << std::endl;
	std::cout << "(!) We use the Midpoint Integration" << std::endl;
	setIntegrator(MIDPOINT);
	setMass(10);
	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	setStiffness(40);
	addSpring(0, 1, 1);

}
void MassSpringSystemSimulator::demo4() {
	reset();
	std::cout << ">>>>>>>>>>>>>>>>DEMO4<<<<<<<<<<<<<<<<<" << std::endl;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler, Leapfrog, Midpoint");
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &gravityScale, "step=1 min=0");
	setGravity(Vec3(0,-gravityScale, 0));
	setMass(10);
	int fixPoint1 = addMassPoint(Vec3(-3, 3, 0), Vec3(0.f), true);
	int fixPoint2 = addMassPoint(Vec3(3, 3, 0), Vec3(0.f), true);

	//cube
	int ltf = addMassPoint(Vec3(1, 1, 1), Vec3(0.f), false);
	int ltb = addMassPoint(Vec3(1, 1, -1), Vec3(0.f), false);
	int rtf = addMassPoint(Vec3(-1, 1, 1), Vec3(0.f), false);
	int rtb = addMassPoint(Vec3(-1, 1, -1), Vec3(0.f), false);
	int lbf = addMassPoint(Vec3(1, -1, 1), Vec3(0.f), false);
	int lbb = addMassPoint(Vec3(1, -1, -1), Vec3(0.f), false);
	int rbf = addMassPoint(Vec3(-1, -1, 1), Vec3(0.f), false);
	int rbb = addMassPoint(Vec3(-1, -1, -1), Vec3(0.f), false);


	setStiffness(40);

	//cube
	addSpring(fixPoint1, rtf, 3);
	addSpring(fixPoint2, ltf, 3);
	addSpring(fixPoint1, rtb, 3);
	addSpring(fixPoint2, ltb, 3);
	addSpring(ltf, ltb, 1);
	addSpring(ltf, rtf, 1);
	addSpring(ltf, lbf, 1);
	addSpring(ltb, lbb, 1);
	addSpring(ltb, rtb, 1);
	addSpring(rtb, rtf, 1);
	addSpring(rtb, rbb, 1);
	addSpring(rtf, rbf, 1);
	addSpring(lbf, lbb, 1);
	addSpring(lbb, rbb, 1);
	addSpring(rbb, rbf, 1);
	addSpring(rbf, lbf, 1);



}
void startSimulation() {
	
}
void MassSpringSystemSimulator::printPointsInfo()
{
	size_t counter = 0;
	for (Point& p : points) {
		std::cout << "This point " << counter++ << ":" << std::endl;
		std::cout << "\tPosition " << p.position << std::endl;
		std::cout << "\tVelocity " << p.velocity << std::endl;

	}
}

void MassSpringSystemSimulator::setMass(float mass)
{
	this->m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	this->m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	this->m_fDamping = damping;
}

void MassSpringSystemSimulator::setGravity(Vec3 gravity)
{
	this->gravity = gravity;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	points.emplace_back(position, velocity, m_fMass, isFixed);
	return points.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	springs.emplace_back(masspoint1, masspoint2, m_fStiffness, initialLength);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return points.at(index).position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return points.at(index).velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	for (Point& p: points)
	{
		p.force += force;
	}
}





void MassSpringSystemSimulator::reset()
{
	points.clear();
	springs.clear();
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	for (Point& p : points) {
		DUC->drawSphere(p.position,0.01f);
	}
	for (Spring& s : springs) {
		Point& p1 = points[s.point1];
		Point& p2 = points[s.point2];
		DUC->beginLine();
		DUC->drawLine(p1.position, Vec3(0, 0, 1), p2.position, Vec3(0, 0, 1));
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.3f;
		inputWorld = inputWorld * inputScale;
		m_externalForce = inputWorld;
	}
	else {
		m_externalForce = Vec3(0, 0, 0);
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0: reset(); return;
	case 1 :
	case 2	: timeStep = 0.005; break;
	default:
		break;
	}
	switch (m_iIntegrator)
	{
	case EULER: simulateEuler(timeStep); break;
	case MIDPOINT: simulateMidpoint(timeStep); break;
	default:
		break;
	}
	if (m_iTestCase == 3) {
		checkBoundaries();
	}

}

void MassSpringSystemSimulator::simulateEuler(float timestep)
{
	computePointsAndSpringsForces();
	externalForcesCalculations(timestep);
	applyExternalForce(m_externalForce);
	integrateEulerPosition(timestep);
	integrateEulorVelocity(timestep);

}

void MassSpringSystemSimulator::computePointsAndSpringsForces()
{
	for (Point& p : points) {
		p.force = gravity;
	} 
	for (Spring& s : springs) {
		Point& p1 = points.at(s.point1);
		Point& p2 = points.at(s.point2);
		float l = norm(p1.position - p2.position);
		Vec3 force = -s.stiffness * (s.initialLength - l) * getNormalized(p1.position - p2.position);
		p1.force += force;
		p2.force -= force;
	}
}

void MassSpringSystemSimulator::computeTempPointsAndSpringsForces() {
	for (Point& p : points) {
		p.force = gravity;
	}
	for (Spring& s : springs) {
		Point& p1 = points.at(s.point1);
		Point& p2 = points.at(s.point2);
		float l = norm(p1.tempPosition- p2.tempPosition);
		Vec3 force = -s.stiffness * (s.initialLength - l) * getNormalized(p1.tempPosition - p2.tempPosition);
		p1.force += force;
		p2.force -= force;
	}
}

void MassSpringSystemSimulator::integrateEulerPosition(float timestep)
{
	for (Point& p : points) {
		if (!p.isFixed) {
			p.position = p.position + p.velocity * timestep;
		}
	}
}

void MassSpringSystemSimulator::integrateEulorVelocity(float timestep)
{
	for (Point& p : points) {
		if (!p.isFixed) {
			Vec3 acceleration = p.force / p.mass;
			p.velocity = p.velocity + acceleration * timestep;
		}
	}
}

void MassSpringSystemSimulator::simulateMidpoint(float timestep)
{
	computePointsAndSpringsForces();
	externalForcesCalculations(timestep);
	applyExternalForce(m_externalForce);
	for (Point& p : points) {
		if (!p.isFixed) {
			p.tempPosition = p.position + p.velocity * timestep / 2;
			Vec3 acceleration = p.force / p.mass;
			p.tempVelocity = p.velocity + acceleration * timestep / 2;
			p.position = p.position + timestep * p.tempVelocity;
		}
	}
	computeTempPointsAndSpringsForces();
	externalForcesCalculations(timestep);
	applyExternalForce(m_externalForce);
	for (Point& p : points) {
		if (!p.isFixed) {
			Vec3 acceleration = p.force / p.mass;
			p.velocity = p.velocity + timestep * acceleration;
		}
	}

}

void MassSpringSystemSimulator::checkBoundaries()
{
	bool bounded = false;
	for (Point& p : points) {
		if (std::abs(p.position.x) > BOUNDS) {
			p.velocity.x = -p.velocity.x;
			p.position.x = p.position.x < -BOUNDS ? -BOUNDS : BOUNDS;
		}
		if (std::abs(p.position.y) > BOUNDS) {
			p.velocity.y = -p.velocity.y;
			p.position.y = p.position.y < -BOUNDS ? -BOUNDS : BOUNDS;
		}
		if (std::abs(p.position.z) > BOUNDS) {
			p.velocity.z = -p.velocity.z;
			p.position.z = p.position.z < -BOUNDS ? -BOUNDS : BOUNDS;
		}
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
