#include "MassSpringSystemSimulator.h"
#include "Demo1.cpp"


MassSpringSystemSimulator::MassSpringSystemSimulator():gravity(0,0,0)
{
	demo1 = std::make_unique<Demo1>(*this).get();
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo 1, Demo 2, Demo 3, Demo 4";
}


void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler, Leapfrog, Midpoint");
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
	reset();
	
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
		std::cout << force << std::endl;
		p.force += force;
	}
}





void MassSpringSystemSimulator::reset()
{
	//demo->setup();
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	for (Point& p : points) {
		DUC->drawSphere(p.position, 0.01f);
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
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
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
		float inputScale = 1.0f;
		inputWorld = inputWorld * inputScale;
		m_externalForce = inputWorld;
	}
	else {
		m_externalForce = Vec3(0, 0, 0);
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	simulateEuler(timeStep);
	checkBoundaries();
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
		Vec3 force = -s.stiffness * (l - s.initialLength) * (p1.position - p2.position) / l;
		p1.force += force;
		p2.force += -force;
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

}

void MassSpringSystemSimulator::checkBoundaries()
{
	for (Point& p : points) {
		if (std::abs(p.position.x) > BOUNDS) {
			p.velocity.x = 0;
			p.position.x = p.position.x < -BOUNDS ? -BOUNDS : BOUNDS;
		}
		if (std::abs(p.position.y) > BOUNDS) {
			p.velocity.y = 0;
			p.position.y = p.position.y < -BOUNDS ? -BOUNDS : BOUNDS;
		}
		if (std::abs(p.position.z) > BOUNDS) {
			p.velocity.z = 0;
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
