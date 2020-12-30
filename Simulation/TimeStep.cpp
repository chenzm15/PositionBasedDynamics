#include "TimeStep.h"
#include "TimeManager.h"
#include "Simulation.h"


using namespace PBD;
using namespace std;
using namespace GenParam;

TimeStep::TimeStep()
{
}

TimeStep::~TimeStep(void)
{
}

void TimeStep::init()
{
	initParameters();
}

void TimeStep::initParameters()
{
	ParameterObject::initParameters();
}

static void computeAccelerationUnderSlidingFriction(const ParticleData& pd, unsigned int i, const Vector3r& grav, Vector3r& a)
{
    const Real friction_coeff = 0.8;
    const Real eps = static_cast<Real>(1e-6);
    Vector3r total_other_force = pd.getConstraintForce(i) + pd.getMass(i) * grav;
    const Vector3r& velocity = pd.getVelocity(i);
    Real total_other_force_x = std::abs(total_other_force.x());
    Real pressure = std::sqrt(total_other_force.y() * total_other_force.y() + total_other_force.z() * total_other_force.z());
    Real max_friction = friction_coeff * pressure;
    if (velocity.norm() < eps) {
        if (total_other_force_x <= max_friction + eps) {
            a.setZero();
        }
        else {
            a.y() = a.z() = 0.0;
            int sign = total_other_force.x() > 0 ? 1 : -1;
            a.x() = sign * (total_other_force_x - max_friction) * pd.getInvMass(i);
        }
    }
    else {
        int neg_sign = velocity.x() < 0 ? 1 : -1;
        a.y() = a.z() = 0.0;
        a.x() = (total_other_force.x() + neg_sign * max_friction) * pd.getInvMass(i);
    }
    /*if (a.x() != 0.0) {
        printf("a_x = %lf", a.x());
    }*/
}

void TimeStep::clearAccelerations(SimulationModel &model)
{
	//////////////////////////////////////////////////////////////////////////
	// rigid body model
	//////////////////////////////////////////////////////////////////////////

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	Simulation *sim = Simulation::getCurrent();
	const Vector3r grav(sim->getVecValue<Real>(Simulation::GRAVITATION));
	for (size_t i = 0; i < rb.size(); i++)
	{
		// Clear accelerations of dynamic particles
		if (rb[i]->getMass() != 0.0)
		{
			Vector3r &a = rb[i]->getAcceleration();
			a = grav;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// particle model
	//////////////////////////////////////////////////////////////////////////

	ParticleData &pd = model.getParticles();
	const unsigned int count = pd.size();
	for (unsigned int i = 0; i < count; i++)
	{
		// Clear accelerations of dynamic particles
        if (pd.getParticleType(i) == ParticleType::CURTAIN_HANGING_POINT)
            computeAccelerationUnderSlidingFriction(pd, i, grav, pd.getAcceleration(i));
		else if (pd.getMass(i) != 0.0)
		{
			Vector3r &a = pd.getAcceleration(i);
			a = grav;
		}
        pd.setConstraintForce(i, Vector3r(0.0, 0.0, 0.0));
	}
}

void TimeStep::reset()
{
}

void TimeStep::setCollisionDetection(SimulationModel &model, CollisionDetection *cd)
{
	m_collisionDetection = cd;
	m_collisionDetection->setContactCallback(contactCallbackFunction, &model);
	m_collisionDetection->setSolidContactCallback(solidContactCallbackFunction, &model);
}

CollisionDetection *TimeStep::getCollisionDetection()
{
	return m_collisionDetection;
}

void TimeStep::contactCallbackFunction(const unsigned int contactType, const unsigned int bodyIndex1, const unsigned int bodyIndex2,
	const Vector3r &cp1, const Vector3r &cp2,
	const Vector3r &normal, const Real dist,
	const Real restitutionCoeff, const Real frictionCoeff, void *userData)
{
	SimulationModel *model = (SimulationModel*)userData;
	if (contactType == CollisionDetection::RigidBodyContactType)
		model->addRigidBodyContactConstraint(bodyIndex1, bodyIndex2, cp1, cp2, normal, dist, restitutionCoeff, frictionCoeff);
	else if (contactType == CollisionDetection::ParticleRigidBodyContactType)
		model->addParticleRigidBodyContactConstraint(bodyIndex1, bodyIndex2, cp1, cp2, normal, dist, restitutionCoeff, frictionCoeff);
}

void TimeStep::solidContactCallbackFunction(const unsigned int contactType, const unsigned int bodyIndex1, const unsigned int bodyIndex2,
	const unsigned int tetIndex, const Vector3r &bary,
	const Vector3r &cp1, const Vector3r &cp2,
	const Vector3r &normal, const Real dist,
	const Real restitutionCoeff, const Real frictionCoeff, void *userData)
{
	SimulationModel *model = (SimulationModel*)userData;
	if (contactType == CollisionDetection::ParticleSolidContactType)
		model->addParticleSolidContactConstraint(bodyIndex1, bodyIndex2, tetIndex, bary, cp1, cp2, normal, dist, restitutionCoeff, frictionCoeff);
}