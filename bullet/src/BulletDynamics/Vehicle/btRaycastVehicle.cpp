/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/

#include "LinearMath/btVector3.h"
#include "btRaycastVehicle.h"

#include "BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h"
#include "BulletDynamics/ConstraintSolver/btJacobianEntry.h"
#include "LinearMath/btQuaternion.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "btVehicleRaycaster.h"
#include "btWheelInfo.h"
#include "LinearMath/btMinMax.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"

#define ROLLING_INFLUENCE_FIX

#include "BulletCollision/CollisionShapes/btTriangleMesh.h" // Mackey Kinard
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h" // Mackey Kinard
#include "BulletCollision/CollisionShapes/btConvexHullShape.h" // Mackey Kinard

btRigidBody& btActionInterface::getFixedBody()
{
	static btRigidBody s_fixed(0, 0,0);
	s_fixed.setMassProps(btScalar(0.),btVector3(btScalar(0.),btScalar(0.),btScalar(0.)));
	return s_fixed;
}

btRaycastVehicle::btRaycastVehicle(const btVehicleTuning& tuning,btRigidBody* chassis,	btVehicleRaycaster* raycaster )
:m_vehicleRaycaster(raycaster),
m_pitchControl(btScalar(0.))
{
	m_chassisBody = chassis;
	m_indexRightAxis = 0;
	m_indexUpAxis = 2;
	m_indexForwardAxis = 1;

	// Mackey Kinard
	m_enableMultiRaycast = false;
	m_minimumWheelContacts = 4;
	m_trackConnectionAccel = btScalar(0.0);
	m_smoothFlyingImpulse = btScalar(0.0);
	m_stabilizingForce = btScalar(0.0);
	m_maxImpulseForce = btScalar(0.0);

	defaultInit(tuning);
}


void btRaycastVehicle::defaultInit(const btVehicleTuning& tuning)
{
	(void)tuning;
	m_currentVehicleSpeedKmHour = btScalar(0.);
	m_steeringValue = btScalar(0.);
	
}

	

btRaycastVehicle::~btRaycastVehicle()
{
}


//
// basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed
//
btWheelInfo&	btRaycastVehicle::addWheel( const btVector3& connectionPointCS, const btVector3& wheelDirectionCS0,const btVector3& wheelAxleCS, btScalar suspensionRestLength, btScalar wheelRadius,const btVehicleTuning& tuning, bool isFrontWheel)
{

	btWheelInfoConstructionInfo ci;

	ci.m_chassisConnectionCS = connectionPointCS;
	ci.m_wheelDirectionCS = wheelDirectionCS0;
	ci.m_wheelAxleCS = wheelAxleCS;
	ci.m_suspensionRestLength = suspensionRestLength;
	ci.m_wheelRadius = wheelRadius;
	ci.m_suspensionStiffness = tuning.m_suspensionStiffness;
	ci.m_wheelsDampingCompression = tuning.m_suspensionCompression;
	ci.m_wheelsDampingRelaxation = tuning.m_suspensionDamping;
	ci.m_frictionSlip = tuning.m_frictionSlip;
	ci.m_bIsFrontWheel = isFrontWheel;
	ci.m_maxSuspensionTravelCm = tuning.m_maxSuspensionTravelCm;
	ci.m_maxSuspensionForce = tuning.m_maxSuspensionForce;

	m_wheelInfo.push_back( btWheelInfo(ci));
	
	btWheelInfo& wheel = m_wheelInfo[getNumWheels()-1];
	
	updateWheelTransformsWS( wheel , false );
	updateWheelTransform(getNumWheels()-1,false);
	return wheel;
}




const btTransform&	btRaycastVehicle::getWheelTransformWS( int wheelIndex ) const
{
	btAssert(wheelIndex < getNumWheels());
	const btWheelInfo& wheel = m_wheelInfo[wheelIndex];
	return wheel.m_worldTransform;

}

void	btRaycastVehicle::updateWheelTransform( int wheelIndex , bool interpolatedTransform)
{
	
	btWheelInfo& wheel = m_wheelInfo[ wheelIndex ];
	updateWheelTransformsWS(wheel,interpolatedTransform);
	btVector3 up = -wheel.m_raycastInfo.m_wheelDirectionWS;
	const btVector3& right = wheel.m_raycastInfo.m_wheelAxleWS;
	btVector3 fwd = up.cross(right);
	fwd = fwd.normalize();
//	up = right.cross(fwd);
//	up.normalize();

	//rotate around steering over de wheelAxleWS
	btScalar steering = wheel.m_steering;
	
	btQuaternion steeringOrn(up,steering);//wheel.m_steering);
	btMatrix3x3 steeringMat(steeringOrn);

	btQuaternion rotatingOrn(right,-wheel.m_rotation);
	btMatrix3x3 rotatingMat(rotatingOrn);

	btMatrix3x3 basis2(
		right[0],fwd[0],up[0],
		right[1],fwd[1],up[1],
		right[2],fwd[2],up[2]
	);
	
	wheel.m_worldTransform.setBasis(steeringMat * rotatingMat * basis2);
	wheel.m_worldTransform.setOrigin(
		wheel.m_raycastInfo.m_hardPointWS + wheel.m_raycastInfo.m_wheelDirectionWS * wheel.m_raycastInfo.m_suspensionLength
	);
}

void btRaycastVehicle::resetSuspension()
{

	int i;
	for (i=0;i<m_wheelInfo.size();	i++)
	{
			btWheelInfo& wheel = m_wheelInfo[i];
			wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
			wheel.m_suspensionRelativeVelocity = btScalar(0.0);
			
			wheel.m_raycastInfo.m_contactNormalWS = - wheel.m_raycastInfo.m_wheelDirectionWS;
			//wheel_info.setContactFriction(btScalar(0.0));
			wheel.m_clippedInvContactDotSuspension = btScalar(1.0);
	}
}

// Mackey Kinard (Note: Raycast Fraction - Helps Collision Mesh Gaps)
void	btRaycastVehicle::updateWheelTransformsWS(btWheelInfo& wheel , bool interpolatedTransform, float raycastFraction)
{
	wheel.m_raycastInfo.m_isInContact = false;

	btTransform chassisTrans = getChassisWorldTransform();
	if (interpolatedTransform && (getRigidBody()->getMotionState()))
	{
		getRigidBody()->getMotionState()->getWorldTransform(chassisTrans);
	}

	wheel.m_raycastInfo.m_hardPointWS = chassisTrans( wheel.m_chassisConnectionPointCS * raycastFraction );
	wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() *  wheel.m_wheelDirectionCS ;
	wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}

btScalar btRaycastVehicle::rayCast(btWheelInfo& wheel, float fraction)
{
	updateWheelTransformsWS( wheel,false,fraction );

	
	btScalar depth = -1;
	
	btScalar raylen = wheel.getSuspensionRestLength()+wheel.m_wheelsRadius;

	btVector3 rayvector = wheel.m_raycastInfo.m_wheelDirectionWS * (raylen);
	const btVector3& source = wheel.m_raycastInfo.m_hardPointWS;
	wheel.m_raycastInfo.m_contactPointWS = source + rayvector;
	const btVector3& target = wheel.m_raycastInfo.m_contactPointWS;

	btScalar param = btScalar(0.);
	
	btVehicleRaycaster::btVehicleRaycasterResult	rayResults;

	btAssert(m_vehicleRaycaster);

	void* object = m_vehicleRaycaster->castRay(source,target,rayResults);

	wheel.m_raycastInfo.m_groundObject = 0;

	if (object)
	{
		param = rayResults.m_distFraction;
		depth = raylen * rayResults.m_distFraction;
		wheel.m_raycastInfo.m_contactNormalWS  = rayResults.m_hitNormalInWorld;
		wheel.m_raycastInfo.m_isInContact = true;
		
		wheel.m_raycastInfo.m_groundObject = &getFixedBody();///@todo for driving on dynamic/movable objects!;
		//wheel.m_raycastInfo.m_groundObject = object;


		btScalar hitDistance = param*raylen;
		wheel.m_raycastInfo.m_suspensionLength = hitDistance - wheel.m_wheelsRadius;
		//clamp on max suspension travel

		btScalar  minSuspensionLength = wheel.getSuspensionRestLength() - wheel.m_maxSuspensionTravelCm*btScalar(0.01);
		btScalar maxSuspensionLength = wheel.getSuspensionRestLength()+ wheel.m_maxSuspensionTravelCm*btScalar(0.01);
		if (wheel.m_raycastInfo.m_suspensionLength < minSuspensionLength)
		{
			wheel.m_raycastInfo.m_suspensionLength = minSuspensionLength;
		}
		if (wheel.m_raycastInfo.m_suspensionLength > maxSuspensionLength)
		{
			wheel.m_raycastInfo.m_suspensionLength = maxSuspensionLength;
		}

		wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;

		btScalar denominator= wheel.m_raycastInfo.m_contactNormalWS.dot( wheel.m_raycastInfo.m_wheelDirectionWS );

		btVector3 chassis_velocity_at_contactPoint;
		btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS-getRigidBody()->getCenterOfMassPosition();

		chassis_velocity_at_contactPoint = getRigidBody()->getVelocityInLocalPoint(relpos);

		btScalar projVel = wheel.m_raycastInfo.m_contactNormalWS.dot( chassis_velocity_at_contactPoint );

		if ( denominator >= btScalar(-0.1))
		{
			wheel.m_suspensionRelativeVelocity = btScalar(0.0);
			wheel.m_clippedInvContactDotSuspension = btScalar(1.0) / btScalar(0.1);
		}
		else
		{
			btScalar inv = btScalar(-1.) / denominator;
			wheel.m_suspensionRelativeVelocity = projVel * inv;
			wheel.m_clippedInvContactDotSuspension = inv;
		}
			
	} else
	{
		//put wheel info as in rest position
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
		wheel.m_suspensionRelativeVelocity = btScalar(0.0);
		wheel.m_raycastInfo.m_contactNormalWS = - wheel.m_raycastInfo.m_wheelDirectionWS;
		wheel.m_clippedInvContactDotSuspension = btScalar(1.0);
	}

	return depth;
}


const btTransform& btRaycastVehicle::getChassisWorldTransform() const
{
	/*if (getRigidBody()->getMotionState())
	{
		btTransform chassisWorldTrans;
		getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans);
		return chassisWorldTrans;
	}
	*/

	
	return getRigidBody()->getCenterOfMassTransform();
}


////////////////////////////////////////////////////////////////////////////////////
// Mackey Kinard - Advanced Raycast Vehicle Suspension Simulation
////////////////////////////////////////////////////////////////////////////////////

void btRaycastVehicle::updateVehicle( btScalar step )
{
	{
		for (int i=0;i<getNumWheels();i++)
		{
			updateWheelTransform(i,false);
		}
	}


	m_currentVehicleSpeedKmHour = btScalar(3.6) * getRigidBody()->getLinearVelocity().length();
	
	const btTransform& chassisTrans = getChassisWorldTransform();

	btVector3 forwardW (
		chassisTrans.getBasis()[0][m_indexForwardAxis],
		chassisTrans.getBasis()[1][m_indexForwardAxis],
		chassisTrans.getBasis()[2][m_indexForwardAxis]);

	if (forwardW.dot(getRigidBody()->getLinearVelocity()) < btScalar(0.))
	{
		m_currentVehicleSpeedKmHour *= btScalar(-1.);
	}

	//////////////////////////////////////////
	// LEGACY: simulate suspension
	//////////////////////////////////////////
	// int i=0;
	// for (i=0;i<m_wheelInfo.size();i++)
	// {
	//	btScalar depth; 
	//	depth = rayCast( m_wheelInfo[i]);
	// }
	//////////////////////////////////////////

	// Mackey Kinard - Multi raycasting wheel on ground contact
	// -------------------------------------------------------------
	int i = 0, wheelsOnGround = 0;
	for (i=0;i<m_wheelInfo.size();i++)
	{
		btScalar depth;
		depth = rayCast(m_wheelInfo[i]);
		if(m_wheelInfo[i].m_raycastInfo.m_isInContact)
		{
			wheelsOnGround++;
		}
		else
		{
	    	if (m_enableMultiRaycast == true)
			{
				// If the original raycast did not hit the ground,
				// try a little bit (5%) closer to the centre of the chassis.
				// Some tracks have very minor gaps that would otherwise
				// trigger very odd physical behaviour.
				depth = rayCast(m_wheelInfo[i], 0.95f);
				if (m_wheelInfo[i].m_raycastInfo.m_isInContact)
				{
					wheelsOnGround++;
				}
			}
		}
	}

	// Mackey Kinard - If the vehicle is flying, try to keep it parallel to the ground.
	// -------------------------------------------------------------
	if (wheelsOnGround == 0 && m_smoothFlyingImpulse > 0)
	{
		btVector3 vehicle_up    = getChassisWorldTransform().getBasis().getColumn(1);
		btVector3 terrain_up = -m_chassisBody->getGravity();
		terrain_up = terrain_up.normalize();

		// Length of axis depends on the angle - i.e. the further awat
		// the vehicle is from being upright, the larger the applied impulse
		// will be, resulting in fast changes when the vehicle is on its
		// side, but not overcompensating (and therefore shaking) when
		// the vehicle is not much away from being upright.
		btVector3 axis = vehicle_up.cross(terrain_up);

		// To avoid the vehicle going backwards/forwards (or rolling sideways),
		// set the pitch/roll to 0 before applying the 'straightening' impulse.
		// TODO: make this works if gravity is changed.
		btVector3 av = m_chassisBody->getAngularVelocity();
		av.setX(0);
		av.setZ(0);
		m_chassisBody->setAngularVelocity(av);

		// Give a nicely balanced feeling for rebalancing the vehicle
		m_chassisBody->applyTorqueImpulse(axis * m_smoothFlyingImpulse);
	}

	updateSuspension(step);

	
	for (i=0;i<m_wheelInfo.size();i++)
	{
		//apply suspension force
		btWheelInfo& wheel = m_wheelInfo[i];
		
		btScalar suspensionForce = wheel.m_wheelsSuspensionForce;
		
		if (suspensionForce > wheel.m_maxSuspensionForce)
		{
			suspensionForce = wheel.m_maxSuspensionForce;
		}
		btVector3 impulse = wheel.m_raycastInfo.m_contactNormalWS * suspensionForce * step;
		btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();
		
		getRigidBody()->applyImpulse(impulse, relpos);
	
	}
	
	updateFriction( step);

	// Mackey Kinard - If configured, add a force to keep vehicles on the track
	// -------------------------------------------------------------
	if (m_stabilizingForce > 0 && m_minimumWheelContacts > 0 && wheelsOnGround >= m_minimumWheelContacts)
	{
		float rawSpeedFactor = getRigidBody()->getLinearVelocity().length();
		float downImpluseFactor = fabsf(rawSpeedFactor) * m_stabilizingForce;
		if (m_maxImpulseForce > 0 && downImpluseFactor > m_maxImpulseForce)
		{
			downImpluseFactor = m_maxImpulseForce;
		}
		btVector3 downImpulseVector = m_chassisBody->getWorldTransform().getBasis() * btVector3(0, -downImpluseFactor, 0);
		m_chassisBody->applyCentralImpulse(downImpulseVector);
	}

 	// TODO: Mackey Kinard - Apply additional impulse vector - ???
	// -------------------------------------------------------------
    //if (m_ticks_additional_impulse > 0)
    //{
    //    // We have fixed timestep
    //    float dt = stk_config->ticks2Time(1);
    //    m_chassisBody->applyCentralImpulse(m_additional_impulse * dt);
    //    m_ticks_additional_impulse--;
    //}
	
	for (i=0;i<m_wheelInfo.size();i++)
	{
		btWheelInfo& wheel = m_wheelInfo[i];
		btVector3 relpos = wheel.m_raycastInfo.m_hardPointWS - getRigidBody()->getCenterOfMassPosition();
		btVector3 vel = getRigidBody()->getVelocityInLocalPoint( relpos );

		if (wheel.m_raycastInfo.m_isInContact)
		{
			const btTransform&	chassisWorldTransform = getChassisWorldTransform();

			btVector3 fwd (
				chassisWorldTransform.getBasis()[0][m_indexForwardAxis],
				chassisWorldTransform.getBasis()[1][m_indexForwardAxis],
				chassisWorldTransform.getBasis()[2][m_indexForwardAxis]);

			btScalar proj = fwd.dot(wheel.m_raycastInfo.m_contactNormalWS);
			fwd -= wheel.m_raycastInfo.m_contactNormalWS * proj;

			btScalar proj2 = fwd.dot(vel);
			
			wheel.m_deltaRotation = (proj2 * step) / (wheel.m_wheelsRadius);
			wheel.m_rotation += wheel.m_deltaRotation;

		} else
		{
			wheel.m_rotation += wheel.m_deltaRotation;
		}
		
		wheel.m_deltaRotation *= btScalar(0.99);//damping of rotation when not in contact

	}



}


void	btRaycastVehicle::setSteeringValue(btScalar steering,int wheel)
{
	btAssert(wheel>=0 && wheel < getNumWheels());

	btWheelInfo& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_steering = steering;
}



btScalar	btRaycastVehicle::getSteeringValue(int wheel) const
{
	return getWheelInfo(wheel).m_steering;
}


void	btRaycastVehicle::applyEngineForce(btScalar force, int wheel)
{
	btAssert(wheel>=0 && wheel < getNumWheels());
	btWheelInfo& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_engineForce = force;
}


const btWheelInfo&	btRaycastVehicle::getWheelInfo(int index) const
{
	btAssert((index >= 0) && (index < 	getNumWheels()));
	
	return m_wheelInfo[index];
}

btWheelInfo&	btRaycastVehicle::getWheelInfo(int index) 
{
	btAssert((index >= 0) && (index < 	getNumWheels()));
	
	return m_wheelInfo[index];
}

void btRaycastVehicle::setBrake(btScalar brake,int wheelIndex)
{
	btAssert((wheelIndex >= 0) && (wheelIndex < 	getNumWheels()));
	getWheelInfo(wheelIndex).m_brake = brake;
}


void	btRaycastVehicle::updateSuspension(btScalar deltaTime)
{
	(void)deltaTime;

	btScalar chassisMass = btScalar(1.) / m_chassisBody->getInvMass();
	
	for (int w_it=0; w_it<getNumWheels(); w_it++)
	{
		btWheelInfo &wheel_info = m_wheelInfo[w_it];
		
		if ( wheel_info.m_raycastInfo.m_isInContact )
		{
			btScalar force;
			//	Spring
			{
				btScalar	susp_length			= wheel_info.getSuspensionRestLength();
				btScalar	current_length = wheel_info.m_raycastInfo.m_suspensionLength;

				btScalar length_diff = (susp_length - current_length);

				force = wheel_info.m_suspensionStiffness
					* length_diff * wheel_info.m_clippedInvContactDotSuspension;
			}
		
			// Damper
			{
				btScalar projected_rel_vel = wheel_info.m_suspensionRelativeVelocity;
				{
					btScalar	susp_damping;
					if ( projected_rel_vel < btScalar(0.0) )
					{
						susp_damping = wheel_info.m_wheelsDampingCompression;
					}
					else
					{
						susp_damping = wheel_info.m_wheelsDampingRelaxation;
					}
					force -= susp_damping * projected_rel_vel;
				}
			}

			// RESULT
			wheel_info.m_wheelsSuspensionForce = force * chassisMass;
			if (wheel_info.m_wheelsSuspensionForce < btScalar(0.))
			{
				wheel_info.m_wheelsSuspensionForce = btScalar(0.);
			}
		}
		else
		{
			// LEGACY: wheel_info.m_wheelsSuspensionForce = btScalar(0.0);
			// Mackey Kinard - Warning
			// -------------------------------------------------------------
			// A very unphysical thing to handle slopes that are a bit too
            // steep or uneven (resulting in only one wheel on the ground)
            // If only the front or only the rear wheels are on the ground, add
            // a force pulling the axis down (towards the ground). Note that it
            // is already guaranteed that either both or no wheels on one axis
            // are on the ground, so we have to test only one of the wheels
            // In hindsight it turns out that this code basically adds
            // additional gravity when a kart is flying. So if this code would
            // be removed some jumps (esp. Enterprise) do not work as expected anymore.
			// -------------------------------------------------------------
			if (m_trackConnectionAccel > 0)
			{
            	wheel_info.m_wheelsSuspensionForce = -m_trackConnectionAccel * chassisMass;
			}
			else
			{
				wheel_info.m_wheelsSuspensionForce = btScalar(0.0);
			}
		}
	}

}


struct btWheelContactPoint
{
	btRigidBody* m_body0;
	btRigidBody* m_body1;
	btVector3	m_frictionPositionWorld;
	btVector3	m_frictionDirectionWorld;
	btScalar	m_jacDiagABInv;
	btScalar	m_maxImpulse;


	btWheelContactPoint(btRigidBody* body0,btRigidBody* body1,const btVector3& frictionPosWorld,const btVector3& frictionDirectionWorld, btScalar maxImpulse)
		:m_body0(body0),
		m_body1(body1),
		m_frictionPositionWorld(frictionPosWorld),
		m_frictionDirectionWorld(frictionDirectionWorld),
		m_maxImpulse(maxImpulse)
	{
		btScalar denom0 = body0->computeImpulseDenominator(frictionPosWorld,frictionDirectionWorld);
		btScalar denom1 = body1->computeImpulseDenominator(frictionPosWorld,frictionDirectionWorld);
		btScalar	relaxation = 1.f;
		m_jacDiagABInv = relaxation/(denom0+denom1);
	}



};

btScalar calcRollingFriction(btWheelContactPoint& contactPoint);
btScalar calcRollingFriction(btWheelContactPoint& contactPoint)
{

	btScalar j1=0.f;

	const btVector3& contactPosWorld = contactPoint.m_frictionPositionWorld;

	btVector3 rel_pos1 = contactPosWorld - contactPoint.m_body0->getCenterOfMassPosition(); 
	btVector3 rel_pos2 = contactPosWorld - contactPoint.m_body1->getCenterOfMassPosition();
	
	btScalar maxImpulse  = contactPoint.m_maxImpulse;
	
	btVector3 vel1 = contactPoint.m_body0->getVelocityInLocalPoint(rel_pos1);
	btVector3 vel2 = contactPoint.m_body1->getVelocityInLocalPoint(rel_pos2);
	btVector3 vel = vel1 - vel2;

	btScalar vrel = contactPoint.m_frictionDirectionWorld.dot(vel);

	// calculate j that moves us to zero relative velocity
	j1 = -vrel * contactPoint.m_jacDiagABInv;
	btSetMin(j1, maxImpulse);
	btSetMax(j1, -maxImpulse);

	return j1;
}




btScalar sideFrictionStiffness2 = btScalar(1.0);
void	btRaycastVehicle::updateFriction(btScalar	timeStep)
{

		//calculate the impulse, so that the wheels don't move sidewards
		int numWheel = getNumWheels();
		if (!numWheel)
			return;

		m_forwardWS.resize(numWheel);
		m_axle.resize(numWheel);
		m_forwardImpulse.resize(numWheel);
		m_sideImpulse.resize(numWheel);
		
		int numWheelsOnGround = 0;
	

		//collapse all those loops into one!
		for (int i=0;i<getNumWheels();i++)
		{
			btWheelInfo& wheelInfo = m_wheelInfo[i];
			class btRigidBody* groundObject = (class btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;
			if (groundObject)
				numWheelsOnGround++;
			m_sideImpulse[i] = btScalar(0.);
			m_forwardImpulse[i] = btScalar(0.);

		}
	
		{
	
			for (int i=0;i<getNumWheels();i++)
			{

				btWheelInfo& wheelInfo = m_wheelInfo[i];
					
				class btRigidBody* groundObject = (class btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;

				if (groundObject)
				{

					const btTransform& wheelTrans = getWheelTransformWS( i );

					btMatrix3x3 wheelBasis0 = wheelTrans.getBasis();
					m_axle[i] = btVector3(	
						wheelBasis0[0][m_indexRightAxis],
						wheelBasis0[1][m_indexRightAxis],
						wheelBasis0[2][m_indexRightAxis]);
					
					const btVector3& surfNormalWS = wheelInfo.m_raycastInfo.m_contactNormalWS;
					btScalar proj = m_axle[i].dot(surfNormalWS);
					m_axle[i] -= surfNormalWS * proj;
					m_axle[i] = m_axle[i].normalize();
					
					m_forwardWS[i] = surfNormalWS.cross(m_axle[i]);
					m_forwardWS[i].normalize();

				
					resolveSingleBilateral(*m_chassisBody, wheelInfo.m_raycastInfo.m_contactPointWS,
							  *groundObject, wheelInfo.m_raycastInfo.m_contactPointWS,
							  btScalar(0.), m_axle[i],m_sideImpulse[i],timeStep);

					m_sideImpulse[i] *= sideFrictionStiffness2;
						
				}
				

			}
		}

	btScalar sideFactor = btScalar(1.);
	btScalar fwdFactor = 0.5;

	bool sliding = false;
	{
		for (int wheel =0;wheel <getNumWheels();wheel++)
		{
			btWheelInfo& wheelInfo = m_wheelInfo[wheel];
			class btRigidBody* groundObject = (class btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;

			btScalar	rollingFriction = 0.f;

			if (groundObject)
			{
				if (wheelInfo.m_engineForce != 0.f)
				{
					rollingFriction = wheelInfo.m_engineForce* timeStep;
				} else
				{
					btScalar defaultRollingFrictionImpulse = 0.f;
					btScalar maxImpulse = wheelInfo.m_brake ? wheelInfo.m_brake : defaultRollingFrictionImpulse;
					btWheelContactPoint contactPt(m_chassisBody,groundObject,wheelInfo.m_raycastInfo.m_contactPointWS,m_forwardWS[wheel],maxImpulse);
					rollingFriction = calcRollingFriction(contactPt);
				}
			}

			//switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)
			



			m_forwardImpulse[wheel] = btScalar(0.);
			m_wheelInfo[wheel].m_skidInfo= btScalar(1.);

			if (groundObject)
			{
				m_wheelInfo[wheel].m_skidInfo= btScalar(1.);
				
				btScalar maximp = wheelInfo.m_wheelsSuspensionForce * timeStep * wheelInfo.m_frictionSlip;
				btScalar maximpSide = maximp;

				btScalar maximpSquared = maximp * maximpSide;
			

				m_forwardImpulse[wheel] = rollingFriction;//wheelInfo.m_engineForce* timeStep;

				btScalar x = (m_forwardImpulse[wheel] ) * fwdFactor;
				btScalar y = (m_sideImpulse[wheel] ) * sideFactor;
				
				btScalar impulseSquared = (x*x + y*y);

				if (impulseSquared > maximpSquared)
				{
					sliding = true;
					
					btScalar factor = maximp / btSqrt(impulseSquared);
					
					m_wheelInfo[wheel].m_skidInfo *= factor;
				}
			} 

		}
	}

	


		if (sliding)
		{
			for (int wheel = 0;wheel < getNumWheels(); wheel++)
			{
				if (m_sideImpulse[wheel] != btScalar(0.))
				{
					if (m_wheelInfo[wheel].m_skidInfo< btScalar(1.))
					{
						m_forwardImpulse[wheel] *=	m_wheelInfo[wheel].m_skidInfo;
						m_sideImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
					}
				}
			}
		}

		// apply the impulses
		{
			for (int wheel = 0;wheel<getNumWheels() ; wheel++)
			{
				btWheelInfo& wheelInfo = m_wheelInfo[wheel];

				btVector3 rel_pos = wheelInfo.m_raycastInfo.m_contactPointWS - 
						m_chassisBody->getCenterOfMassPosition();

				if (m_forwardImpulse[wheel] != btScalar(0.))
				{
					m_chassisBody->applyImpulse(m_forwardWS[wheel]*(m_forwardImpulse[wheel]),rel_pos);
				}
				if (m_sideImpulse[wheel] != btScalar(0.))
				{
					class btRigidBody* groundObject = (class btRigidBody*) m_wheelInfo[wheel].m_raycastInfo.m_groundObject;

					btVector3 rel_pos2 = wheelInfo.m_raycastInfo.m_contactPointWS - 
						groundObject->getCenterOfMassPosition();

					
					btVector3 sideImp = m_axle[wheel] * m_sideImpulse[wheel];

#if defined ROLLING_INFLUENCE_FIX // fix. It only worked if car's up was along Y - VT.
					btVector3 vChassisWorldUp = getRigidBody()->getCenterOfMassTransform().getBasis().getColumn(m_indexUpAxis);
					rel_pos -= vChassisWorldUp * (vChassisWorldUp.dot(rel_pos) * (1.f-wheelInfo.m_rollInfluence));
#else
					rel_pos[m_indexUpAxis] *= wheelInfo.m_rollInfluence;
#endif
					m_chassisBody->applyImpulse(sideImp,rel_pos);

					//apply friction impulse on the ground
					groundObject->applyImpulse(-sideImp,rel_pos2);
				}
			}
		}

	
}



void	btRaycastVehicle::debugDraw(btIDebugDraw* debugDrawer)
{

	for (int v=0;v<this->getNumWheels();v++)
	{
		btVector3 wheelColor(0,1,1);
		if (getWheelInfo(v).m_raycastInfo.m_isInContact)
		{
			wheelColor.setValue(0,0,1);
		} else
		{
			wheelColor.setValue(1,0,1);
		}

		btVector3 wheelPosWS = getWheelInfo(v).m_worldTransform.getOrigin();

		btVector3 axle = btVector3(	
			getWheelInfo(v).m_worldTransform.getBasis()[0][getRightAxis()],
			getWheelInfo(v).m_worldTransform.getBasis()[1][getRightAxis()],
			getWheelInfo(v).m_worldTransform.getBasis()[2][getRightAxis()]);

		//debug wheels (cylinders)
		debugDrawer->drawLine(wheelPosWS,wheelPosWS+axle,wheelColor);
		debugDrawer->drawLine(wheelPosWS,getWheelInfo(v).m_raycastInfo.m_contactPointWS,wheelColor);

	}
}


void* btDefaultVehicleRaycaster::castRay(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result)
{
//	RayResultCallback& resultCallback;

	btCollisionWorld::ClosestRayResultCallback rayCallback(from,to);

	m_dynamicsWorld->rayTest(from, to, rayCallback);

	if (rayCallback.hasHit())
	{
		
		const btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
        if (body && body->hasContactResponse())
		{
			result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
			result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
			result.m_hitNormalInWorld.normalize();
			result.m_distFraction = rayCallback.m_closestHitFraction;
			return (void*)body;
		}
	}
	return 0;
}

////////////////////////////////////////////////////
// Mackey Kinard - Smooth Vehicle Raycaster
////////////////////////////////////////////////////

void* btSmoothVehicleRaycaster::castRay(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result)
{
	// DEPRECIATED: KEEP FOR REFERENCE
	//if (this->m_shapeTestingMode == true)
	//{
	//	return this->performShapeTest(from, to, result);
	//}
	//else
	//{
		return this->performLineTest(from, to, result);
	//}
}

void* btSmoothVehicleRaycaster::performLineTest(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result)
{
	SmoothRayCastResultCallback rayCallback(from, to);
	rayCallback.m_collisionFilterMask = this->m_collisionFilterMask;
	rayCallback.m_collisionFilterGroup = this->m_collisionFilterGroup;

	// If this is a rigid body, m_collision_object is NULL, and the
    // rigid body is the actual collision object.
    // btCollisionWorld::rayTestSingle(from, to, m_collision_object ? m_collision_object : body, m_collision_shape, world_trans, ray_callback);
	m_dynamicsWorld->rayTest(from, to, rayCallback);
	if (rayCallback.hasHit())
	{
		const btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
        if (body && body->hasContactResponse())
		{
			result.m_distFraction = rayCallback.m_closestHitFraction;
			result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
			result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
			result.m_hitNormalInWorld.normalize();

			if (this->m_interpolateNormals == true)
			{
				btCollisionShape* shape = (btCollisionShape*)body->getCollisionShape();
				if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
				{
					btTriangleMeshShape *mesh_shape = static_cast<btTriangleMeshShape *>(shape);
					btStridingMeshInterface *mesh_interface = mesh_shape->getMeshInterface();
					btSmoothTriangleMesh *mesh_smoothing = dynamic_cast<btSmoothTriangleMesh*>(mesh_interface);
					if (mesh_smoothing != NULL && mesh_smoothing->hasVertexNormals())
					{
						//btVector3 n1 = btVector3(result.m_hitNormalInWorld.x(), result.m_hitNormalInWorld.y(), result.m_hitNormalInWorld.z());
						result.m_hitNormalInWorld = mesh_smoothing->interpolateMeshNormal(body->getWorldTransform(), mesh_interface, rayCallback.m_shapePart, rayCallback.m_triangleIndex, rayCallback.m_hitPointWorld);
						//btVector3 n2 = btVector3(result.m_hitNormalInWorld.x(), result.m_hitNormalInWorld.y(), result.m_hitNormalInWorld.z());
						//printf("Perform Line Test - Hit Normal (%f x %f x %f) -> Bary Normal: (%f x %f x %f)\n", n1.x(), n1.y(), n1.z(), n2.x(), n2.y(), n2.z());
					}
				}
			}
			return (void*)body;
		}
	}
	return 0;
}

void* btSmoothVehicleRaycaster::performShapeTest(const btVector3& from, const btVector3& to, btVehicleRaycasterResult& result)
{
	SmoothShapeCastResultCallback convexCallback(from, to);
	convexCallback.m_collisionFilterMask = m_collisionFilterMask;
	convexCallback.m_collisionFilterGroup = m_collisionFilterGroup;

	btVector3 localFrom(0,0,0); // for convexSweepTest we need to have localized vectors
	btVector3 localTo((to.x() - from.x()), (to.y() - from.y()), (to.z() - from.z()));
	btTransform fromTransform, toTransform;
	fromTransform.setIdentity();
	fromTransform.setOrigin(localFrom);
	toTransform.setIdentity();
	toTransform.setOrigin(localTo);

	btConvexHullShape convexHullShape; 
	for (int i = 0; i < m_testPointCount; i++)
	{
		double angleRad = 360 / m_testPointCount * i * (M_PI / 180); // angle for each dot
		float x = from.x() + (m_shapeTestingSize + 1e-3f) * cos(angleRad);
		float y = from.y() + (m_shapeTestingSize + 1e-3f) * sin(angleRad) + m_shapeTestingSize * 2; // this modification is important 
		float z = from.z() + (m_shapeTestingSize + 1e-3f) * cos(angleRad);
		btVector3 point(x, y, z);
		convexHullShape.addPoint(point);
	}
	convexHullShape.setMargin(m_shapeTestingSize);
	m_dynamicsWorld->convexSweepTest(&convexHullShape, fromTransform, toTransform, convexCallback, this->m_sweepPenetration);
	if (convexCallback.hasHit())
	{
		const btRigidBody* body = btRigidBody::upcast(convexCallback.m_hitCollisionObject);
        if (body && body->hasContactResponse())
		{
			result.m_distFraction = convexCallback.m_closestHitFraction;
			result.m_hitPointInWorld = convexCallback.m_hitPointWorld;
			result.m_hitNormalInWorld = convexCallback.m_hitNormalWorld;
			result.m_hitNormalInWorld.normalize();

			if (this->m_interpolateNormals == true)
			{
				btCollisionShape* shape = (btCollisionShape*)body->getCollisionShape();
				if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
				{
					btTriangleMeshShape *mesh_shape = static_cast<btTriangleMeshShape *>(shape);
					btStridingMeshInterface *mesh_interface = mesh_shape->getMeshInterface();
					btSmoothTriangleMesh *mesh_smoothing = dynamic_cast<btSmoothTriangleMesh*>(mesh_interface);
					if (mesh_smoothing != NULL && mesh_smoothing->hasVertexNormals())
					{
						//btVector3 n1 = btVector3(result.m_hitNormalInWorld.x(), result.m_hitNormalInWorld.y(), result.m_hitNormalInWorld.z());
						result.m_hitNormalInWorld = mesh_smoothing->interpolateMeshNormal(body->getWorldTransform(), mesh_interface, convexCallback.m_shapePart, convexCallback.m_triangleIndex, convexCallback.m_hitPointWorld);
						//btVector3 n2 = btVector3(result.m_hitNormalInWorld.x(), result.m_hitNormalInWorld.y(), result.m_hitNormalInWorld.z());
						//printf("Perform Shape Test - Hit Normal (%f x %f x %f) -> Bary Normal: (%f x %f x %f)\n", n1.x(), n1.y(), n1.z(), n2.x(), n2.y(), n2.z());
					}
				}
			}
			return (void*)body;
		}
	}
	return 0;
}
