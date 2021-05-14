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
#ifndef BT_RAYCASTVEHICLE_H
#define BT_RAYCASTVEHICLE_H

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "btVehicleRaycaster.h"
class btDynamicsWorld;
#include "LinearMath/btAlignedObjectArray.h"
#include "btWheelInfo.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"

class btVehicleTuning;

// Mackey Kinard
class btConvexHullShape;
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h" 

///rayCast vehicle, very special constraint that turn a rigidbody into a vehicle.
class btRaycastVehicle : public btActionInterface
{

		btAlignedObjectArray<btVector3>	m_forwardWS;
		btAlignedObjectArray<btVector3>	m_axle;
		btAlignedObjectArray<btScalar>	m_forwardImpulse;
		btAlignedObjectArray<btScalar>	m_sideImpulse;
	
		///backwards compatibility
		int	m_userConstraintType;
		int	m_userConstraintId;

public:
	class btVehicleTuning
		{
			public:

			btVehicleTuning()
				:m_suspensionStiffness(btScalar(5.88)),
				m_suspensionCompression(btScalar(0.83)),
				m_suspensionDamping(btScalar(0.88)),
				m_maxSuspensionTravelCm(btScalar(500.)),
				m_frictionSlip(btScalar(10.5)),
				m_maxSuspensionForce(btScalar(6000.))
			{
			}
			btScalar	m_suspensionStiffness;
			btScalar	m_suspensionCompression;
			btScalar	m_suspensionDamping;
			btScalar	m_maxSuspensionTravelCm;
			btScalar	m_frictionSlip;
			btScalar	m_maxSuspensionForce;

		};
private:

	btScalar	m_tau;
	btScalar	m_damping;
	btVehicleRaycaster*	m_vehicleRaycaster;
	btScalar		m_pitchControl;
	btScalar	m_steeringValue; 
	btScalar m_currentVehicleSpeedKmHour;

	btRigidBody* m_chassisBody;

	int m_indexRightAxis;
	int m_indexUpAxis;
	int	m_indexForwardAxis;

	void defaultInit(const btVehicleTuning& tuning);

public:

	//constructor to create a car from an existing rigidbody
	btRaycastVehicle(const btVehicleTuning& tuning,btRigidBody* chassis,	btVehicleRaycaster* raycaster );

	virtual ~btRaycastVehicle() ;

	// Mackey Kinard
	bool m_enableMultiRaycast;
	int m_minimumWheelContacts;
	btScalar m_trackConnectionAccel;
	btScalar m_smoothFlyingImpulse;
	btScalar m_stabilizingForce;
	btScalar m_maxImpulseForce;

	///btActionInterface interface
	virtual void updateAction( btCollisionWorld* collisionWorld, btScalar step)
	{
        (void) collisionWorld;
		updateVehicle(step);
	}
	

	///btActionInterface interface
	void	debugDraw(btIDebugDraw* debugDrawer);
			
	const btTransform& getChassisWorldTransform() const;
	
	//btScalar rayCast(btWheelInfo& wheel);
	// Mackey Kinard (Note: Raycast Fraction - Helps Collision Mesh Gaps)
	btScalar rayCast(btWheelInfo& wheel, float fraction = 1.0f);

	virtual void updateVehicle(btScalar step);
	
	
	void resetSuspension();

	btScalar	getSteeringValue(int wheel) const;

	void	setSteeringValue(btScalar steering,int wheel);


	void	applyEngineForce(btScalar force, int wheel);

	const btTransform&	getWheelTransformWS( int wheelIndex ) const;

	void	updateWheelTransform( int wheelIndex, bool interpolatedTransform = true );
	
//	void	setRaycastWheelInfo( int wheelIndex , bool isInContact, const btVector3& hitPoint, const btVector3& hitNormal,btScalar depth);

	btWheelInfo&	addWheel( const btVector3& connectionPointCS0, const btVector3& wheelDirectionCS0,const btVector3& wheelAxleCS,btScalar suspensionRestLength,btScalar wheelRadius,const btVehicleTuning& tuning, bool isFrontWheel);

	inline int		getNumWheels() const {
		return int (m_wheelInfo.size());
	}
	
	btAlignedObjectArray<btWheelInfo>	m_wheelInfo;


	const btWheelInfo&	getWheelInfo(int index) const;

	btWheelInfo&	getWheelInfo(int index);

	//void	updateWheelTransformsWS(btWheelInfo& wheel , bool interpolatedTransform = true);
	// Mackey Kinard (Note: Raycast Fraction - Helps Collision Mesh Gaps)
	void updateWheelTransformsWS(btWheelInfo& wheel , bool interpolatedTransform = true, float raycastFraction = 1.0f);

	void setBrake(btScalar brake,int wheelIndex);

	void	setPitchControl(btScalar pitch)
	{
		m_pitchControl = pitch;
	}
	
	void	updateSuspension(btScalar deltaTime);

	virtual void	updateFriction(btScalar	timeStep);



	inline btRigidBody* getRigidBody()
	{
		return m_chassisBody;
	}

	const btRigidBody* getRigidBody() const
	{
		return m_chassisBody;
	}

	inline int	getRightAxis() const
	{
		return m_indexRightAxis;
	}
	inline int getUpAxis() const
	{
		return m_indexUpAxis;
	}

	inline int getForwardAxis() const
	{
		return m_indexForwardAxis;
	}

	
	///Worldspace forward vector
	btVector3 getForwardVector() const
	{
		const btTransform& chassisTrans = getChassisWorldTransform(); 

		btVector3 forwardW ( 
			  chassisTrans.getBasis()[0][m_indexForwardAxis], 
			  chassisTrans.getBasis()[1][m_indexForwardAxis], 
			  chassisTrans.getBasis()[2][m_indexForwardAxis]); 

		return forwardW;
	}

	///Velocity of vehicle (positive if velocity vector has same direction as foward vector)
	btScalar	getCurrentSpeedKmHour() const
	{
		return m_currentVehicleSpeedKmHour;
	}

	virtual void	setCoordinateSystem(int rightIndex,int upIndex,int forwardIndex)
	{
		m_indexRightAxis = rightIndex;
		m_indexUpAxis = upIndex;
		m_indexForwardAxis = forwardIndex;
	}


	///backwards compatibility
	int getUserConstraintType() const
	{
		return m_userConstraintType ;
	}

	void	setUserConstraintType(int userConstraintType)
	{
		m_userConstraintType = userConstraintType;
	};

	void	setUserConstraintId(int uid)
	{
		m_userConstraintId = uid;
	}

	int getUserConstraintId() const
	{
		return m_userConstraintId;
	}

};

class btDefaultVehicleRaycaster : public btVehicleRaycaster
{
	btDynamicsWorld*	m_dynamicsWorld;
public:
	btDefaultVehicleRaycaster(btDynamicsWorld* world)
		:m_dynamicsWorld(world)
	{
	}

	virtual void* castRay(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result);

};

////////////////////////////////////////////////////
// Mackey Kinard - Smooth Vehicle Raycaster
////////////////////////////////////////////////////

// Write a ray result callback that saves the shapePart and triangleIndex
struct SmoothRayCastResultCallback : public btCollisionWorld::RayResultCallback
{
	SmoothRayCastResultCallback(const btVector3 &rayFromWorld, const btVector3 &rayToWorld) : m_rayFromWorld(rayFromWorld), m_rayToWorld(rayToWorld) { }

	//used to calculate hitPointWorld from hitFraction
	btVector3 m_rayFromWorld; 
	btVector3 m_rayToWorld;

	btVector3 m_hitNormalWorld;
	btVector3 m_hitPointWorld;

	int m_shapePart;
	int m_triangleIndex;

	virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
	{
		btAssert(rayResult.m_hitFraction <= m_closestHitFraction);

		if (rayResult.m_localShapeInfo)
		{
			m_shapePart = rayResult.m_localShapeInfo->m_shapePart;
			m_triangleIndex = rayResult.m_localShapeInfo->m_triangleIndex;
		}

		m_collisionObject = rayResult.m_collisionObject;
		m_closestHitFraction = rayResult.m_hitFraction;
		m_hitPointWorld.setInterpolate3(m_rayFromWorld, m_rayToWorld, rayResult.m_hitFraction);
		if (normalInWorldSpace)
		{
			m_hitNormalWorld = rayResult.m_hitNormalLocal;
		}
		else
		{
			m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis() * rayResult.m_hitNormalLocal; // transform normal into worldspace
		}
		return m_closestHitFraction;
	}
};

// Write a convex result callback that saves the shapePart and triangleIndex
struct	SmoothShapeCastResultCallback : public btCollisionWorld::ConvexResultCallback
{
	SmoothShapeCastResultCallback(const btVector3&	convexFromWorld,const btVector3& convexToWorld) : m_convexFromWorld(convexFromWorld), m_convexToWorld(convexToWorld), m_hitCollisionObject(0) { }

	//used to calculate hitPointWorld from hitFraction
	btVector3	m_convexFromWorld;
	btVector3	m_convexToWorld;

	btVector3	m_hitNormalWorld;
	btVector3	m_hitPointWorld;
	const btCollisionObject*	m_hitCollisionObject;
	
	int m_shapePart;
	int m_triangleIndex;

	virtual	btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace)
	{
		btAssert(convexResult.m_hitFraction <= m_closestHitFraction);

		if (convexResult.m_localShapeInfo)
		{
			m_shapePart = convexResult.m_localShapeInfo->m_shapePart;
			m_triangleIndex = convexResult.m_localShapeInfo->m_triangleIndex;
		}

		m_hitCollisionObject = convexResult.m_hitCollisionObject;
		m_closestHitFraction = convexResult.m_hitFraction;
		m_hitPointWorld.setInterpolate3(m_convexFromWorld, m_convexToWorld, convexResult.m_hitFraction);
		if (normalInWorldSpace)
		{
			m_hitNormalWorld = convexResult.m_hitNormalLocal;
		}
		else
		{
			m_hitNormalWorld = m_hitCollisionObject->getWorldTransform().getBasis() * convexResult.m_hitNormalLocal; // transform normal into worldspace
		}

		return m_closestHitFraction;
	}
};

class btSmoothVehicleRaycaster : public btVehicleRaycaster
{
	btDynamicsWorld*	m_dynamicsWorld;
public:
	short int m_collisionFilterGroup;
	short int m_collisionFilterMask;
	bool m_interpolateNormals;
	bool m_shapeTestingMode;
	int m_testPointCount;
	btScalar m_sweepPenetration;
	btScalar m_shapeTestingSize;

	btSmoothVehicleRaycaster(btDynamicsWorld* world) : m_dynamicsWorld(world)
	{
		m_interpolateNormals = false;
		m_shapeTestingMode = false;
		m_testPointCount = 32; 					// shall be enough, but can be higher
		m_shapeTestingSize = btScalar(0.2f); 	// should be half of wheel radius
		m_sweepPenetration = btScalar(0.0f); 	// default to zero
		m_collisionFilterGroup = btBroadphaseProxy::DefaultFilter;
		m_collisionFilterMask = btBroadphaseProxy::StaticFilter;
	}

	virtual void* castRay(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result);
	void* performLineTest(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result);
	void* performShapeTest(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result);
};

#endif //BT_RAYCASTVEHICLE_H

