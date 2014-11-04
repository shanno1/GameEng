#include "AssignmentClass.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

using namespace BGE;

AssignmentClass::AssignmentClass(void)
{
}

AssignmentClass::~AssignmentClass(void)
{
}


bool AssignmentClass::Initialise()
{	
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();	
	physicsFactory->CreateWall(glm::vec3(0, 1, 0),20.0f, 5.0f, 1.0f, 1.0f, 1.0f);
	if (!Game::Initialise()) {
		return false;
	}

	camera->transform->position = glm::vec3(0,10, 20);

	return true;
}

void BGE::AssignmentClass::Update(float timeDelta)
{

	Game::Update(timeDelta);
}

void BGE::AssignmentClass::Cleanup()
{
	Game::Cleanup();
}
void AssignmentClass::CreateWall(){
	
}