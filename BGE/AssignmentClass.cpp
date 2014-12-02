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
#include "GravityController.h"

using namespace BGE;

AssignmentClass::AssignmentClass(void)
{
}

AssignmentClass::~AssignmentClass(void)
{
}


bool AssignmentClass::Initialise()
{	
	setGravity(glm::vec3(0, 0, 0));
	//foot f_left, f_right,b_left, b_right
	//leg f_left, f_right,b_left, b_right
	//head
	//body top, flexjoint, back
	//tail 10ball joints decreasing in size.
	
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();	
	CreateWall(glm::vec3(0,35,0),30,10,1,1,1);
	if (!Game::Initialise()) {
		return false;
	}

	camera->transform->position = glm::vec3(0,10, 20);
	// A Slider
	//box1 = physicsFactory->CreateBox(1, 1, 4, glm::vec3(25, 5, 0), glm::quat());
	//box2 = physicsFactory->CreateBox(1, 1, 4, glm::vec3(25, 5, 5), glm::quat());
	btTransform box1Transform;
	btTransform box2Transform;
	box1Transform.setIdentity();
	box2Transform.setIdentity();
	shared_ptr<PhysicsController> sphere1 = physicsFactory->CreateSphere(5, glm::vec3(-5, 10, 0), glm::quat(), true, true);
	shared_ptr<PhysicsController> sphere = physicsFactory->CreateSphere(5, glm::vec3(5, 10, 0), glm::quat(), true, true);
	//shared_ptr<PhysicsController> sphere = physicsFactory->CreateSphere(float radius, glm::vec3(0,0,0)[position], glm::quat(), true, true)
	//shared_ptr<PhysicsController> box1 = physicsFactory->CreateBox(height, width, depth, glm::vec3(5,10, 0), glm::quat());
	btHingeConstraint * hinge = new btHingeConstraint(*sphere1->rigidBody, *sphere->rigidBody, btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);

	// A hinge
	//btHingeConstraint * hinge = new btHingeConstraint(*box1->rigidBody, *box2->rigidBody, btVector3(0, 0, 2.5f), btVector3(0, 0, -2.5f), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	//dynamicsWorld->addConstraint(hinge);
	//shared_ptr<PhysicsController> ball_joint = physicsFactory->CreateSphere(1.f,glm::vec3(0,10,0), glm::quat(),true,true);
	// Another hinge
	//shared_ptr<PhysicsController> box3 = physicsFactory->CreateBox(6, 5, 2, glm::vec3(15, 10, 0), glm::quat());
//	hinge = new btHingeConstraint(*box2->rigidBody, *box3->rigidBody, btVector3(0, 0, -6), btVector3(0, 3, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	//dynamicsWorld->addConstraint(hinge);
	
	//shared_ptr<PhysicsController> MIDJOINT = physicsFactory->CreateCylinder(.5f, .5f, glm::vec3(15, 10, 0), glm::angleAxis(90.0f, glm::vec3(0, 0, 1)));
	//hinge = new btHingeConstraint(*MIDJOINT->rigidBody,*box3->rigidBody , btVector3(0, 0, 1), btVector3(0, 3, 0), btVector3(0, 1,0), btVector3(0, 1, 0), true);
	//btPoint2PointConstraint * ptpConstraint = new btPoint2PointConstraint(*box3->rigidBody, *ball_joint->rigidBody, btVector3(0, 0, 2.5f), btVector3(0, 0, -2.5f));
	dynamicsWorld->addConstraint(hinge);
	//dynamicsWorld->addConstraint(hinge);

	// You have to make the x axis rotate to the axis you want to slide
	//box1Transform.setRotation(GLToBtQuat(glm::angleAxis(60.0f, glm::vec3(0, 1, 0))));
	//box2Transform.setRotation(GLToBtQuat(glm::angleAxis(60.0f, glm::vec3(0, 1, 0))));

	//btSliderConstraint * slider = new btSliderConstraint(*box1->rigidBody, *box2->rigidBody, box1Transform, box2Transform, true);
	//dynamicsWorld->addConstraint(slider);
	return Game::Initialise();
}

void BGE::AssignmentClass::Update(float deltaTime)
{

	Game::Update(deltaTime);
}

void BGE::AssignmentClass::Cleanup()
{
	Game::Cleanup();
}
void BGE::AssignmentClass::CreateWall(glm::vec3 startAt, float width, float height, float blockWidth, float blockHeight, float blockDepth){
	
	float z = startAt.z;


	for (int w = 0; w < width; w++)
	{
		for (int h = 0; h < height; h++)
		{
			float x = startAt.x + ((blockWidth + .1f) * w);
			float y = (blockHeight / 2.0f) + (blockHeight * h);
			physicsFactory->CreateBox(blockWidth, blockHeight, blockDepth, glm::vec3(x, y, z), glm::quat());
		}
	}
}