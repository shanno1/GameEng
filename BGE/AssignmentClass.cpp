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
#include <stdio.h>      
#include <stdlib.h>     
#include <time.h>
#include "Utils.h"


using namespace BGE;

AssignmentClass::AssignmentClass(void)
{
}

AssignmentClass::~AssignmentClass(void)
{
}
shared_ptr<PhysicsController> BACK_LEFT_LEG,BODY_FRONT;
shared_ptr<PhysicsController> FRONT_LEFT_LEG;
shared_ptr<PhysicsController> BACK_RIGHT_LEG;
shared_ptr<PhysicsController> FRONT_RIGHT_LEG;
shared_ptr<PhysicsController> TAIL,TAIL2,HEAD, SUN,SLED;
bool isWalking;
bool Stepping;
btHingeConstraint * hinge;

bool AssignmentClass::Initialise()
{	
	setGravity(glm::vec3(0, -9.8, 0));
	
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();	

	btTransform box1Transform;
	btTransform box2Transform;
	box1Transform.setIdentity();
	box2Transform.setIdentity();
	
	
	float bodywidth = 5, bodyheight = 1, bodydepth = 3;
	float legwidth = 1, legheight = 3, legdepth = 1;
	float tailwidth = 3, tailheight = 1, taildepth = 1;
	glm::vec3 position = glm::vec3(0, 10, 0);
	glm::vec3 offset;
	
	
	btTransform BODYTransform;
	btTransform BODY2Transform;
	
	Stepping = true;
	isWalking = true;

	// You have to make the x axis rotate to the axis you want to slide
	BODYTransform.setRotation(GLToBtQuat(glm::angleAxis(20.0f, glm::vec3(0, 0, 1))));
	BODY2Transform.setRotation(GLToBtQuat(glm::angleAxis(20.0f, glm::vec3(0, 0, 1))));

	//BODY FRONT -----------------------------------------------------------------
	BODY_FRONT = physicsFactory->CreateBox(bodywidth, bodyheight, bodydepth, position, glm::quat());
	//----------------------------------------------------------------------------
	
btFixedConstraint *hinge2;
	btTransform box1;
	btTransform box2;
	box1.setIdentity();
	box2.setIdentity();
	box1.setOrigin(btVector3(-(bodywidth+1), 0, 0));

	//BACK BODY -------------------------------------------------------------------
	offset = glm::vec3((-bodywidth), 0, 0);	
	shared_ptr<PhysicsController>BODY_BACK = physicsFactory->CreateBox(bodywidth, bodyheight, bodydepth, position+offset, glm::quat());
	hinge2 = new btFixedConstraint(*BODY_FRONT->rigidBody, *BODY_BACK->rigidBody,box1,box2);
	dynamicsWorld->addConstraint(hinge2);
	//-----------------------------------------------------------------------------
	
	//TAIL ------------------------------------------------------------------------
	offset = glm::vec3(-(bodywidth-.5f) , 0,0 );
	TAIL = physicsFactory->CreateBox(tailwidth, tailheight, taildepth, position + offset, glm::quat());
	hinge = new btHingeConstraint(*BODY_BACK->rigidBody, *TAIL->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(1, 0, 0), btVector3(1,0, 0), true);
	dynamicsWorld->addConstraint(hinge);
	//------------------------------------------------------------------------------

	//TAIL2 -----------------------------------------------------------------------
	offset = glm::vec3(-(bodywidth - .8f ), 0, 0);
	TAIL2 = physicsFactory->CreateBox(tailwidth, tailheight, taildepth, position + offset, glm::quat());
	hinge = new btHingeConstraint(*TAIL->rigidBody, *TAIL2->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(1, 0,0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(hinge);
	//------------------------------------------------------------------------------
	
	//LEFT BACK LEG ---------------------------------------------------------------
	offset = glm::vec3(-(bodywidth + (bodywidth*.8) - legdepth-5), -(legheight / 2), -(bodywidth / 2));
	BACK_LEFT_LEG = physicsFactory->CreateBox(legwidth, legheight, legdepth, position + offset, glm::quat());
	hinge = new btHingeConstraint(*BODY_BACK->rigidBody, *BACK_LEFT_LEG->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(1, 0,0), btVector3(1, 1,1), true);
	hinge->setLimit(.5, .9);
	dynamicsWorld->addConstraint(hinge);
	//------------------------------------------------------------------------------
	
	//LEFT FRONT LEG --------------------------------------------------------------
	offset = glm::vec3(+(bodywidth / 2 - legdepth+1), -(legheight / 2), -(bodywidth / 2));
	FRONT_LEFT_LEG = physicsFactory->CreateBox(legwidth, legheight, legdepth, position + offset, glm::quat());
	hinge = new btHingeConstraint(*BODY_FRONT->rigidBody, *FRONT_LEFT_LEG->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(1, 0,0), btVector3(1, 0, 1), true);
	hinge->setLimit(.5, .9);
	dynamicsWorld->addConstraint(hinge);
	//-------------------------------------------------------------------------------

	//RIGHT FRONT LEG ---------------------------------------------------------------
	offset = glm::vec3(+(bodywidth / 2 - legdepth+1), - (legheight / 2), +(bodywidth / 2 ));
	FRONT_RIGHT_LEG = physicsFactory->CreateBox(legwidth, legheight, legdepth, position + offset, glm::quat());
	hinge = new btHingeConstraint(*BODY_FRONT->rigidBody, *FRONT_RIGHT_LEG->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(1, 0,0), btVector3(1, 0,1), true);
	hinge->setLimit(-1.1, -.7);
	
	dynamicsWorld->addConstraint(hinge);
	//------------------------------------------------------------------------------

	//RIGHT BACK LEG ---------------------------------------------------------------
	offset = glm::vec3(-(bodywidth+(bodywidth*.8) - legdepth-5), -(legheight / 2), +(bodywidth / 2));
	BACK_RIGHT_LEG = physicsFactory->CreateBox(legwidth, legheight, legdepth, position + offset, glm::quat());
	hinge = new btHingeConstraint(*BODY_BACK->rigidBody, *BACK_RIGHT_LEG->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(1,0,0), btVector3(1,0 , 1), true);
	hinge->setLimit(-1.1, -.7);
	dynamicsWorld->addConstraint(hinge);
	//------------------------------------------------------------------------------

	//HEAD -------------------------------------------------------------------------
	offset = glm::vec3(+(bodywidth/2+1), 1, 0);
	HEAD = physicsFactory->CreateBox(2,2,2, position + offset, glm::quat());
	hinge = new btHingeConstraint(*BODY_FRONT->rigidBody, *HEAD->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(1, 0, 0), btVector3(1, 0, 1), true);
	dynamicsWorld->addConstraint(hinge);
	//------------------------------------------------------------------------------
	

	//Colouring the rigid bodys
	BACK_RIGHT_LEG->transform->diffuse = glm::vec3(.6, .5, .35);
	BACK_LEFT_LEG->transform->diffuse = glm::vec3(.6, .5, .35);
	FRONT_LEFT_LEG->transform->diffuse = glm::vec3(.6, .5, .35);
	FRONT_RIGHT_LEG->transform->diffuse = glm::vec3(.6, .5, .35);
	BODY_BACK->transform->diffuse = glm::vec3(.6, .5, .35);
	BODY_FRONT->transform->diffuse = glm::vec3(.6, .5, .35);
	TAIL->transform->diffuse = glm::vec3(.6, .5, .35);
	TAIL2->transform->diffuse = glm::vec3(.6, .5, .35);
	HEAD->transform->diffuse = glm::vec3(.6, .5, .35);

	//The Sun
	SUN = physicsFactory->CreateSphere(10,glm::vec3(35,200,-140),glm::quat(),true,true);
	SUN->transform->diffuse = glm::vec3(1, 1, .1);
	SUN->transform->ambient = glm::vec3(.8, .8, .8);

	hinge2 = new btFixedConstraint(*BODY_FRONT->rigidBody, *BODY_BACK->rigidBody, box1, box2);


	//Random tree generator
	for (int i = 0; i < 25; i++) {
		CreateTree(glm::vec3((rand() % 500), 0, (rand() % 500)));
		CreateTree(glm::vec3(-(rand() % 500), 0, -(rand() % 500)));
	}

	if (!Game::Initialise()) {
		return false;
	}

	camera->transform->position = glm::vec3(0, 10, 20);
	return Game::Initialise();
}
float delta;
void BGE::AssignmentClass::Update(float deltaTime)
{
	//walking not working 
	if (isWalking && Stepping)
	{
		FRONT_RIGHT_LEG->rigidBody->applyForce(GLToBtVector(glm::vec3(0.0f, 0.0f, 1.0f)), GLToBtVector(glm::vec3(0.0f, 0.0f, 1.0f)));
		BACK_LEFT_LEG->rigidBody->applyForce(GLToBtVector(glm::vec3(0.0f, 0.0f, 1.0f)), GLToBtVector(glm::vec3(0.0f, 0.0f, 1.0f)));
		Stepping = false;
	}
	else if (isWalking && !Stepping)
	{
		BACK_RIGHT_LEG->rigidBody->applyForce(GLToBtVector(glm::vec3(0.0f, 0.0f, 1.0f)),GLToBtVector(glm::vec3(0.0f, 0.0f, 1.0f)));
		FRONT_LEFT_LEG->rigidBody->applyForce(GLToBtVector(glm::vec3(0.0f, 0.0f, 1.0f)),GLToBtVector(glm::vec3(0.0f, 0.0f, 1.0f)));
		Stepping = true;
	}
	
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
void BGE::AssignmentClass::CreateTree(glm::vec3 pos){
	glm::vec3 offset = glm::vec3(0,17,0);
	shared_ptr<PhysicsController> TreeTrunk;
	btTransform box1;
	btTransform box2;
	box1.setIdentity();
	box2.setIdentity();
	TreeTrunk = physicsFactory->CreateBox(2, 15, 2, pos, glm::quat());
	TreeTrunk->transform->diffuse = glm::vec3(.5, .11, .11);

	shared_ptr<PhysicsController> TreeBranch = physicsFactory->CreateBox(5, 5, 5, pos + offset, glm::quat());
	TreeBranch->transform->diffuse = glm::vec3(.15,.5,.11);
	btFixedConstraint *hinge2 = new btFixedConstraint(*TreeTrunk->rigidBody, *TreeBranch->rigidBody, box1, box2);
	
}