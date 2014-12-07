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
shared_ptr<PhysicsController> BACK_LEFT_LEG;
shared_ptr<PhysicsController> FRONT_LEFT_LEG;
shared_ptr<PhysicsController> BACK_RIGHT_LEG;
shared_ptr<PhysicsController> FRONT_RIGHT_LEG;
bool isWalking;
bool Stepping;
btHingeConstraint * hinge;

bool AssignmentClass::Initialise()
{	
	setGravity(glm::vec3(0, -9.8, 0));
	
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();	
	//CreateWall(glm::vec3(0,35,0),30,10,1,1,1);
	
	// A Slider
	//box1 = physicsFactory->CreateBox(1, 1, 4, glm::vec3(25, 5, 0), glm::quat());
	//box2 = physicsFactory->CreateBox(1, 1, 4, glm::vec3(25, 5, 5), glm::quat());
	btTransform box1Transform;
	btTransform box2Transform;
	box1Transform.setIdentity();
	box2Transform.setIdentity();
	
	
	float bodywidth = 5, bodyheight = 1, bodydepth = 3;
	float legwidth = 1, legheight = 3, legdepth = 1;
	float footwidth = 2, footheight = 1, footdepth = 1;
	glm::vec3 position = glm::vec3(0, 10, 0);
	glm::vec3 offset;
	glm::quat q = glm::angleAxis(glm::quarter_pi<float>(), glm::vec3(0, 0, 1));
	btTransform BODYTransform;
	btTransform BODY2Transform;
	
	Stepping = true;
	isWalking = true;
	// You have to make the x axis rotate to the axis you want to slide
	BODYTransform.setRotation(GLToBtQuat(glm::angleAxis(20.0f, glm::vec3(0, 0, 1))));
	BODY2Transform.setRotation(GLToBtQuat(glm::angleAxis(20.0f, glm::vec3(0, 0, 1))));
	shared_ptr<PhysicsController> BODY_FRONT = physicsFactory->CreateBox(bodywidth, bodyheight, bodydepth, position, glm::quat());
	btFixedConstraint *hinge2;
	btTransform box1;
	btTransform box2;
	box1.setIdentity();
	box2.setIdentity();
	box1.setOrigin(btVector3(-(bodywidth+1), 0, 0));
	//box1.setRotation(GLToBtQuat(glm::angleAxis(10.0f, glm::vec3(0,0 , 1))));
	//box1.setRotation(GLToBtQuat(glm::angleAxis(90.0f, glm::vec3(0, 1, 0))));
	//box2.setRotation(GLToBtQuat(glm::angleAxis(180.0f, glm::vec3(1, 0,0))));

	offset = glm::vec3((-bodywidth), 0, 0);	
	shared_ptr<PhysicsController>BODY_BACK = physicsFactory->CreateBox(bodywidth, bodyheight, bodydepth, position+offset, glm::quat());
	hinge2 = new btFixedConstraint(*BODY_FRONT->rigidBody, *BODY_BACK->rigidBody,box1,box2);
	dynamicsWorld->addConstraint(hinge2);
	//shared_ptr<PhysicsController> FRONT_LEFT_LEG = physicsFactory->CreateBox(legwidth, legdepth, legheight,(bodywidth/2), glm::quat());
	//shared_ptr<PhysicsController> FRONT_RIGHT_LEG = physicsFactory->CreateBox(legwidth, legdepth, legheight, glm::vec3(-1, 10, -1.5), glm::quat());


	offset = glm::vec3(-(bodywidth + (bodywidth*.8) - legdepth-5), -(legheight / 2), -(bodywidth / 2));
	BACK_LEFT_LEG = physicsFactory->CreateBox(legwidth, legheight, legdepth, position+offset, glm::quat());
	hinge = new btHingeConstraint(*BODY_BACK->rigidBody, *BACK_LEFT_LEG->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0,1), btVector3(1, 0, 0), true);
	hinge->setLimit(.1, .5);
	dynamicsWorld->addConstraint(hinge);
	
	offset = glm::vec3(+(bodywidth / 2 - legdepth+1), -(legheight / 2), -(bodywidth / 2));
	FRONT_LEFT_LEG = physicsFactory->CreateBox(legwidth, legheight, legdepth, position + offset, glm::quat());
	hinge = new btHingeConstraint(*BODY_FRONT->rigidBody, *FRONT_LEFT_LEG->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(1, 0, 0), true);
	hinge->setLimit(.1, .5);
	dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(+(bodywidth / 2 - legdepth+1), - (legheight / 2), +(bodywidth / 2 ));
	FRONT_RIGHT_LEG = physicsFactory->CreateBox(legwidth, legheight,legdepth, position + offset, q);
	hinge = new btHingeConstraint(*BODY_FRONT->rigidBody, *FRONT_RIGHT_LEG->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0,1), btVector3(1, 0, 0), true);
	hinge->setLimit(.1, .5);
	dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(-(bodywidth+(bodywidth*.8) - legdepth-5), -(legheight / 2), +(bodywidth / 2));
	BACK_RIGHT_LEG = physicsFactory->CreateBox(legwidth, legheight, legdepth, position + offset, q);
	hinge = new btHingeConstraint(*BODY_BACK->rigidBody, *BACK_RIGHT_LEG->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0,1), btVector3(1, 0, 0), true);
	hinge->setLimit(.1, .5);
	dynamicsWorld->addConstraint(hinge);

	//shared_ptr<PhysicsController> FRONT_RIGHT_FOOT = physicsFactory->CreateBox(footwidth, footdepth, footheight, glm::vec3(0, 10, 0), glm::quat());
//	shared_ptr<PhysicsController> BACK_LEFT_FOOT = physicsFactory->CreateBox(footwidth, footdepth, footheight, glm::vec3(0, 10, 0), glm::quat());
//	shared_ptr<PhysicsController> BACK_RIGHT_FOOT = physicsFactory->CreateBox(footwidth, footdepth, footheight, glm::vec3(0, 10, 0), glm::quat());
//	shared_ptr<PhysicsController> sphere_front = physicsFactory->CreateSphere(2, glm::vec3(2, 10, 0), glm::quat(), false, true);
//	shared_ptr<PhysicsController> sphere_head = physicsFactory->CreateSphere(1, glm::vec3(4, 11, 0), glm::quat(), false, true);

//	hinge = new btHingeConstraint(*sphere_front->rigidBody, *sphere_back->rigidBody, btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
//	dynamicsWorld->addConstraint(hinge);
	//	
//	dynamicsWorld->addConstraint(hinge);
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

	//dynamicsWorld->addConstraint(hinge);

	// You have to make the x axis rotate to the axis you want to slide
	//box1Transform.setRotation(GLToBtQuat(glm::angleAxis(60.0f, glm::vec3(0, 1, 0))));
	//box2Transform.setRotation(GLToBtQuat(glm::angleAxis(60.0f, glm::vec3(0, 1, 0))));

	//btSliderConstraint * slider = new btSliderConstraint(*box1->rigidBody, *box2->rigidBody, box1Transform, box2Transform, true);
	//dynamicsWorld->addConstraint(slider);
	if (!Game::Initialise()) {
		return false;
	}
	camera->transform->position = glm::vec3(0, 10, 20);
	return Game::Initialise();
}

void BGE::AssignmentClass::Update(float deltaTime)
{

	if (isWalking && Stepping)
	{ 
		FRONT_RIGHT_LEG->rigidBody->applyTorque(GLToBtVector(glm::vec3(0.0f, 0.0f, 1.0f)));
		BACK_LEFT_LEG->rigidBody->applyTorque(GLToBtVector(glm::vec3(0.0f, 0.0f, 1.0f)));
	}
	else if (isWalking && !Stepping)
	{ 
		BACK_RIGHT_LEG->rigidBody->applyTorque(GLToBtVector(glm::vec3(0.0f, 0.0f, 1.0f)));
		FRONT_LEFT_LEG->rigidBody->applyTorque(GLToBtVector(glm::vec3(0.0f, 0.0f, 1.0f)));
	}
	if (hinge->getLowerLimit() == .5f && hinge->getRigidBodyB() == *){
		Stepping = true;
	}
	else if (hinge->getLowerLimit() == .4f){
		Stepping = false;
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