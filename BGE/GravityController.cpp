#include "GravityController.h"
#include "Utils.h"

using namespace BGE;

BGE::GravityController::GravityController()
{
}


BGE::GravityController::~GravityController()
{
}

void BGE::GravityController::Update(){
	transform->velocity += gravity * Time::deltaTime;
	transform->position += transform->velocity * Time::deltaTime;

	if (transform->position.y - transform->scale.x < 0){
		transform->velocity = -transform->velocity;
		transform->position.y += transform->scale.y;
	
	}

}