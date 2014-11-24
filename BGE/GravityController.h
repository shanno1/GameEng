#pragma once
#include "Game.h"
#include "GameComponent.h"
#include "Utils.h"
namespace BGE{

	class GravityController :public GameComponent
	{
	public:
		GravityController();
		~GravityController();

		void Update();
		bool Initialise();

		glm::vec3 gravity = glm::vec3(0, -9.8, 0);
	};

}