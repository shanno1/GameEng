#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>
#include "Utils.h"

namespace BGE
{
	class AssignmentClass :
	public Game
	{
	private:

	public:
		AssignmentClass(void);
		~AssignmentClass(void);
		bool Initialise();
		void Update(float Time);
		void Cleanup();
		void CreateWall(glm::vec3 startAt, float width, float height, float blockWidth, float blockHeight, float blockDepth);
		void CreateTree(glm::vec3 pos);

	};
}
