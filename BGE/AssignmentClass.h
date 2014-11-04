#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

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
		void Update(float timeDelta);
		void Cleanup();
		void CreateWall();
	};
}
