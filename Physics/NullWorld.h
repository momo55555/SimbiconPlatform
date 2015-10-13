#include "World.h"
#include "CollisionDetectionPrimitive.h"
#include "SphereCDP.h"
#include "CapsuleCDP.h"
#include "BoxCDP.h"
#include "PlaneCDP.h"
#include "PreCollisionQuery.h"

class NullWorld : public World{
};