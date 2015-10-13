// #pragma once
// 
// #include "PxSetupEnv.h"
// #include "PxRigidBody.h"
// #include "iostream"
// #include <vector>
// 
// using namespace physx;
// using namespace std;
// 
// namespace Physics
// {
//     class CBodyNode
//     {
//         typedef vector<CBodyNode*> Childs;
// 
//     public:
//         CBodyNode(string Name_, PhysicsContext* PhysicsContext_, CBodyNode* fatherNode_, PxTransform localFrame_, PxTransform JointOffsetToFather_, Vector3f bodyGeometry_);
//         void CBodyNode::createNode(bool bi, float lowerJointLimitSagittal, float upperJointLimitSagittal);
//         PxD6Joint* getJoint()
//         {
//             return joint;
//         }
//         CRigidBody* getBody()
//         {
//             return Body;
//         }
//         PxTransform getGlobalFrame()
//         {
//             return globalFrame;
//         }
//         Vector3f getGeometry()
//         {
//             return bodyGeometry;
//         }
//         CBodyNode* getFatherNode()
//         {
//             return fatherNode;
//         }
//         Childs getChildList()
//         {
//             return childList;
//         }
//         void setChild(CBodyNode* childNode)
//         {
//             childList.push_back(childNode);
//             ++childCount;
//         }
//         bool isRootNode;
//         string Name;
// 
//     private:
//         int childCount;
//         Vector3f bodyGeometry;
//         PxD6Joint* joint;
//         CRigidBody* Body;
//         PxTransform localFrame;
//         PxTransform globalFrame;
//         PxTransform JointOffsetToFather;
//         CBodyNode* fatherNode;
//         typedef vector<CBodyNode*> Childs;
//         Childs childList;
//         PhysicsContext* mPhysicsContext;
//     };
// }