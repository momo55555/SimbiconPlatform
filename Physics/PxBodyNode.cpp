 #include "stdafx.h"
// 
// #include "PxBodyNode.h"
// 
// using namespace Physics;
// 
// CBodyNode::CBodyNode(string Name_, PhysicsContext* PhysicsContext_, CBodyNode* fatherNode_, PxTransform localFrame_, PxTransform JointOffsetToFather_, Vector3f bodyGeometry_)
// {
//     isRootNode = false;
//     if(fatherNode_ == NULL)
//     {
//         isRootNode = true;
//     }
//     
//     Name = Name_;
//     mPhysicsContext = PhysicsContext_;
//     fatherNode = fatherNode_;
//     localFrame = localFrame_;
//     JointOffsetToFather = JointOffsetToFather_;
//     bodyGeometry = bodyGeometry_;
//     childList.clear();
//     childCount = 0;
// }
// 
// void CBodyNode::createNode(bool bi, float lowerJointLimitSagittal, float upperJointLimitSagittal)
// {
//     globalFrame = localFrame;
//     if(!isRootNode)
//     {
//         globalFrame = fatherNode->getGlobalFrame() * JointOffsetToFather.getInverse() * localFrame.getInverse();    
//     }
// 
//     Body = new CRigidBody(mPhysicsContext);
//     if(!isRootNode) Body->createRigidBody(globalFrame, bodyGeometry, 0.6f, 0.6f, 0.0001f, false);
//     else  Body->createRigidBody(globalFrame, bodyGeometry, 0.5f, 0.5f, 0.0001f, true);
//     //Body->getRigidBodyEntity()->setSolverIterationCounts(8, 2);
// 
//     if(isRootNode)
//     {
//     }
//     //Body->getRigidBodyEntity()->setMass(0.01);
// 
//     if(!isRootNode)
//     {
//         PxRigidActor* fatherBody = fatherNode->getBody()->getRigidBodyEntity();
//         joint = PxD6JointCreate(*mPhysicsContext->getSDK(), fatherBody, PxTransform(-JointOffsetToFather.p), Body->getRigidBodyEntity(), PxTransform(localFrame));
//         
//         if(joint == NULL)
//         {
//             exit(1);
//         }
// 
//         joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
//         //joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
// 
//         joint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
//         joint->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
//         joint->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
//         joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
//         joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
//         joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
// 
//         PxJointAngularLimitPair m_AngularPair = PxJointAngularLimitPair(lowerJointLimitSagittal, upperJointLimitSagittal, 0.1);
//         m_AngularPair.damping = 100000000000;
//         m_AngularPair.stiffness = 0.000000000001;
//         m_AngularPair.restitution = 0;
//         bool flag = m_AngularPair.isSoft();
// 
//         joint->setTwistLimit(m_AngularPair);
// 
//         if(bi)
//         {
//             // I don't know how to constraint these joints yet...
//             //joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
//             //joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
//         }
//         else
//         {
// //             joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLOCKED);
// //             joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);
//         }
//     }
// }