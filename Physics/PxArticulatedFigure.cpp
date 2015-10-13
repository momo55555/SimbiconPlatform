 #include "stdafx.h"
// 
// #include "PxArticulatedFigure.h"
// 
// using namespace Physics;
// 
// CArticulatedFigure::CArticulatedFigure(PhysicsContext* mPhysicsContext_)
// {
//     mPhysicsContext = mPhysicsContext_;
//     m_Aggregate = mPhysicsContext->getSDK()->createAggregate(MAX_BODY, false);
// }
// 
// void CArticulatedFigure::createFigure()
// {
//     PxTransform localframe(PxVec3(0.0, 25.2, 0.0), PxQuat(0.0, PxVec3(0, 0, 1)));
//     PxTransform jointOffset(PxVec3(0));
//     Vector3f geometry(3, 1.8, 1.0);
//     mBodyNodes[0] = new CBodyNode("Torso", mPhysicsContext, NULL, localframe, jointOffset, geometry);
//     mBodyNodes[0]->createNode(false, 0, 0);
// 
//     localframe = PxTransform(PxVec3(0.0, 2.1, 0));
//     jointOffset = PxTransform(PxVec3(0.0, 2.1, 0));
//     geometry = Vector3f(3, 1.8, 1.0);
//     mBodyNodes[1] = new CBodyNode("Pelvis", mPhysicsContext, mBodyNodes[0], localframe, jointOffset, geometry);
//     mBodyNodes[1]->createNode(true, -PxPiDivFour, PxHalfPi);
// 
//     mBodyNodes[0]->setChild(mBodyNodes[1]);
// 
//     localframe = PxTransform(PxVec3(1.2, 2.8, 0));
//     jointOffset = PxTransform(PxVec3(3.2, -1.5, 0));
//     geometry = Vector3f(1.0, 3.0, 1.0);
//     mBodyNodes[2] = new CBodyNode("LeftShoulder", mPhysicsContext, mBodyNodes[0], localframe, jointOffset, geometry);
//     mBodyNodes[2]->createNode(true, -PxHalfPi-PxPiDivFour, PxHalfPi);
// // 
//     localframe = PxTransform(PxVec3(-1.2, 2.8, 0));
//     jointOffset = PxTransform(PxVec3(-3.2, -1.5, 0));
//     geometry = Vector3f(1.0, 3.0, 1.0);
//     mBodyNodes[3] = new CBodyNode("RightShoulder", mPhysicsContext, mBodyNodes[0], localframe, jointOffset, geometry);
//     mBodyNodes[3]->createNode(true, -PxHalfPi-PxPiDivFour, PxHalfPi);
// //     
//     mBodyNodes[0]->setChild(mBodyNodes[2]);
//     mBodyNodes[0]->setChild(mBodyNodes[3]);
// // // // // 
//     localframe = PxTransform(PxVec3(0, 3.2, 0));
//     jointOffset = PxTransform(PxVec3(0, 3.2, 0));
//     geometry = Vector3f(1.0, 3.0, 1.0);
//     mBodyNodes[4] = new CBodyNode("LeftArm", mPhysicsContext, mBodyNodes[2], localframe, jointOffset, geometry);
//     mBodyNodes[4]->createNode(true, 0, PxHalfPi+PxPiDivFour);
// 
//     mBodyNodes[2]->setChild(mBodyNodes[4]);
// // 
//     localframe = PxTransform(PxVec3(0, 3.2, 0));
//     jointOffset = PxTransform(PxVec3(0, 3.2, 0));
//     geometry = Vector3f(1.0, 3.0, 1.0);
//     mBodyNodes[5] = new CBodyNode("RightArm", mPhysicsContext, mBodyNodes[3], localframe, jointOffset, geometry);
//     mBodyNodes[5]->createNode(false, 0, PxHalfPi+PxPiDivFour);
// 
//     mBodyNodes[3]->setChild(mBodyNodes[5]);
// // // 
//     localframe = PxTransform(PxVec3(0, 4.2, 0));
//     jointOffset = PxTransform(PxVec3(2.2, 2.0, 0));
//     geometry = Vector3f(1.0, 4.0, 1.0);
//     mBodyNodes[6] = new CBodyNode("LeftHip", mPhysicsContext, mBodyNodes[1], localframe, jointOffset, geometry);
//     mBodyNodes[6]->createNode(false, -PxHalfPi-PxPiDivFour, PxPiDivFour);
// 
//     localframe = PxTransform(PxVec3(0, 4.2, 0));
//     jointOffset = PxTransform(PxVec3(-2.2, 2.0, 0));
//     geometry = Vector3f(1.0, 4.0, 1.0);
//     mBodyNodes[7] = new CBodyNode("RightHip", mPhysicsContext, mBodyNodes[1], localframe, jointOffset, geometry);
//     mBodyNodes[7]->createNode(true, -PxHalfPi-PxPiDivFour, PxPiDivFour);
// 
//     mBodyNodes[1]->setChild(mBodyNodes[6]);
//     mBodyNodes[1]->setChild(mBodyNodes[7]);
// // // // 
//     localframe = PxTransform(PxVec3(0, 4.2, 0));
//     jointOffset = PxTransform(PxVec3(0, 4.2, 0));
//     geometry = Vector3f(1.0, 4.0, 1.0);
//     mBodyNodes[8] = new CBodyNode("LeftKnee", mPhysicsContext, mBodyNodes[6], localframe, jointOffset, geometry);
//     mBodyNodes[8]->createNode(false, -PxPi, PxPi);
// 
//     mBodyNodes[6]->setChild(mBodyNodes[8]);
// 
//     localframe = PxTransform(PxVec3(0, 4.2, 0));
//     jointOffset = PxTransform(PxVec3(0, 4.2, 0));
//     geometry = Vector3f(1.0, 4.0, 1.0);
//     mBodyNodes[9] = new CBodyNode("RightKnee", mPhysicsContext, mBodyNodes[7], localframe, jointOffset, geometry);
//     mBodyNodes[9]->createNode(false, -PxPi, PxPi);
// 
//     mBodyNodes[7]->setChild(mBodyNodes[9]);
// // // 
//     localframe = PxTransform(PxVec3(0, 1.2, 0));
//     jointOffset = PxTransform(PxVec3(0, 4.2, 0.0));
//     geometry = Vector3f(1.0, 1.0, 2.0);
//     mBodyNodes[10] = new CBodyNode("LeftAnkle", mPhysicsContext, mBodyNodes[8], localframe, jointOffset, geometry);
//     mBodyNodes[10]->createNode(true, -PxPi, PxPi);
// 
//     mBodyNodes[8]->setChild(mBodyNodes[10]);
// 
//     localframe = PxTransform(PxVec3(0, 1.2, 0));
//     jointOffset = PxTransform(PxVec3(0, 4.2, 0.0));
//     geometry = Vector3f(1.0, 1.0, 2.0);
//     mBodyNodes[11] = new CBodyNode("RightAnkle", mPhysicsContext, mBodyNodes[9], localframe, jointOffset, geometry);
//     mBodyNodes[11]->createNode(true, -PxPi, PxPi);
// 
//     mBodyNodes[9]->setChild(mBodyNodes[11]);
// 
//     localframe = PxTransform(PxVec3(0, -2.2, 0));
//     jointOffset = PxTransform(PxVec3(0, -2.0, 0.0));
//     geometry = Vector3f(2.0, 2.0, 2.0);
//     mBodyNodes[12] = new CBodyNode("Head", mPhysicsContext, mBodyNodes[0], localframe, jointOffset, geometry);
//     mBodyNodes[12]->createNode(true, -PxPi, PxPi);
// 
//     mBodyNodes[0]->setChild(mBodyNodes[12]);
// 
//     for(int i = 0; i < MAX_BODY; ++i)
//     {
//         m_Aggregate->addActor(*mBodyNodes[i]->getBody()->getRigidBodyEntity());
//     }
// 
//     //mPhysicsContext->getScene()->addAggregate(*m_Aggregate);
// }
// 
// void CArticulatedFigure::addAggregateToScene()
// {
//     mPhysicsContext->getScene()->addAggregate(*m_Aggregate);
// }