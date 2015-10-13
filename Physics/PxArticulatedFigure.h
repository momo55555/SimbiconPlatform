// #pragma once
// 
// #include "PxBodyNode.h"
// 
// #define MAX_BODY 13
// 
// namespace Physics
// {
//     class CArticulatedFigure
//     {
//     public:
//         CArticulatedFigure(PhysicsContext* mPhysicsContext_);
//         ~CArticulatedFigure()
//         {
//             m_Aggregate->release();
//         }
//         void createFigure();
//         void CArticulatedFigure::addAggregateToScene();
//         CBodyNode* mBodyNodes[MAX_BODY];
//     
//     private:       
//         PhysicsContext* mPhysicsContext;
//         PxAggregate* m_Aggregate;
//     };
// }