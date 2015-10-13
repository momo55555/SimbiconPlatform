// /*
// =====================================================================
// 
// File Name	  :	RenderBuffer.h
// 
// Description	  : Functions in this header file are used for rendering the PhysX scene objects.
// 				It mainly renders primitives like- points, lines and triangles returned by
// 				the function 'PxScene::getRenderBuffer()'.
// 
// =====================================================================
// */
// 
// #pragma once  
// 
// #include <PxPhysicsAPI.h> //Single header file to include all features of PhysX API 
// #include <GL/freeglut.h>  //OpenGL window tool kit 
// 
// using namespace physx; 
// 
// 
// void RenderBuffer(float* pVertList, float* pColorList, int type, int num);
// void RenderData(const PxRenderBuffer & data);