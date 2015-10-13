#include "stdafx.h"

// #include "stdafx.h"
// 
// #include "PhysXVisualization.h"
// 
// void RenderBuffer(float* pVertList, float* pColorList, int type, int num)
// {
//     glEnableClientState(GL_VERTEX_ARRAY);
//     glVertexPointer(3, GL_FLOAT, 0, pVertList);
// 
//     glEnableClientState(GL_COLOR_ARRAY);
//     glColorPointer(4, GL_FLOAT, 0, pColorList);
// 
//     glDrawArrays(type, 0, num);
// 
//     glDisableClientState(GL_COLOR_ARRAY);
//     glDisableClientState(GL_VERTEX_ARRAY);
// }
// 
// 
// void RenderData(const PxRenderBuffer & data)
// {
//     glLineWidth(1.0f);
//     glDisable(GL_LIGHTING);
// 
//     //----------Render Points------------------
//     unsigned int NbPoints = data.getNbPoints();
//     if(NbPoints)
//     {
//         float* pVertList = new float[NbPoints*3];
//         float* pColorList = new float[NbPoints*4];
//         int vertIndex = 0;
//         int colorIndex = 0;
//         const PxDebugPoint* Points = data.getPoints();
//         while(NbPoints--)
//         {
//             pVertList[vertIndex++] = Points->pos.x;
//             pVertList[vertIndex++] = Points->pos.y;
//             pVertList[vertIndex++] = Points->pos.z;
//             pColorList[colorIndex++] = (float)((Points->color>>16)&0xff)/255.0f;
//             pColorList[colorIndex++] = (float)((Points->color>>8)&0xff)/255.0f;
//             pColorList[colorIndex++] = (float)(Points->color&0xff)/255.0f;
//             pColorList[colorIndex++] = 1.0f;
//             Points++;
//         }
// 
//         RenderBuffer(pVertList, pColorList, GL_POINTS, data.getNbPoints());
// 
//         delete[] pVertList;
//         delete[] pColorList;
//     }
// 
// 
//     //----------Render Lines------------------
//     unsigned int NbLines = data.getNbLines();
//     if(NbLines)
//     {
//         float* pVertList = new float[NbLines*3*2];
//         float* pColorList = new float[NbLines*4*2];
//         int vertIndex = 0;
//         int colorIndex = 0;
//         const PxDebugLine* Lines = data.getLines();
//         while(NbLines--)
//         {
//             pVertList[vertIndex++] = Lines->pos0.x;
//             pVertList[vertIndex++] = Lines->pos0.y;
//             pVertList[vertIndex++] = Lines->pos0.z;
//             pColorList[colorIndex++] = (float)((Lines->color0>>16)&0xff)/255.0f;
//             pColorList[colorIndex++] = (float)((Lines->color0>>8)&0xff)/255.0f;
//             pColorList[colorIndex++] = (float)(Lines->color0&0xff)/255.0f;
//             pColorList[colorIndex++] = 1.0f;
// 
//             pVertList[vertIndex++] = Lines->pos1.x;
//             pVertList[vertIndex++] = Lines->pos1.y;
//             pVertList[vertIndex++] = Lines->pos1.z;
//             pColorList[colorIndex++] = (float)((Lines->color0>>16)&0xff)/255.0f;
//             pColorList[colorIndex++] = (float)((Lines->color0>>8)&0xff)/255.0f;
//             pColorList[colorIndex++] = (float)(Lines->color0&0xff)/255.0f;
//             pColorList[colorIndex++] = 1.0f;
// 
//             Lines++;
//         }
// 
//         RenderBuffer(pVertList, pColorList, GL_LINES, data.getNbLines()*2);
// 
//         delete[] pVertList;
//         delete[] pColorList;
//     }
// 
// 
//     //----------Render Triangles------------------
//     unsigned int NbTris = data.getNbTriangles();
//     if(NbTris)
//     {
//         float* pVertList = new float[NbTris*3*3];
//         float* pColorList = new float[NbTris*4*3];
//         int vertIndex = 0;
//         int colorIndex = 0;
//         const PxDebugTriangle* Triangles = data.getTriangles();
//         while(NbTris--)
//         {
//             pVertList[vertIndex++] = Triangles->pos0.x;
//             pVertList[vertIndex++] = Triangles->pos0.y;
//             pVertList[vertIndex++] = Triangles->pos0.z;
// 
//             pVertList[vertIndex++] = Triangles->pos1.x;
//             pVertList[vertIndex++] = Triangles->pos1.y;
//             pVertList[vertIndex++] = Triangles->pos1.z;
// 
//             pVertList[vertIndex++] = Triangles->pos2.x;
//             pVertList[vertIndex++] = Triangles->pos2.y;
//             pVertList[vertIndex++] = Triangles->pos2.z;
// 
//             for(int i=0;i<3;i++)
//             {
//                 pColorList[colorIndex++] = (float)((Triangles->color0>>16)&0xff)/255.0f;
//                 pColorList[colorIndex++] = (float)((Triangles->color0>>8)&0xff)/255.0f;
//                 pColorList[colorIndex++] = (float)(Triangles->color0&0xff)/255.0f;
//                 pColorList[colorIndex++] = 1.0f;
//             }
// 
//             Triangles++;
//         }
// 
//         RenderBuffer(pVertList, pColorList, GL_TRIANGLES, data.getNbTriangles()*3);
// 
//         delete[] pVertList;
//         delete[] pColorList;
//     }
//     glEnable(GL_LIGHTING);
//     glColor4f(1.0f,1.0f,1.0f,1.0f);
// }