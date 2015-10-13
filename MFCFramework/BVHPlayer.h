/*

Copyright 2014 Rudy Snow

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "stdafx.h"
#include "Windows.h"

#include "GLCallbacks.h"
#include "SkinningTechnique.h"
#include "FloorTechnique.h"
#include "LightingTechnique.h"
#include "WireFrameTechnique.h"
#include "Skybox.h"
#include "MeshLoader_Skel.h"
#include "MeshLoader.h"
#include "GLUtil.h"
#include "Camera.h"
#include "Pipeline.h"
#include "EngineCommon.h"
#include "PxPlane.h"
//#include "PhysXVisualization.h"
#include "PxRigidBody.h"
#include "PxArticulatedFigure.h"

#ifndef WIN32
#include <sys/time.h>
#include <unistd.h>
#endif
#include <sys/types.h>
#include <conio.h>

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <iostream>
#include <fstream>
#include <cmath>

#undef max
#undef min

#include "Objectives1.h"
#include <shark/Algorithms/DirectSearch/MOCMA.h>

using namespace shark;
using namespace std;

#define RENDERING
//#define PHYSX_DEBUGGING
//#define LONG_HAND
//#define LONG_LEG

// Inherent from CGLCallbacks to override those virtual interface
class CBVHPlayer : public CGLCallbacks
{
public:

    CBVHPlayer()
    {
         m_SkinningTechnique = NULL;
         m_FloorTechnique = NULL;
         m_LightingTechnique = NULL;
         m_Skeleton = NULL;
         m_Floor = NULL;
         m_BoxMesh = NULL;
         m_Box12Mesh = NULL;
         m_Box13Mesh = NULL;
         m_Box14Mesh = NULL;
         m_Box32Mesh = NULL;
         Pelvis = NULL;
         Torso = NULL;
         Head = NULL;
         lUpperarm = NULL;
         lLowerarm = NULL;
         rUpperarm = NULL;
         rLowerarm = NULL;
         lUpperleg = NULL;
         lLowerleg = NULL;
         rUpperleg = NULL;
         rLowerleg = NULL;
         lFoot = NULL;
         rFoot = NULL;
         m_FloorTexture = NULL;
//          m_Plane = NULL;
//          m_PhysicsSDK = NULL;
//          gScene = NULL;
//          m_Box = NULL;
         m_GameCamera = NULL;        
         m_SkyBox = NULL;
/*         m_AF = NULL;*/
 
         m_DirectionLight.AmbientIntensity = 1.0f;
         //m_DirectionLight.DiffuseIntensity = 0.8f;
         m_DirectionLight.Color = COLOR_RED;
         //m_DirectionLight.Direction = Vector3f(1.0f, -1.0f, 0.0f);
        stepNum = 0;
    }

    ~CBVHPlayer()
    {
         SAFE_DELETE(m_SkinningTechnique);
         SAFE_DELETE(m_FloorTechnique);
         SAFE_DELETE(m_LightingTechnique);
         SAFE_DELETE(m_Floor);
         SAFE_DELETE(m_BoxMesh);
         SAFE_DELETE(m_Box12Mesh);
         SAFE_DELETE(m_Box13Mesh);
         SAFE_DELETE(m_Box14Mesh);
         SAFE_DELETE(m_Box32Mesh);
         SAFE_DELETE(m_FloorTexture);
         SAFE_DELETE(Pelvis);
         SAFE_DELETE(Torso);
         SAFE_DELETE(Head);
         SAFE_DELETE(lUpperarm);
         SAFE_DELETE(lLowerarm);
         SAFE_DELETE(rUpperarm);
         SAFE_DELETE(rLowerarm);
         SAFE_DELETE(lUpperleg);
         SAFE_DELETE(lLowerleg);
         SAFE_DELETE(rUpperleg);
         SAFE_DELETE(rLowerleg);
         SAFE_DELETE(lFoot);
         SAFE_DELETE(rFoot);
/*         SAFE_DELETE(m_PhysicsSDK);*/
/*         SAFE_DELETE(m_Box);*/
         SAFE_DELETE(m_SkyBox);
         SAFE_DELETE(m_Skeleton);    
         SAFE_DELETE(m_GameCamera);
         //SAFE_DELETE(m_AF);
         SAFE_DELETE(Globals::app);
    }

    virtual void Render()
    {
//         if(gScene)
//         {
            StepPhysX();
//         }

        // Clear the color buffer
#ifdef RENDERING
#ifndef PHYSX_DEBUGGING       
        RigidBody* Bone1 = (Globals::app)->conF->getCharacter()->getARBByName("pelvis"); 
        Point3d Origin1(0, 0, 0);
        Point3d BoneWorldFrame1(0, 0, 0);
        BoneWorldFrame1 = Bone1->getWorldCoordinates(Origin1);
        //Bone1 = NULL;
        //m_GameCamera->m_pos = Vector3f(BoneWorldFrame1.getX()+3.0f, BoneWorldFrame1.getY(), BoneWorldFrame1.getZ()+3.0f);
        m_GameCamera->m_target = Vector3f(-3.0f, 0, -3.0f);
        m_GameCamera->m_up = Vector3f(0.0f, 1.0f, 0);
        m_GameCamera->OnRender();
#endif

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Pipeline p;



        p.SetCamera(m_GameCamera->GetPos(), m_GameCamera->GetTarget(), m_GameCamera->GetUp());
        p.SetPerspectiveProj(m_persProjInfo);
        glDisable(GL_CULL_FACE);
        m_FloorTechnique->Enable();
        m_FloorTexture->Bind(GL_TEXTURE0); 

        for(int i = 0; i < 40; ++i)
        {
            for(int j = 0; j < 40; ++j)
            {
                p.Rotate(-90.0f, 0.0f, 0.0f);
                p.Scale(5.0f, 5.0f, 5.0f);
                p.WorldPos(-200.0f + i * 10.0f, -0.0f, -200.0f + j * 10.0f);
#ifdef LONG_LEG
                p.WorldPos(-200.0f + i * 10.0f, -0.50f, -200.0f + j * 10.0f);
#endif

                m_FloorTechnique->SetWVP(p.GetWVPTrans());
                m_FloorTechnique->SetWorldMatrix(p.GetWorldTrans());

                m_Floor->Render();
            }
        }

        glEnable(GL_CULL_FACE);
        RigidBody* Bone = NULL;
        Point3d BoneWorldFrame(0, 0, 0);
        Point3d Origin(0, 0, 0);



        int i  = 0;
        for(i = 0; i < 13; ++i)
        {
//             PxTransform BoxTrans = m_AF->mBodyNodes[i]->getBody()->getRigidBodyEntity()->getGlobalPose();
//             Vector3f scaleFactor = m_AF->mBodyNodes[i]->getGeometry();
            switch(i)
            {
                case 0: 
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("pelvis"); 
                        break;
                    }
                case 1: 
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("torso"); 
                        break;
                    }
                case 2: 
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("head"); 
                        break;
                    }
                case 3: 
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("lUpperarm"); 
                        break;
                    }
                case 4:
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("lLowerarm"); 
                        break;
                    }
                case 5:
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("rUpperarm"); 
                        break;
                    }
                case 6:
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("rLowerarm"); 
                        break;
                    }
                case 7:
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("lUpperleg"); 
                        break;
                    }
                case 8: 
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("lLowerleg");
                        break;
                    }
                case 9:
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("rUpperleg"); 
                        break;
                    }
                case 10:
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("rLowerleg"); 
                        break;
                    }
                case 11: 
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("lFoot");
                        break;
                    }
                case 12:
                    {
                        Bone = (Globals::app)->conF->getCharacter()->getARBByName("rFoot"); 
                        break;
                    }
            }
            BoneWorldFrame = Bone->getWorldCoordinates(Origin);
            p.WorldPos(BoneWorldFrame.getX(), BoneWorldFrame.getY(), BoneWorldFrame.getZ());
//             PxReal angle1, angle2, angle3;
//             PxVec3 axis1(1.0, 0.0, 0.0), axis2(0.0, 1.0, 0.0), axis3(0.0, 0.0, 1.0);
//             Quaternion q(BoxTrans.q.x, BoxTrans.q.y, BoxTrans.q.z, -BoxTrans.q.w);
//             fout << Bone->getParentJoint()->getParentJointPosition().getX() << "\t" << Bone->getParentJoint()->getParentJointPosition().getY()
//                 << "\t" << Bone->getParentJoint()->getParentJointPosition().getZ() << "\t";

            Quaternion q = Bone->getOrientation().getComplexConjugate();
            //Bone = NULL;
           
//             Quaternion q1;
//             Bone->getParentJoint()->computeRelativeOrientation(q1);
//             fout << q1.v.getX() << "\t" << q1.v.getY() << "\t" << q1.v.getZ() << "\t" << q1.s << endl;

            p.Rotate(q);
            p.Scale(1.0, 1.0, 1.0);

#ifdef LONG_HAND
            if(i == 4 || i == 6)
            {
                p.WorldPos(BoneWorldFrame.getX(), BoneWorldFrame.getY(), BoneWorldFrame.getZ());
                p.Scale(1.3333, 3.0, 3.0);
            }
            if(i == 3 || i == 5)
            {
                p.WorldPos(BoneWorldFrame.getX(), BoneWorldFrame.getY(), BoneWorldFrame.getZ());
                p.Scale(1.0, 3.0, 3.0);
            }
#endif

#ifdef LONG_LEG
            if(i == 8 || i == 10)
            {
                p.WorldPos(BoneWorldFrame.getX(), BoneWorldFrame.getY(), BoneWorldFrame.getZ());
                p.Scale(1.0, 2.0, 1.0);
            }
#endif

            m_LightingTechnique->Enable();
            m_LightingTechnique->SetWVP(p.GetWVPTrans());
            m_LightingTechnique->SetWorldMatrix(p.GetWorldTrans());
            
            switch(i)
            {
                case 0: Pelvis->Render(); break;
                case 1: Torso->Render(); break;
                case 2: Head->Render(); break;
                case 3: lUpperarm->Render(); break;
                case 4: lLowerarm->Render(); break;
                case 5: rUpperarm->Render(); break;
                case 6: rLowerarm->Render(); break;
                case 7: lUpperleg->Render(); break;
                case 8: lLowerleg->Render(); break;
                case 9: rUpperleg->Render(); break;
                case 10: rLowerleg->Render(); break;
                case 11: lFoot->Render(); break;
                case 12: rFoot->Render(); break;
            }

            glPolygonOffset(-1.0f, -1.0f);
            glEnable(GL_POLYGON_OFFSET_LINE);
            glEnable(GL_LINE_SMOOTH);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glLineWidth(0.5f);

            Vector3f vBlack = COLOR_BLACK;
            m_WireframeTechnique->Enable();
            m_WireframeTechnique->SetColor(vBlack);
            m_WireframeTechnique->SetWVP(p.GetWVPTrans());
            m_WireframeTechnique->SetWorldMatrix(p.GetWorldTrans());

            switch(i)
            {
                case 0: Pelvis->Render(); break;
                case 1: Torso->Render(); break;
                case 2: Head->Render(); break;
                case 3: lUpperarm->Render(); break;
                case 4: lLowerarm->Render(); break;
                case 5: rUpperarm->Render(); break;
                case 6: rLowerarm->Render(); break;
                case 7: lUpperleg->Render(); break;
                case 8: lLowerleg->Render(); break;
                case 9: rUpperleg->Render(); break;
                case 10: rLowerleg->Render(); break;
                case 11: lFoot->Render(); break;
                case 12: rFoot->Render(); break;
            }

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_POLYGON_OFFSET_LINE);
            glLineWidth(1.0f);
            glDisable(GL_BLEND);
        }

        p.Scale(0.1f, 0.1f, 0.1f);
        p.WorldPos(0.0f, 0.0f, 0.0f);
        p.Rotate(0.0f, 0.0f, 0.0f);

        vector<Matrix4f> Transforms;
        float RunningTime = (float)((double)GetCurrentTimeMillis() - (double)m_startTime) / 1000.0f;

        m_SkinningTechnique->Enable();

//         m_Skeleton->BoneTransform(RunningTime, Transforms);
//         for (uint i = 0 ; i < Transforms.size() ; i++) 
//         {
//             m_SkinningTechnique->SetBoneTransform(i, Transforms[i]);
//         }
// 
//         m_SkinningTechnique->SetEyeWorldPos(m_GameCamera->GetPos());
//         m_SkinningTechnique->SetWVP(p.GetWVPTrans());
//         m_SkinningTechnique->SetWorldMatrix(p.GetWorldTrans());
// 
//         m_Skeleton->Render();

        m_SkyBox->Render();

        m_frameCount++;
//         glLoadIdentity();
// 
//         glTranslatef(0, 0, gCamDistance);
//         glRotatef(gCamRoateX, 1, 0, 0);
//         glRotatef(gCamRoateY, 0, 1, 0);
// 
//         for(int i = 0; i < MAX_BODY; ++i)
//         {
//             PxTransform BoxTrans = m_AF->mBodyNodes[i]->getBody()->getRigidBodyEntity()->getGlobalPose();
//             Vector3f scaleFactor = m_AF->mBodyNodes[i]->getGeometry();
//         }
// 
//         RenderData(gScene->getRenderBuffer());

        //gConnectedBox->setAngularVelocity(PxVec3(0,1,0)); //Applying angular velocity to the actor
        //gBox->addForce(PxVec3(0,0,-180));	

        // CODE OMITTED: here are the code to swap buffers
        //glutSwapBuffers();
#endif
}

    virtual bool Initialize()
    {
        m_frameCount = 0;
        // set the background color when clear
#ifndef PHYSX_DEBUGGING
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_MULTISAMPLE);
        glFrontFace(GL_CCW);
        glCullFace(GL_BACK);
        glEnable(GL_CULL_FACE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        m_DirectionLight.Color = COLOR_RED;
        m_SkinningTechnique = new SkinningTechnique();
        if (!m_SkinningTechnique->Init()) 
        {
            printf("Error initializing the Skinning technique\n");
            return false;
        }
        m_SkinningTechnique->Enable();
        m_SkinningTechnique->SetColorTextureUnit(COLOR_TEXTURE_UNIT_INDEX);
        m_SkinningTechnique->SetDirectionalLight(m_DirectionLight);
        m_SkinningTechnique->SetMatSpecularPower(0);

        m_FloorTechnique = new FloorTechnique();
        if (!m_FloorTechnique->Init()) 
        {
            printf("Error initializing the Floor technique\n");
            return false;
        }
        m_DirectionLight.Color = COLOR_WHITE;
        m_FloorTechnique->Enable();
        m_FloorTechnique->SetDirectionalLight(m_DirectionLight);
        m_FloorTechnique->SetTextureUnit(0);

        m_LightingTechnique = new LightingTechnique();
        if (!m_LightingTechnique->Init()) 
        {
            printf("Error initializing the Lighting Technique\n");
            return false;
        }
        m_DirectionLight.Color = COLOR_PURPLE;
        m_LightingTechnique->Enable();
        m_LightingTechnique->SetDirectionalLight(m_DirectionLight);
        m_LightingTechnique->SetTextureUnit(0);

        m_WireframeTechnique = new WireFrameTechnique();
        m_WireframeTechnique->Init();

//         m_Skeleton = new MeshSkel();
//         if (!m_Skeleton->LoadMesh("../Data/BVH/Boxing_Toes.bvh"))
//         {
//             return false;
//         }

        m_Floor = new Mesh();
        if(!m_Floor->LoadMesh("../Data/OBJ/quad.obj"))
        {
            return false;
        }

        AllocConsole();

//         m_BoxMesh = new Mesh();
//         if(!m_BoxMesh->LoadMesh("../Data/OBJ/Cube.obj"))
//         {
//             return false;
//         }
// 
//         m_Box12Mesh = new Mesh();
//         if(!m_Box12Mesh->LoadMesh("../Data/OBJ/Cube12.obj"))
//         {
//             return false;
//         }
// 
//         m_Box13Mesh = new Mesh();
//         if(!m_Box13Mesh->LoadMesh("../Data/OBJ/Cube13.obj"))
//         {
//             return false;
//         }
// 
//         m_Box14Mesh = new Mesh();
//         if(!m_Box14Mesh->LoadMesh("../Data/OBJ/Cube14.obj"))
//         {
//             return false;
//         }

        Pelvis = new Mesh();
        if(!Pelvis->LoadMesh("../Data/OBJ/pelvis_2_s.obj"))
        {
            return false;
        }

        Torso = new Mesh();
        if(!Torso->LoadMesh("../Data/OBJ/torso_2_s_v2.obj"))
        {
            return false;
        }

        Head = new Mesh();
        if(!Head->LoadMesh("../Data/OBJ/head_s.obj"))
        {
            return false;
        }

        lUpperarm = new Mesh();
        if(!lUpperarm->LoadMesh("../Data/OBJ/lupperarm.obj"))
        {
            return false;
        }

        lLowerarm = new Mesh();
        if(!lLowerarm->LoadMesh("../Data/OBJ/llowerarm.obj"))
        {
            return false;
        }

        rUpperarm = new Mesh();
        if(!rUpperarm->LoadMesh("../Data/OBJ/rupperarm.obj"))
        {
            return false;
        }
        
        rLowerarm = new Mesh();
        if(!rLowerarm->LoadMesh("../Data/OBJ/rlowerarm.obj"))
        {
            return false;
        }

        lUpperleg = new Mesh();
        if(!lUpperleg->LoadMesh("../Data/OBJ/lupperleg.obj"))
        {
            return false;
        }

        lLowerleg = new Mesh();
        if(!lLowerleg->LoadMesh("../Data/OBJ/llowerleg.obj"))
        {
            return false;
        }

        rUpperleg = new Mesh();
        if(!rUpperleg->LoadMesh("../Data/OBJ/rupperleg.obj"))
        {
            return false;
        }

        rLowerleg = new Mesh();
        if(!rLowerleg->LoadMesh("../Data/OBJ/rlowerleg.obj"))
        {
            return false;
        }

        lFoot= new Mesh();
        if(!lFoot->LoadMesh("../Data/OBJ/lfoot.obj"))
        {
            return false;
        }

        rFoot= new Mesh();
        if(!rFoot->LoadMesh("../Data/OBJ/rfoot.obj"))
        {
            return false;
        }

        m_FloorTexture = new Texture(GL_TEXTURE_2D, "../Data/Texture/Floor.bmp");
        if(!m_FloorTexture->Load(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR_MIPMAP_LINEAR, GL_CLAMP_TO_EDGE))
        {
            return false;
        }

        m_startTime = GetCurrentTimeMillis();

        Globals::app = new ControllerEditor();

        (Globals::app)->init();

//         std::vector<RealVector> data;
//         ifstream dataFile;
//         dataFile.open("../Data/Optimization/fwalk.in");
//         double ppp;
//         for(std::size_t i = 0; i != 23; ++i){
//             dataFile >> ppp;
//             RealVector tmppp;
//             tmppp.push_back(ppp);
//             data.push_back(tmppp);
//         }
//         dataFile.close();
        
        //RealVector point(23);
#ifndef RENDERING
        mocma.init(obj1);
#endif

#endif

//         gOldMouseX = 0;
//         gOldMouseY = 0;
// 
//         gCamRoateX = 15; 
//         gCamRoateY = 0;
//         gCamDistance = -50;
// 
//         m_PhysicsSDK = new PhysicsContext();
//         gScene = m_PhysicsSDK->getScene();
// 
//         m_Plane = new CPlane(m_PhysicsSDK);
//         m_Plane->createPlane(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(-90.0f, 0.0f, 0.0f), 0.1, 0.1, 0.0f);
// 
//         m_AF = new CArticulatedFigure(m_PhysicsSDK);
//         m_AF->createFigure();
//         m_AF->addAggregateToScene();

#ifdef PHYSX_DEBUGGING
        //This will enable basic visualization of PhysX objects like- actors collision shapes and it's axis. 
        //The function PxScene::getRenderBuffer() is used to render any active visualization for scene.
        gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0);	//Global visualization scale which gets multiplied with the individual scales
        gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);	//Enable visualization of actor's shape
        gScene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES,	 1.0f);	//Enable visualization of actor's axis

        gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);
        gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
#endif

        return true;
    }

    virtual bool Update(GLuint current, GLuint delta)
    {
        return true;
    }

    void StepPhysX()					//Stepping PhysX
    { 


#ifdef RENDERING
        (Globals::app)->processTask();
#else 
        mocma.step(obj1);
        stepNum++;
        if(obj1.evaluationCounter() == 5010)
        {
            exit(0);
        }
//         if(stepNum == 18)
//         {
//             int a = 5;
//         }
//         if(stepNum > 1000)
//         {
//             ofstream resFile;
//             resFile.open("../Data/Result/BigWalk.txt");
//             for(size_t i = 0; i < mocma.solution().size(); ++i)
//             {
//                 for(size_t j = 0; j < 43; ++j)
//                 {
//                     resFile << mocma.solution()[i].point[j] << " ";		
//                 }
//                 resFile << endl;
//             }
//             resFile.close();
//             system("pause");
//         }
        _cprintf("step num: %d\n", stepNum);
#endif;

//         for( std::size_t i = 0; i < mocma.solution().size(); i++ ) {
//             for( std::size_t j = 0; j < obj1.numberOfObjectives(); j++ ) {
//                 _cprintf("%lf ", mocma.solution()[ i ].value[j]);
//             }
//             _cprintf("\n");
//         }
        
//         gScene->simulate(m_PhysicsSDK->gTimeStep);	//Advances the simulation by 'gTimeStep' time
//         gScene->fetchResults(true);		//Block until the simulation run is completed
    }

    void ShutdownPhysics()
    {
/*        m_PhysicsSDK->shutdown();*/
    }

    Camera* m_GameCamera;
    PersProjInfo m_persProjInfo;
    SkyBox* m_SkyBox;

    int gOldMouseX;
    int gOldMouseY;

    float gCamRoateX; 
    float gCamRoateY;
    float gCamDistance;

private:
    SkinningTechnique* m_SkinningTechnique;
    FloorTechnique* m_FloorTechnique;
    LightingTechnique* m_LightingTechnique;
    WireFrameTechnique* m_WireframeTechnique;
    MeshSkel* m_Skeleton;
    Mesh* m_Floor;
    Mesh* m_BoxMesh;
    Mesh* m_Box12Mesh;
    Mesh* m_Box13Mesh;
    Mesh* m_Box14Mesh;
    Mesh* m_Box32Mesh;
    Mesh* Pelvis;
    Mesh* Torso;
    Mesh* Head;
    Mesh* lUpperarm;
    Mesh* lLowerarm;
    Mesh* rUpperarm;
    Mesh* rLowerarm;
    Mesh* lUpperleg;
    Mesh* lLowerleg;
    Mesh* rUpperleg;
    Mesh* rLowerleg;
    Mesh* lFoot;
    Mesh* rFoot;
    Texture* m_FloorTexture;
    DirectionalLight m_DirectionLight;
    //PxScene* gScene;

    long long m_startTime;
    int m_frameCount;
    float m_fps;
    Vector3f m_position;
    float m_scale;
    
    MOCMA mocma;
    ObjectiveFunctions obj1;
    int stepNum;
};