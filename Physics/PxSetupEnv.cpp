 #include "stdafx.h"
// 
// #include <iostream>
// #include <fstream>
// #include "PxSetupEnv.h"
// 
// using namespace std;
// using namespace Physics;
// 
// PhysicsContext::PhysicsContext()
// {
//     gTimeStep = 1.0f / 60.0f;
//     init();
// }
// 
// void PhysicsContext::init()
// {
//     //Creating foundation for PhysX
//     gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
// 
//     //Creating instance of PhysX SDK
//     gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale() );
// 
//     if(gPhysicsSDK == NULL)
//     {
//         cerr << "Error creating PhysX3 device, Exiting..." << endl;
//         exit(1);
//     }
// 
//     //Creating scene
//     PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());		//Descriptor class for scenes 
// 
//     sceneDesc.gravity		= PxVec3(0, -10.0, 0);			//Setting gravity
//     sceneDesc.cpuDispatcher = PxDefaultCpuDispatcherCreate(1);		//Creating default CPU dispatcher for the scene
//     sceneDesc.filterShader  = PxDefaultSimulationFilterShader;		//Creating default collision filter shader for the scene
// 
//     gScene = gPhysicsSDK->createScene(sceneDesc);					//Creating a scene 
// 
// //     // check if PvdConnection manager is available on this platform
// //     if(gPhysicsSDK->getPvdConnectionManager() == NULL)
// //         return;
// // 
// //     // setup connection parameters
// //     const char*     pvd_host_ip = "127.0.0.1";  // IP of the PC which is running PVD
// //     int             port        = 5425;         // TCP port to connect to, where PVD is listening
// //     unsigned int    timeout     = 100;          // timeout in milliseconds to wait for PVD to respond,
// //     // consoles and remote PCs need a higher timeout.
// //     PxVisualDebuggerConnectionFlags connectionFlags = PxVisualDebuggerConnectionFlag::eDEBUG;
// // 
// //     // and now try to connect
// //     theConnection = PxVisualDebuggerExt::createConnection(gPhysicsSDK->getPvdConnectionManager(), pvd_host_ip, port, timeout, connectionFlags);
// // 
// //     gPhysicsSDK->getVisualDebugger()->setVisualizeConstraints(true);
// //     gPhysicsSDK->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_CONTACTS, true);
// //     gPhysicsSDK->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_SCENEQUERIES, true);
// }
// 
// void PhysicsContext::shutdown()
// {
//     if(theConnection)
//         theConnection->release();
//     gPhysicsSDK->release();
//     gFoundation->release();
// }