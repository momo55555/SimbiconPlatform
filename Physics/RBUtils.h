#pragma once

#define RB_NOT_IMPORTANT				1
#define RB_RB							2
#define RB_END_RB						3
#define RB_ARB							4
#define RB_MESH_NAME					5
#define RB_MASS							6
#define RB_MOI							7
#define RB_COLOUR						8
#define RB_ROOT							9
#define RB_ARTICULATED_FIGURE			10
#define RB_CHILD						11
#define RB_PARENT						12
#define RB_END_ARTICULATED_FIGURE		13
#define RB_NAME							14
#define RB_JOINT						15
#define RB_END_JOINT					16
#define RB_PPOS							17
#define RB_CPOS							18
#define RB_SPHERE						19
#define RB_CAPSULE						20
#define RB_PLANE						21
#define RB_LOCKED						22
#define RB_POSITION						23
#define RB_ORIENTATION					24
#define RB_VELOCITY						25
#define RB_ANGULAR_VELOCITY				26
#define RB_FRICTION_COEFF				27
#define RB_RESTITUTION_COEFF			28
#define RB_MIN_BDG_SPHERE				29
#define RB_JOINT_TYPE_HINGE				30
#define RB_JOINT_LIMITS					31
#define RB_JOINT_TYPE_UNIVERSAL			32
#define RB_JOINT_TYPE_BALL_IN_SOCKET	33
#define RB_BOX							34
#define RB_ODE_GROUND_COEFFS			35
#define RB_PLANAR						36
#define RB_SOFT_BODY					37

/**
	This method is used to determine the type of a line that was used in the input file for a rigid body.
	It is assumed that there are no white spaces at the beginning of the string that is passed in. the pointer buffer
	will be updated to point at the first character after the keyword.
*/
int getRBLineType(char* &buffer);