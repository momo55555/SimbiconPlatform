//
// Vertex shader for hemispherical lighting
//
// Author: Randi Rost
//
// Copyright (C) 2005 3Dlabs, Inc.
//
// See 3Dlabs-License.txt for license information
//




varying vec3 vNorm;
varying vec3 ecPosition;


void main(void)
{
    ecPosition = vec3(gl_ModelViewMatrix * gl_Vertex);
    vNorm      = normalize(gl_NormalMatrix * gl_Normal);
    gl_Position     = ftransform();
}