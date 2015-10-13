// Copyright (C) 2007 Dave Griffiths
// Fluxus Shader Library
// ---------------------
// Blinn/Phong Shader
// This is the standard per-fragment lighting
// shading model

//normal in world coordinates
varying vec3 N;
//local coordinates of the current vertex
varying vec3 P;
//location of current vertex in world coordinates
varying vec3 V;

varying vec3 L;

void main(){    
    vec3 LightPos = vec3(gl_LightSource[0].position);
//	vec3 LightPos = vec3(0,0,100);
    N = normalize(gl_NormalMatrix*gl_Normal);
    P = gl_Vertex.xyz;
    V = -vec3(gl_ModelViewMatrix*gl_Vertex);
//	L = vec3(gl_ModelViewMatrix*(vec4(LightPos,1)+vec4(V,1)));
	L = vec3(gl_ModelViewMatrix*(vec4(LightPos,1)+vec4(P,1)));
    gl_Position = ftransform();
}
