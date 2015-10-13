// Copyright (C) 2007 Dave Griffiths
// Fluxus Shader Library
// ---------------------
// Glossy Specular Reflection Shader
// A more controllable version of blinn shading,
// Useful for ceramic or fluids - from Advanced 
// Renderman, thanks to Larry Gritz

//uniform vec3 AmbientColour;
//uniform vec3 DiffuseColour;
//uniform vec3 SpecularColour;
//uniform float AmbientIntensity;
//uniform float DiffuseIntensity;
//uniform float SpecularIntensity;
//uniform float Roughness;
//uniform float Sharpness;

vec3 AmbientColour = vec3(0.75, 0.75, 0.75);
vec3 DiffuseColour = vec3(0.6, 0.6, 0.0);
vec3 SpecularColour = vec3(0.0, 0.0, 0.6);

float AmbientIntensity = 0.1;
float DiffuseIntensity = 0.8;
float SpecularIntensity = 1.1;
float Roughness = 0.3;
float Sharpness = 0.0;

varying vec3 N;
varying vec3 P;
varying vec3 V;
varying vec3 L;
    
void main()
{ 
	float w = 0.18*(1.0-Sharpness);
	
    vec3 l = normalize(L);
    vec3 n = normalize(N);
    vec3 v = normalize(V);
    vec3 h = normalize(l+v);

    float diffuse = dot(l,n);
    float specular = smoothstep(0.72-w,0.72+w,pow(max(0.0,dot(n,h)),1/Roughness));
    
    gl_FragColor = vec4(AmbientColour*AmbientIntensity + 
                        DiffuseColour*diffuse*DiffuseIntensity +
                        SpecularColour*specular*SpecularIntensity,1);
}
