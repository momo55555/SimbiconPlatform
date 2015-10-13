// Copyright (C) 2007 Dave Griffiths
// Fluxus Shader Library
// ---------------------
// Blinn/Phong Shader
// This is the standard per-fragment lighting
// shading model

vec3 AmbientColour = vec3(0.75, 0.75, 0.75);
vec3 DiffuseColour = vec3(0.6, 0.6, 0.0);
vec3 SpecularColour = vec3(0.0, 0.0, 0.6);

float AmbientIntensity = 0.1;
float DiffuseIntensity = 0.8;
float SpecularIntensity = 0.1;
float Roughness = 0.3;

varying vec3 N;
varying vec3 P;
varying vec3 V;
varying vec3 L;

void main()
{ 
    vec3 l = normalize(L);
    vec3 n = normalize(N);
    vec3 v = normalize(V);
    vec3 h = normalize(l+v);

    float diffuse = dot(l,n);
    float specular = pow(max(0.0,dot(n,h)),1.0/Roughness);
    
    gl_FragColor = vec4(AmbientColour*AmbientIntensity + 
                        DiffuseColour*diffuse*DiffuseIntensity +
                        SpecularColour*specular*SpecularIntensity,1);
}
