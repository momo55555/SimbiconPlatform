// Copyright (C) 2007 Dave Griffiths
// Fluxus Shader Library
// ---------------------
// Facing Ratio Shader
// Blends from inner to outer colour depending
// on the facing ratio of the fragment. Useful for
// getting the scanning electron microscope look.

//uniform vec4 OuterColour;
//uniform vec4 InnerColour;

vec4 InnerColour = vec4(0.8, 0.8, 1.0, 1.0);
vec4 OuterColour = vec4(0.0, 0.0, 0.2, 1.0);


varying vec3 N;
varying vec3 V;

void main()
{ 
    float ratio = dot(normalize(V),normalize(N));
    clamp(ratio,0.0,1.0);
    gl_FragColor = mix(OuterColour,InnerColour,ratio);
}
