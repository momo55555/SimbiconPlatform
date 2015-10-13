// Copyright (C) 2007 Dave Griffiths
// Fluxus Shader Library
// ---------------------
// Glossy Specular Reflection Shader
// A more controllable version of blinn shading,
// Useful for ceramic or fluids - from Advanced 
// Renderman, thanks to Larry Gritz


varying vec3 N;
varying vec3 P;
varying vec3 V;
varying vec3 L;

void main()
{   
	vec3 LightPos = vec3(gl_LightSource[0].position);
    N = normalize(gl_NormalMatrix*gl_Normal);
    P = gl_Vertex.xyz;
    V = -vec3(gl_ModelViewMatrix*gl_Vertex);
	L = vec3(gl_ModelViewMatrix*(vec4(LightPos,1)-gl_Vertex));
    gl_Position = ftransform();
}
