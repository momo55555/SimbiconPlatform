


varying vec3  ecPosition;
varying vec3  tnorm;
varying vec4  vColor;


void main(){
    ecPosition = vec3(gl_ModelViewMatrix * gl_Vertex);
    tnorm      = normalize(gl_NormalMatrix * gl_Normal);
    vColor = gl_FrontMaterial.diffuse;
    gl_Position     = ftransform();
}