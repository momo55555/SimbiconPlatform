
varying vec3 Normal;

void main(void){
	//compute the normal, in eye coordinates
	Normal = normalize(gl_NormalMatrix * gl_Normal);
	//and the position of the vertex
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
