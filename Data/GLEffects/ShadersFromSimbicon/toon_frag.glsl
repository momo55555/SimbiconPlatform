vec3 DiffuseColor = vec3(0.2, 0.4, 0.6);
vec3 PhongColor = vec3(0.0, 0.9, 0.9);
float Edge = 0.2;
float Phong = 0.8;
varying vec3 Normal;

void main (void){
	vec3 color = DiffuseColor;
	float f = dot(vec3(0,0,1),Normal);
	//see if the current fragment should get a bit of specular highlight, or if
	//it is part of an edge
	if (abs(f) < Edge)
		color = vec3(0);
	if (f > Phong)
		color = PhongColor;

	gl_FragColor = vec4(color, 1);
}
