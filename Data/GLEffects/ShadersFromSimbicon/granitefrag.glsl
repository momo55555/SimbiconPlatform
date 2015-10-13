// Very loosely based on the granite shader in the Orange Book.

varying vec3 normal;
varying vec4 pos;
varying vec4 rawpos;

uniform float scale;

float noise(vec4);
float snoise(vec4);
float noise(vec3);
float snoise(vec3);
float unsign(float x);

vec4 noise4d(vec4);

void main() {
	vec4 color;
	vec4 matspec = gl_FrontMaterial.specular;
	float shininess = gl_FrontMaterial.shininess;
	vec4 lightspec = gl_LightSource[0].specular;
	vec4 lpos = gl_LightSource[0].position;
	vec4 s = -normalize(pos-lpos); 	//not sure why this needs to 
									// be negated, but it does.
	vec3 light = s.xyz;
	vec3 n = normalize(normal);
	vec3 r = -reflect(light, n);
	r = normalize(r);
	vec3 v = -pos.xyz; // We are in eye coordinates,
					   // so the viewing vector is
					   // [0.0 0.0 0.0] - pos
	v = normalize(v);

	float scalelocal;
	if (scale == 0.0) {
		scalelocal = 1.0; //default value
	} else {
		scalelocal = scale;
	}

	vec4 tp = gl_TexCoord[0] * scalelocal;
	vec3 rp = rawpos.xyz * scalelocal * 20.0;
	
	//vec4 noisevec = vec4(noise(rp), noise(rp + 3.33), noise(rp + 7.77), noise(rp + 13.32));
	
//	float intensity = min(1.0, noise(rp) * 5.0);
	float intensity = min(1.0, (unsign(noise(rp))/16.0) * 18.0);
	color = vec4(intensity, intensity, intensity, 1.0);
	
	vec4 diffuse  = color * max(0.0, dot(n, s.xyz)) * gl_LightSource[0].diffuse;
	vec4 specular;
	if (shininess != 0.0) {
		specular = lightspec * matspec * pow(max(0.0, dot(r, v)), shininess);
	} else {
		specular = vec4(0.0, 0.0, 0.0, 0.0);
	}
	
	gl_FragColor = diffuse + specular;


}

