#define NNOISE 4

#define PI 3.141592653

#define PALE_BLUE vec4(0.25, 0.25, 0.35, 1.0)
//#define PALE_BLUE vec4(0.90, 0.90, 1.0, 1.0)
#define MEDIUM_BLUE vec4(0.10, 0.10, 0.30, 1.0)
#define DARK_BLUE vec4(0.05, 0.05, 0.26, 1.0)
#define DARKER_BLUE vec4(0.03, 0.03, 0.20, 1.0)

#define SKYCOLOR vec4(0.15, 0.15, 0.6, 1.0)
#define CLOUDCOLOR vec4(1.0, 1.0, 1.0, 1.0)

varying vec3 normal;
varying vec4 pos;
varying vec4 rawpos;
uniform float time;

float noise(vec4);
float snoise(vec4);
float noise(vec3);
float snoise(vec3);
vec4 marble_color(float);
vec4 cloud_color(float);
vec4 mossyrock_color(float);
vec4 spline(float x, int y, vec4 z[]);

void main() {
	//vec4 color = gl_FrontMaterial.diffuse;
	//vec4 matspec = gl_FrontMaterial.specular;
	//float shininess = gl_FrontMaterial.shininess;
	//vec4 lightspec = gl_LightSource[0].specular;
	//vec4 lpos = gl_LightSource[0].position;
	//vec4 s = -normalize(pos-lpos); 	//not sure why this needs to 
									// be negated, but it does.
	//vec3 light = s.xyz;
	//vec3 n = normalize(normal);
	//vec3 r = -reflect(light, n);
	//r = normalize(r);
	//vec3 v = -pos.xyz; // We are in eye coordinates,
					   // so the viewing vector is
					   // [0.0 0.0 0.0] - pos
	//v = normalize(v);
	
	float scale=0.009;
	//vec4 tp = gl_TexCoord[0] * scale;
	//vec3 rp = rawpos.xyz * scale;
	vec3 rp = vec3(gl_FragCoord.xy, 0.0) * scale;
	
	// create the grayscale marbling here
	float marble=0.0;
	float f = 1.0;
//	for(int i=0; i < NNOISE; i++) {
//		marble += noise(rp*f)/f;
//		f *= 2.17;
//	}

	float marble1 = noise(rp*f)/f;
	f *= 2.17;
	float marble2 = marble1 + noise(rp*f)/f;
	f *= 2.17;
	float marble3 = marble2 + noise(rp*f)/f;
	f *= 2.17;
	float marble4 = marble3 + noise(rp*f)/f;
	f *= 2.17;
	float marble5 = marble4 + noise(rp*f)/f;
		
	float r1 = (marble1 / 3.542113654178) + 0.5;
	float r2 = ((marble2-marble1) / 3.542113654178) + 0.5;
	float r3 = ((marble3-marble2) / 3.542113654178) + 0.5;
	float r4 = ((marble4-marble3) / 3.542113654178) + 0.5;
	
	vec4 color;
	//color = marble_color(marble4);
	//color = cloud_color(marble4);
	color = mossyrock_color(marble4);
	
	// for some reason the colors are awfully dark
	// I think it looks better this way
	//color *= 2.85;
/*
	vec4 diffuse  = color * max(0.0, dot(n, s.xyz)) * gl_LightSource[0].diffuse;
	vec4 specular;
	if (shininess != 0.0) {
		specular = lightspec * matspec * pow(max(0.0, dot(r, v)), shininess);
	} else {
		specular = vec4(0.0, 0.0, 0.0, 0.0);
	}
*/	
	//gl_FragColor = vec4((marble4 / 3.542113654178) + 0.5);
	
	float gr = gl_FragCoord.x/790.0;
	vec4 col;
	if (gl_FragCoord.y < 572.0 / 2.0) {
		//col = marble_color((gr - 0.5) * 3.542113654178) * 2.85;
		//col = cloud_color((gr - 0.5) * 3.542113654178);
		col =mossyrock_color((gr - 0.5) * 3.542113654178);
	} else {
		col = vec4(gr);
	}
	gl_FragColor = col;
//	gl_FragColor = noise4(pos) != 0.0 ? vec4(1.0, 0.0, 0.0, 1.0) : vec4(0.0, 0.0, 1.0, 1.0);

}

vec4 marble_color(float m) {
	vec4 c[25];
	
	c[0] = PALE_BLUE;
	c[1] = PALE_BLUE;
	c[2] = MEDIUM_BLUE;
	c[3] = MEDIUM_BLUE;
	c[4] = MEDIUM_BLUE;
	c[5] = PALE_BLUE;
	c[6] = PALE_BLUE;
	c[7] = DARK_BLUE;
	c[8] = DARK_BLUE;
	c[9] = DARKER_BLUE;
	c[10] = DARKER_BLUE;
	c[11] = PALE_BLUE;
	c[12] = DARKER_BLUE;
	
	vec4 res = spline(clamp(2.0*m + 0.75, 0.0, 1.0), 13, c);
	
	return res;
}

// roughly based on the clouds in tex&mod, page 449
vec4 cloud_color(float value) {
	float threshold = -0.15;
	vec4 res = mix(SKYCOLOR, CLOUDCOLOR, smoothstep(threshold, 1.0, value));
	return res;
}

vec4 mossyrock_color(float value) {
	vec4 c[25];
	
	c[0]=vec4(0.241, 0.255, 0.212, 1.0);
	c[1]=vec4(0.241, 0.255, 0.212, 1.0);
	c[2]=vec4(0.290, 0.300, 0.265, 1.0);
	c[3]=vec4(0.305, 0.321, 0.275, 1.0);
	c[4]=vec4(0.293, 0.314, 0.240, 1.0);
	c[5]=vec4(0.125, 0.325, 0.135, 1.0);
	c[6]=vec4(0.125, 0.325, 0.135, 1.0);
	c[6]=vec4(0.115, 0.335, 0.130, 1.0);

	vec4 res = spline((value/3.542113654178)+ 0.5, 8, c);
	return res;
}