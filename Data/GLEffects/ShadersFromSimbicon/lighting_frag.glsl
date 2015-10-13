vec3 LightPosition = vec3(0, 0, 100);
varying vec3  ecPosition;
varying vec3  tnorm;

    const float SpecularContribution = 0.2;
    const float DiffuseContribution  = 1.0 - SpecularContribution;

varying vec4  vColor;

void main(){
    vec4  color;
    vec3 lightVec   = normalize(LightPosition - ecPosition);
    vec3 norm = normalize(tnorm);
    vec3 reflectVec = reflect(-lightVec, norm);
    vec3 viewVec    = normalize(-ecPosition);
    float diffuse   = max(dot(lightVec, norm), 0.0);
    float spec      = 0.0;

    if (diffuse > 0.0) {
        spec = max(dot(reflectVec, viewVec), 0.0);
        spec = pow(spec, 16.0);
    }

    float LightIntensity  = DiffuseContribution * diffuse +
                      SpecularContribution * spec;

	LightIntensity = LightIntensity + 0.25;


 
    color = vColor;
    color *= LightIntensity;
    color.a = 1.0;
    gl_FragColor = color;
}