//
// Fragment shader for hemispherical lighting
//
// Author: Randi Rost
//
// Copyright (C) 2005 3Dlabs, Inc.
//
// See 3Dlabs-License.txt for license information
//

vec3 LightPosition = vec3(0,100,0);
vec3 SkyColor = vec3(1,1,1);
vec3 GroundColor = vec3(0,0,0);


varying vec3 vNorm;
varying vec3 ecPosition;

void main(void){
     vec3 lightVec   = normalize(LightPosition - ecPosition);
     float costheta  = dot(vNorm, lightVec);
     float a         = 0.5 + 0.5 * costheta;
 
     vec3 DiffuseColor    = mix(GroundColor, SkyColor, a);

     

     gl_FragColor = vec4(DiffuseColor, 1.0);
}
