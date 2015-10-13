#version 330
                                                                                                                 
in vec2 TexCoord0;                                                                  
in vec3 Normal0;                                                                    
in vec3 WorldPos0;                                                                  
                                                                                    
out vec4 FragColor;                                                                 
                                                                              
uniform vec3 gColor;                                                            
                                                                                         
void main()                                                                                 
{                          
    FragColor = vec4(gColor, 1.0);
}
