#VERT

#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

out vec2 TexCoord;

void main()
{
    gl_Position = vec4(aPos.x * 2.0f, aPos.y * 2.0f, 0.999f, 1.0);
    TexCoord = aTexCoord;
}

#FRAG

#version 330 core
out vec4 FragColor;

uniform sampler2D u_texture0; 

in vec2 TexCoord;

void main()
{
    FragColor = texture(u_texture0, TexCoord);
}