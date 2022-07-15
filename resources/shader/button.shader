#VERT
#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

uniform mat4 u_mvp;

out vec2 TexCoord;

void main()
{
    gl_Position = u_mvp * vec4(aPos, 1.0);
    TexCoord = aTexCoord;
}

#FRAG
#version 330 core
out vec4 FragColor;

uniform sampler2D u_texture0; 

in vec2 TexCoord;

void main()
{
    FragColor = texture(u_texture0, vec2(-TexCoord.x, TexCoord.y));
}