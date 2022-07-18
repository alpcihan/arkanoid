#VERT
#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

uniform mat4 u_model;

out vec2 TexCoord;

void main()
{
    gl_Position = u_model * vec4(aPos, 1.0);
    TexCoord = aTexCoord;
}

#FRAG
#version 330 core
out vec4 FragColor;

uniform sampler2D u_texture0; 

in vec2 TexCoord;

void main()
{
    vec4 tex = texture(u_texture0, TexCoord);
    vec4 color;

    float alpha = 1.0;

    // if transparent
    if((tex.x > 0.8) && (tex.y > 0.8) && (tex.z > 0.8))
    {
        alpha = 0.0;
    }
    color = tex * vec4(1.0, 1.0, 1.0, alpha);

    // if red
    if((tex.x > 0.8) && (tex.y < 0.1) && (tex.z < 0.1))
    {
        color = vec4(1.0, 1.0, 1.0, 1.0);
    }

    FragColor = color;
}