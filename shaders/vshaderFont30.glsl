#version 300 es

uniform mat4 mvp_matrix;

in vec2 a_position;
in vec2 a_texcoord;

out vec2 v_texcoord;

//! [0]
void main()
{
	// Calculate vertex position in screen space
	gl_Position = mvp_matrix * vec4(a_position.xy, 0.0, 1.0);
	v_texcoord = a_texcoord;
}
//! [0]