#version 300 es
#undef lowp
#undef mediump
#undef highp

precision mediump float;

uniform sampler2D text;
uniform vec4 textColor;
in vec2 v_texcoord;
out vec4 out_color;


//! [0]
void main()
{
        vec4 sampled = vec4(1.0, 1.0, 1.0, texture2D(text, v_texcoord).a);
	out_color = sampled * vec4(textColor);
}
//! [0]

