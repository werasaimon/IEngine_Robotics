#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform mat4 u_ProjectionLightMatrix;
uniform mat4 u_ShadowLightMatrix;
uniform mat4 u_ModelMatrix;

varying vec4 v_position;

attribute vec4 a_position;
attribute vec2 a_textcoord;
attribute vec4 a_normal;
attribute vec4 a_tangent;



void main(void)
{
    v_position = u_ProjectionLightMatrix * u_ShadowLightMatrix * u_ModelMatrix * a_position;
    gl_Position = v_position;

}
