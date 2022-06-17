#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif


const int COUNT_LIGHTS = 5; // максимально допустимое количество источников света

uniform highp mat4 u_ProjectionMatrix;
uniform highp mat4 u_ViewMatrix;
uniform highp mat4 u_ModelMatrix;

uniform highp mat4 u_NormalMatrix;


struct ShadowProperty
{
    mat4 ShadowLightMatrix;
    mat4 ProjectionLightMatrix;
};

uniform ShadowProperty u_ShadowProperty[COUNT_LIGHTS];

// Texture shadow coordinates 4d
varying highp vec4 ShadowMapTexCoord[COUNT_LIGHTS];

uniform int u_CountLights;          // реальное количество источников освещения (<= COUNT_LIGHTS)


attribute highp vec4 a_position;
attribute highp vec2 a_textcoord;
attribute highp vec4 a_color;
attribute highp vec4 a_normal;
attribute highp vec4 a_tangent;

varying highp vec4 v_position;
varying highp vec2 v_textcoord;
varying highp vec4 v_color;
varying highp vec3 v_normal;
varying highp mat3 v_tbnMatrix;



highp mat3 transpose(in highp mat3 inMatrix)
{
    highp vec3 i0 = inMatrix[0];
    highp vec3 i1 = inMatrix[1];
    highp vec3 i2 = inMatrix[2];

    highp mat3 outMatrix = mat3(
                vec3(i0.x, i1.x, i2.x),
                vec3(i0.y, i1.y, i2.y),
                vec3(i0.z, i1.z, i2.z));
    return outMatrix;
}


highp mat4 transpose(in highp mat4 inMatrix)
{
    highp vec4 i0 = inMatrix[0];
    highp vec4 i1 = inMatrix[1];
    highp vec4 i2 = inMatrix[2];
    highp vec4 i3 = inMatrix[3];

    highp mat4 outMatrix = mat4(
                vec4(i0.x, i1.x, i2.x, i3.x),
                vec4(i0.y, i1.y, i2.y, i3.y),
                vec4(i0.z, i1.z, i2.z, i3.z),
                vec4(i0.w, i1.w, i2.w, i3.w));
    return outMatrix;
}



void main()
{


    mat4 mv_matrix = u_ViewMatrix * u_ModelMatrix;
    gl_Position = u_ProjectionMatrix * mv_matrix * a_position;

    v_position = u_ModelMatrix * a_position;

    //-----------------------------------------//

    vec3 normal  = normalize(a_normal.xyz);
    vec3 tangent = normalize(a_tangent.xyz);

    tangent = normalize(tangent - dot(tangent, normal) * normal);
    vec3 bitangent = cross(tangent, normal);

    vec3 i_tangent   = normalize(vec4(u_NormalMatrix * vec4(tangent, 0.0)).xyz);
    vec3 i_bitangent = normalize(vec4(u_NormalMatrix * vec4(bitangent, 0.0)).xyz);
    vec3 i_normal    = normalize(vec4(u_NormalMatrix * vec4(normal, 0.0)).xyz);

    v_normal    = i_normal;
    v_tbnMatrix = transpose(mat3(i_tangent, i_bitangent, i_normal));

    //-----------------------------------------//


    for(int i=0; i < u_CountLights; ++i)
    {
        ShadowMapTexCoord[i] = u_ShadowProperty[i].ProjectionLightMatrix *
                               u_ShadowProperty[i].ShadowLightMatrix * u_ModelMatrix * a_position;
    }

    // Pass texture coordinate to fragment shader
    // Value will be automatically interpolated to fragments inside polygon faces
    v_textcoord = a_textcoord;
    v_color = a_color;
}
//! [0]
