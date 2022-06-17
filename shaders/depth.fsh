#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

varying vec4 v_position;

//uniform sampler2D gShadowMap;
//out vec4 FragColor;


float near_plane = 300.f;
float far_plane = 10.f;

float LinearizeDepth(float depth)
{
    float z = depth * 2.0 - 1.0; // Back to NDC
    return (2.0 * near_plane * far_plane) / (far_plane + near_plane - z * (far_plane - near_plane));
}


out float fragmentdepth;
//out vec4  outColor;

void main(void)
{
    float depth = v_position.z / v_position.w;
    depth = depth * 0.5 + 0.5;

    float v1 = depth * 255.0;
    float f1 = fract(v1);
    float vn1 = floor(v1) / 255.0;

    float v2 = f1 * 255.0;
    float f2 = fract(v2);
    float vn2 = floor(v2) / 255.0;

    float v3 = f2 * 255.0;
    float f3 = fract(v3);
    float vn3 = floor(v3) / 255.0;

    gl_FragColor = vec4(vn1, vn2, vn3, f3);

    //fragmentdepth = depth;//gl_FragCoord.z;
}
