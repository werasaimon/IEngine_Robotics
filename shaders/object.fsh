#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif


const int COUNT_LIGHTS = 5; // максимально допустимое количество источников света

struct MaterialProperty
{
    vec3  DiffuseColor;
    vec3  AmbienceColor;
    vec3  SpecularColor;
    float Shines;
};


struct LightProperty
{
    vec3 AmbienceColor;
    vec3 DiffuseColor;
    vec3 SpecularColor;
    vec4 ReflectionColor;
    vec4 Position;
    vec4 Direction;
    float Cutoff;
    float Power;
    int Type; // Direct - 0, Point - 1, Spot - 2
};

uniform MaterialProperty u_MaterialProperty;

uniform bool u_IsUseDiffuseMap;
uniform bool u_IsUseNormalMap;
uniform bool u_IsDrawShadow;

uniform highp float u_ShadowMapSize;
uniform highp float u_ShadowPointCloudFilteringQuality;
uniform highp float u_MainLightPower;// сила освещения

uniform LightProperty u_LightProperty[COUNT_LIGHTS];
uniform int u_CountLights;          // реальное количество источников освещения (<= COUNT_LIGHTS)

uniform highp vec3 u_eyePosition; // World position of the camera

// Texture shadow coordinates 4d
varying highp vec4 ShadowMapTexCoord[COUNT_LIGHTS];


uniform sampler2D u_DiffuseMapTexture;
uniform sampler2D u_NormalMapTexture;
uniform sampler2D u_ShadowMapTexture[COUNT_LIGHTS];

varying highp vec4 v_position;
varying highp vec2 v_textcoord;
varying highp vec4 v_color;
varying highp vec3 v_normal;
varying highp mat3 v_tbnMatrix;

//--------------------------------------------------------------------------------//

uniform bool u_IsUseShadowPFC;

//--------------------------------------------------------------------------------//
float SampleShadowMap(sampler2D map, vec2 coords, float compare)
{
    vec4  v = texture2D(map, coords);
    float value = v.x * 255.0 + (v.y * 255.0 + (v.z * 255.0 + v.w) / 255.0) / 255.0;
    return step(compare, value);
}

float SampleShadowMapLinear(sampler2D map, vec2 coords, float compare, vec2 texelsize)
{
    vec2 pixsize = coords / texelsize + 0.5;
    vec2 pixfractpart = fract(pixsize);
    vec2 starttexel = (pixsize - pixfractpart) * texelsize;
    float bltexel = SampleShadowMap(map, starttexel, compare);
    float brtexel = SampleShadowMap(map, starttexel + vec2(texelsize.x, 0.0), compare);
    float tltexel = SampleShadowMap(map, starttexel + vec2(0.0, texelsize.y), compare);
    float trtexel = SampleShadowMap(map, starttexel + texelsize, compare);

    float mixL = mix(bltexel, tltexel, pixfractpart.y);
    float mixR = mix(brtexel, trtexel, pixfractpart.y);

    return mix(mixL, mixR, pixfractpart.x);
}

// point cloud filtering
float SampleShadowMapPCF(sampler2D map, vec2 coords, float compare, vec2 texelsize)
{
    float result = 0.0;
    float spcfq = u_ShadowPointCloudFilteringQuality;
    for(float y = -spcfq; y < spcfq; y += 1.0)
        for(float x = -spcfq; x < spcfq; x += 1.0)
        {
            vec2 offset = vec2(x, y) * texelsize;
            result += SampleShadowMapLinear(map, coords + offset, compare, texelsize);
        }
    return result / 9.0;
}

float CalcShadowAmount(sampler2D map, vec4 plm , float dotNV , int index )
{
    vec3 value = (plm.xyz / plm.w) * vec3(0.5) + vec3(0.5);
    float offset = 3.8 * dotNV; // первый коэффициент должен влиять на качество тени
    return SampleShadowMapPCF(u_ShadowMapTexture[index], value.xy, value.z * 255.0 + offset, vec2(1.0 / u_ShadowMapSize));
}

////--------------------------------------------------------------------------------//



//! [0]
void main()
{

    int countLights = u_CountLights;
    if(countLights > COUNT_LIGHTS) countLights = COUNT_LIGHTS;


    vec3 eyeVec = normalize(v_position.xyz - u_eyePosition.xyz); // направление взгляда
    vec3 usingNormal = v_normal; // используемая нормаль
    vec4 diffMatColor = texture2D(u_DiffuseMapTexture, v_textcoord); // диффузный цвет


    vec4 SumaColorDiffuse = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    vec4 SumaColorSpecular = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    vec4 SumaColorAmbient = vec4(0.0f, 0.0f, 0.0f, 0.0f);


    if(u_IsUseNormalMap) usingNormal = normalize(texture2D(u_NormalMapTexture, v_textcoord).rgb * 2.0f - 1.0f);
    if(u_IsUseNormalMap) eyeVec = normalize(v_tbnMatrix * eyeVec);


    vec2 poissonDisk[4];
    poissonDisk[0] = vec2( -0.94201624, -0.39906216  );
    poissonDisk[1] = vec2( 0.94558609, -0.76890725   );
    poissonDisk[2] = vec2( -0.094184101, -0.92938870 );
    poissonDisk[3] = vec2( 0.34495938, 0.29387760    );



    for(int i = 0; i < countLights; i++)
    {
        vec3 lightVec = vec3(0.0f, 0.0f, 0.0f); // вектор освещения
        vec4 resultLightColor = vec4(0.0f, 0.0f, 0.0f, 0.0f); // результирующий цвет освещения

        if(u_LightProperty[i].Type == 0) // Directional
        {
            lightVec = normalize(u_LightProperty[i].Direction.xyz);
        }
        else // Point, Spot
        {
            lightVec = normalize(v_position - u_LightProperty[i].Position).xyz;
            if(u_LightProperty[i].Type == 2) // Spot
            {
                float angle = acos(dot(u_LightProperty[i].Direction.xyz, lightVec));
                if(angle > u_LightProperty[i].Cutoff) lightVec = vec3(0.0f, 0.0f, 0.0f);
            }
        }

        if(u_IsUseNormalMap) lightVec = normalize(v_tbnMatrix * lightVec);

        vec3 reflectLight = normalize(reflect(lightVec, usingNormal)); // отражённый свет
        float len = length(v_position.xyz - u_eyePosition.xyz); // расстояние от наблюдателя до точки
        float specularFactor = u_MaterialProperty.Shines; // размер пятна блика
        float ambientFactor = 0.25f; // светимость материала

        if(u_IsUseDiffuseMap == false) diffMatColor = vec4(u_MaterialProperty.DiffuseColor, 1.0f);


        //=======================================================================================//

        float intensive = 1.f;
        if(u_IsDrawShadow)
        {
            if(u_IsUseShadowPFC)
            {
                intensive = (CalcShadowAmount(u_ShadowMapTexture[i], ShadowMapTexCoord[i] , dot(usingNormal, lightVec) , i ));
                intensive += 0.15; // избавляемся от абсолютной черноты тени
                if(intensive > 1.0) intensive = 1.0;
            }
            else
            {


                float bias = 0.005*tan(acos(dot(usingNormal, lightVec)));
                bias = clamp(bias, 0.0 , 0.015);

                if(ShadowMapTexCoord[i].w > 0.0 )
                {
                    vec3 ShadowMapTexCoordProj = (ShadowMapTexCoord[i].xyz / ShadowMapTexCoord[i].w) * vec3(0.5) + vec3(0.5);;

                    if(ShadowMapTexCoordProj.x >= 0.0 && ShadowMapTexCoordProj.x < 1.0 &&
                       ShadowMapTexCoordProj.y >= 0.0 && ShadowMapTexCoordProj.y < 1.0 &&
                       ShadowMapTexCoordProj.z >= 0.0 && ShadowMapTexCoordProj.z < 1.0)
                    {
                        for (int j=0;j<4;j++)
                        {
                            if ( texture2D( u_ShadowMapTexture[i], ShadowMapTexCoordProj.xy + poissonDisk[j]/700.0 ).r < ShadowMapTexCoordProj.z-bias)
                            {
                                intensive -= 0.2;
                            }
                        }
                      }
                  }


               }

         }

        //----------------------------------- Ambient -------------------------------------//


        vec4 ambientColor = ambientFactor * diffMatColor;
        SumaColorAmbient += ambientColor * vec4(u_MaterialProperty.AmbienceColor, 1.0f) *
                                           vec4(u_LightProperty[i].AmbienceColor, 1.0f);

        //--------------------------------- Reflection -------------------------------------//

        vec4 specularColor =  u_LightProperty[i].ReflectionColor *
                              u_LightProperty[i].Power *
                              pow(max(0.0f, dot(reflectLight, -eyeVec)), specularFactor);

        SumaColorSpecular += intensive * specularColor *
                             vec4(u_MaterialProperty.SpecularColor, 1.0f) *
                             vec4(u_LightProperty[i].SpecularColor, 1.0f);

        //----------------------------------- Diffuse ---------------------------------------//

        vec4 diffColor = diffMatColor * u_LightProperty[i].Power *
                         max(0.0f, dot(usingNormal, -lightVec));

         SumaColorDiffuse += intensive * diffColor *
                             vec4(u_LightProperty[i].DiffuseColor, 1.0f);

    }


    // Set fragment color from texture
    gl_FragColor = SumaColorAmbient + SumaColorDiffuse + SumaColorSpecular;
}
//! [0]

