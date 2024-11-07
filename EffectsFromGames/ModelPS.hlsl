
SamplerState linearSampler : register( s0 );

SamplerState pointSampler : register( s1 );

Texture2D  texNormal : register( t1 );

TextureCube  texCube : register( t2 );

Texture2D  texColor : register( t3 );

Texture2D  texColor1 : register( t4 );

Texture2D  texColor2 : register( t5 );

Texture2D  texDepth : register( t6 );

cbuffer cbPostProccess : register( b1 )
{
	float     ScanDistance;
	float     ScanWidth;
    float     dummy1;
    float     dummy2;
	float4    ScanerPosition;
};

void lighting_calculations(
        float3 LightPos, 
        float3 VertexPos, 
        float3 VertexNormal,

        float3 AmbiColor,
        float3 LightColor,

		out float3 ReflectiveLightColor
)
{
    float3 Ln = normalize(LightPos - VertexPos);    
    float3 Nn = VertexNormal;
    float3 Vn = normalize( - VertexPos);
    float3 Hn = normalize(Vn + Ln);
    float4 litV = lit(dot(Ln,Nn),dot(Hn,Nn),0);
    
    ReflectiveLightColor = litV.y * LightColor + AmbiColor;
}

float3 decodeNormal(float4 normal){
   float3 _half = float3(0.5,0.5,0.5); 
   return (normal.xyz - _half)/_half;
}

float NonLinearToLinearDepth(float depth, float near, float far)
{
    return near * far / (far - depth * (far - near));
}

float3x3 GetTBN(float3 t, float3 b, float3 n)
{
    float3 _t = normalize(t);
    float3 _b = normalize(b);
    float3 _n = normalize(n);    

    float3x3 tbn = { 
        _t.x, _t.y, _t.z,
	    _b.x, _b.y, _b.z,
	    _n.x, _n.y, _n.z,
    }; 

    return tbn;
}

float3 GetCameraPositionFromDepth(float2 texcoords, float2 focalLen, float eye_z)
{
    texcoords.xy -= float2(0.5, 0.5);
    texcoords.xy /= float2(0.5, -0.5);
    float2 eye_xy = (texcoords.xy / focalLen.xy) * eye_z;

    return float3(eye_xy, eye_z);
}

struct Targets
{
    float4 color: SV_Target0;
};

///////////////////////////////////////////////////////////////////////////////////////////////

Targets EVE_PS(
	float2 tex_coord      : TEXCOORD1,
	float3 t              : TEXCOORD2,
	float3 b              : TEXCOORD3,
	float3 n              : TEXCOORD4,
	float3 light_pos      : TEXCOORD5,
	float3 pos            : TEXCOORD6,
    float4 clip_pos       : SV_POSITION
):SV_TARGET
{ 
    Targets output;

    float3 AmbiColor = float3(0.1,0.1,0.1);
    float3 LightColor = float3(1.,1.,1.);
    float4 MaterialColor = texColor.Sample(linearSampler, tex_coord);
    float3 ReflectiveLightColor = float3(1.,1.,1.); // changing by lighting_calculations

    float3 fragNormal = normalize(decodeNormal(texNormal.Sample(linearSampler, tex_coord)));
    fragNormal = normalize(mul(fragNormal, GetTBN(t,b,n)));

    lighting_calculations(light_pos, pos, fragNormal, AmbiColor, LightColor, ReflectiveLightColor);

    output.color = float4(clamp(ReflectiveLightColor*MaterialColor,0,1), 1);

    return output;
}

///////////////////////////////////////////////////////////////////////////////////////////////
float triWave(float t, float offset, float yOffset)
{
    float x = abs( frac(t + offset) );
    float y = abs( x * 2 - 1 );
    return saturate( y + yOffset );
}

Targets WINSTONS_BARRIER_PS(
    float2 tex            : TEXCOORD1,
    float4 screenAndNearFar   : TEXCOORD2,
    float3 objectPos:     TEXCOORD3,
	float4 time:          TEXCOORD4,
	float3 normal:        TEXCOORD5, 
	float3 viewPos:       TEXCOORD6, 
    float4 clip_pos       : SV_POSITION
):SV_TARGET
{ 
   Targets output;
   
   float near = screenAndNearFar.z;
   float far = screenAndNearFar.w;
   float oneDivFar = 1.0/far;

   float3 mainTexture = texColor.Sample(linearSampler, tex).rgb;

   float fragDepth = clip_pos.w/far;
   float sampleDepth = NonLinearToLinearDepth(texDepth.SampleLevel(pointSampler, clip_pos.xy/screenAndNearFar.xy, 0).x, near, far)/far;

   float diff = sampleDepth - fragDepth;

   float intersect = 0;
   if(diff>.0)
        intersect = 1 - smoothstep(0, oneDivFar*0.5f, diff);

   float rim = 1.0 - abs(dot(normalize(-viewPos), normalize(normal))) * 2.0;
   float northGlow = (objectPos.y - 0.45) * 20.0;
   float glow = max(max(rim, intersect), northGlow);

   float3 _color = float3(9.0/255.0,50.0/255.0,128.0/255.0);
   float3 _barrier_border = glow * lerp(_color, float3(1.0, 1.0, 1.0), pow(glow, 4));
   float3 _hexes = 12.0 * triWave(time.x, abs(2.0 * objectPos.y), -0.7) * mainTexture.r * _color;
   float3 _rim_hexes = (sin(time.x*15.0 + mainTexture.b * 5) + 1.0) * saturate(rim) * mainTexture.g * _color;

   output.color = float4(_color + _barrier_border + _hexes + _rim_hexes, 1);

   return output;
};

///////////////////////////////////////////////////////////////////////////////////////////////
Targets SKY_PS(
    float3 tex            : TEXCOORD1,
    float4 clip_pos       : SV_POSITION
):SV_TARGET
{ 
   Targets output;

   output.color = float4(texCube.Sample(linearSampler, tex).rgb, 1);

   return output;
};

///////////////////////////////////////////////////////////////////////////////////////////////
Targets GROUND_PS(
    float2 tex            : TEXCOORD0,
	float3 t              : TEXCOORD1,
	float3 b              : TEXCOORD2,
	float3 n              : TEXCOORD3,  
	float3 pos            : TEXCOORD4,
	float3 light_pos      : TEXCOORD5,	
    float4 clip_pos       : SV_POSITION
):SV_TARGET
{ 
   Targets output;

   float3 _t = normalize(t);
   float3 _b = normalize(b);
   float3 _n = normalize(n);
   
   float3x3 tbn = { 
    	_t.x, _t.y, _t.z,
	    _b.x, _b.y, _b.z,
	    _n.x, _n.y, _n.z,
   }; 

   float3 normal = decodeNormal(texNormal.Sample(linearSampler, tex));

   normal = normalize(mul(normal, tbn));

   float3 MaterialColor = texColor.Sample(linearSampler, tex);

   float3 LightColor = float3(1.,1.,1.);
   float3 AmbiColor = float3(0.1,0.1,0.1);

   float3 ReflectiveLightColor;
   lighting_calculations(light_pos, pos, normal, AmbiColor, LightColor, ReflectiveLightColor);

   output.color = float4(clamp(1.0*ReflectiveLightColor*MaterialColor,0,1), 1);

   return output;
};

///////////////////////////////////////////////////////////////////////////////////////////////
Targets BOX_PS(
	float3 normal     : TEXCOORD1,
	float3 fragPos    : TEXCOORD2,
	float3 lightPos   : TEXCOORD3,
    float4 clip_pos   : SV_POSITION
):SV_TARGET
{ 
   Targets output;

   float3 fragNormal = normalize(normal);

   float3 MaterialColor = float3(1.0,1.0,1.0);

   float3 LightColor = float3(1.,1.,1.);
   float3 AmbiColor = float3(0.1,0.1,0.1);

   float3 ReflectiveLightColor;
   lighting_calculations(lightPos, fragPos, fragNormal, AmbiColor, LightColor, ReflectiveLightColor);

   output.color = float4(clamp(ReflectiveLightColor * MaterialColor,0,1), 1);

   return output;
};

///////////////////////////////////////////////////////////////////////////////////////////////
Targets BOX_GLOW_PS(
	float3 normal     : TEXCOORD1,
	float3 fragPos    : TEXCOORD2,
	float3 lightPos   : TEXCOORD3,
    float4 clip_pos   : SV_POSITION
):SV_TARGET
{ 
   Targets output;

   float3 fragNormal = normalize(normal);

   float3 MaterialColor = float3(0.0,1.0,0.0);

   output.color = float4(clamp(MaterialColor,0,1), 1);

   return output;
};

///////////////////////////////////////////////////////////////////////////////////////////////
float3 Gaussian(float d, float3 sigma)
{
    const float pi = 3.14159274;
	float3 sigma_prime = sigma * sigma * 2;
	return	exp(- abs(d) / sigma_prime) / ( pi * sigma_prime);
}
float4 BlurBufferPS (in float2 TexCoord, float2	buffer_texel_size, bool horizontal) 
{
	float4 sum =  float4(0,0,0,0);
	
    const int blur_search_width = 2;

    const int blur_search_start = -blur_search_width;
    const int blur_search_end = blur_search_width + 1;
    const float  blur_scale = 2.0f;
    const float3 BlurSigma = float3(2.0,2.0,2.0);

	if(horizontal)
	{
	
		for(int i = blur_search_start;  i < blur_search_end; ++i)
			sum.rgb += Gaussian(i, BlurSigma) * texColor.SampleLevel(linearSampler, TexCoord + buffer_texel_size * float2(0.5f + 2.0f * i,0.5f) ,0).rgb;
	}
	else
	{
		for(int i = blur_search_start;  i < blur_search_end; ++i)
			sum.rgb += Gaussian(i, BlurSigma) * texColor.SampleLevel(linearSampler, TexCoord + buffer_texel_size * float2(0.5f, 0.5f + 2 * i) ,0).rgb;

	}

	return blur_scale * sum;
}
Targets BLUR_HORIZONTAL_PS(
    float2 tex : TEXCOORD1,
	float4 screenAndProjection     : TEXCOORD2,
	float2 nearAndFarZ            : TEXCOORD3,    
    float4 clip_pos   : SV_POSITION
):SV_TARGET
{ 
   Targets output;
 
   //output.color = BlurBufferPS(float2(clip_pos.x/screenSize.x, clip_pos.y/maxScreenCords.y), float2(1.0/screenSize.x, 1.0/maxScreenCords.y), true);
   //output.color = BlurBufferPS(clip_pos.xy/(0.5*screenAndProjection.xy), float2(1.0/(int(screenAndProjection.x)/int(2)), 1.0/(int(screenAndProjection.y)/int(2))), true);
   output.color = BlurBufferPS(tex, float2(1.0/(int(screenAndProjection.x)/int(2)), 1.0/(int(screenAndProjection.y)/int(2))), true);
   //output.color = texColor.SampleLevel(linearSampler, tex, 0);

   return output;
};
Targets BLUR_VERTICAL_PS(
    float2 tex : TEXCOORD1,
	float4 screenAndProjection     : TEXCOORD2,
	float2 nearAndFarZ            : TEXCOORD3,    
    float4 clip_pos   : SV_POSITION
):SV_TARGET
{ 
   Targets output;

   //output.color = BlurBufferPS(float2(clip_pos.x/screenSize.x, clip_pos.y/maxScreenCords.y),float2(1.0/screenSize.x, 1.0/maxScreenCords.y), false);
   //output.color = BlurBufferPS(clip_pos.xy/(0.5*screenAndProjection.xy), float2(1.0/(int(screenAndProjection.x)/int(2)), 1.0/(int(screenAndProjection.y)/int(2))), false);
   output.color = BlurBufferPS(tex, float2(1.0/(int(screenAndProjection.x)/int(2)), 1.0/(int(screenAndProjection.y)/int(2))), false);
   //output.color = texColor.SampleLevel(linearSampler, tex, 0);

   return output;
};

///////////////////////////////////////////////////////////////////////////////////////////////
float horizontalBar(float y)
{
   return 1 - saturate(round(frac(y*100)*2));
}

Targets POST_PROCCESS_PS(
    float2 tex : TEXCOORD1,
	float4 screenAndProjection     : TEXCOORD2,
	float2 nearAndFarZ            : TEXCOORD3,    
    float4 clip_pos   : SV_POSITION
):SV_TARGET
{ 
   Targets output;

   float3 scene = texColor2.SampleLevel(linearSampler, tex, 0).rgb;

   //blur effect
   {
        float3 glow = texColor.SampleLevel(linearSampler, tex, 0).rgb;
        float3 glowBlur = texColor1.SampleLevel(linearSampler, tex, 0).rgb;
        
        float3 glowColor = 25.0 * max(0, glowBlur - glow);

        //scene += glowColor;
   }

   //scanner effect
   {
        float cameraSpaceNonLinearDepth = texDepth.SampleLevel(pointSampler, tex, 0).x;
        float cameraSpaceLinearDepth = NonLinearToLinearDepth(cameraSpaceNonLinearDepth, nearAndFarZ.x, nearAndFarZ.y);
        float3 cameraSpacePos = GetCameraPositionFromDepth(tex, screenAndProjection.zw, cameraSpaceLinearDepth);

        float dist = length(cameraSpacePos - ScanerPosition.xyz);

        if((ScanDistance - ScanWidth) < dist && dist < ScanDistance)
        {
            float diff = 1.0 - (ScanDistance-dist)/ScanWidth;

            float3 _Lead = float3(255.0/255.0, 255.0/255.0, 255.0/255.0);
            float3 _Mid = float3(0.0/255.0, 84.0/255.0, 18.0/220.0);
            float3 _Trail = float3(0.0/255.0, 202.0/255.0, 236.0/255.0);
        
            float3 _HBarColor = float3(244.0/255.0,223.0/255.0,109.0/255.0);

            float3 scannerColor = lerp(_Trail, lerp(_Mid, _Lead, pow(diff, 25)), diff) + horizontalBar(tex.y)*_HBarColor;
            
            scannerColor *= diff;

            scene += scannerColor;
        }
   }

   //set output
   {
       output.color = float4(scene, 1);
   }

   return output;
};

///////////////////////////////////////////////////////////////////////////////////////////////
Targets COPY_PS(
    float2 tex : TEXCOORD1,
	float4 screenAndProjection     : TEXCOORD2,
  	float2 nearAndFarZ            : TEXCOORD3,    
    float4 clip_pos   : SV_POSITION
):SV_TARGET
{ 
   Targets output;
   
   output.color = texColor.SampleLevel(linearSampler, tex, 0);

   return output;
};

///////////////////////////////////////////////////////////////////////////////////////////////
Targets EVE_PATH_PS(
    float4 clip_pos   : SV_POSITION
):SV_TARGET
{ 
   Targets output;
   
   output.color = float4(1,0,0,1);

   return output;
};

///////////////////////////////////////////////////////////////////////////////////////////////
Targets UNLIT_WHITE_PS(
    float4 clip_pos   : SV_POSITION,
    float4 color      : TEXCOORD1
):SV_TARGET
{ 
   Targets output;
   
   output.color = color;

   return output;
};
///////////////////////////////////////////////////////////////////////////////////////////////

Targets WATER_PS(
    float2 tex1           : TEXCOORD1,
    float4 tex2           : TEXCOORD2,
    float3 waterParams    : TEXCOORD3,
	float3 extArg    	  : TEXCOORD4,
    float4 clip_pos       : SV_POSITION
):SV_TARGET
{ 
   Targets output;

   //////water
   // Move the position the water normal is sampled from to simulate moving water.	
   float2 normalMapTiling = float2(0.01f, 0.02f);
   float2 tex_1 = tex1 / normalMapTiling.x;
   float2 tex_2 = tex1 / normalMapTiling.y;
   tex_1.y += waterParams.z;
   tex_2.y += waterParams.z;
   
   // Sample the normal from the normal map texture using the two different tiled and translated coordinates.
   float4 normalMap1 = texNormal.Sample(linearSampler, tex_1);
   float4 normalMap2 = texNormal.Sample(linearSampler, tex_2);
   float3 normal1 = (normalMap1.rgb * 2.0f) - 1.0f;
   float3 normal2 = (normalMap2.rgb * 2.0f) - 1.0f;
   float3 normal = normalize(normal1 + normal2);
   //////water 

   ///refr   
   float refractScale = 0.03f;
   float4 refractionTint =  float4(0.0f, 0.8f, 1.0f, 1.0f);

   float2 refrUV = float2(clip_pos.x/waterParams.x, clip_pos.y/waterParams.y);
   refrUV = refrUV + (normal.xy * refractScale);
   float4 refrColor = saturate(texColor1.Sample(linearSampler, refrUV) * refractionTint);

   ///refl   
   float reflectScale = 0.03f;

   float2 reflUV = float2(tex2.x / tex2.w * 0.5 + 0.5, tex2.y / tex2.w * -0.5 + 0.5);
   reflUV = reflUV + (normal.xy * reflectScale);
   float4 reflColor = texColor2.Sample(linearSampler, reflUV);

   //////////////////////////////////////////////////////////////////////////////////////////////

   // Get a modified viewing direction of the camera that only takes into account height.
   float3 heightView;
   heightView.x = extArg.y;
   heightView.y = extArg.y;
   heightView.z = extArg.y;

   // Now calculate the fresnel term based solely on height.
   float r = (1.2f - 1.0f) / (1.2f + 1.0f);
   float fresnelFactor = max(0.0f, min(1.0f, r + (1.0f - r) * pow(1.0f - dot(normal, heightView), 2)));

   float4 resultColor =  lerp(reflColor, refrColor, fresnelFactor);

   output.color = resultColor;
   
   return output;
};
/////////////////////////////////////////////////////////////////////