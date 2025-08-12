cbuffer cbMain : register( b0 )
{
	matrix    g_mWorld;                         // World matrix
	matrix    g_mView;                          // View matrix
	matrix    g_mProjection;                    // Projection matrix
	matrix    g_mWorldViewProjection;           // WVP matrix
	matrix    g_mWorldView;                     // WV matrix
	matrix    g_mInvView;                       // Inverse of view matrix

	matrix    g_mObject1;                // VP matrix
	matrix    g_mObject1WorldView;                       // Inverse of view matrix
	matrix    g_mObject1WorldViewProjection;                       // Inverse of view matrix

	matrix    g_mObject2;                // VP matrix
	matrix    g_mObject2WorldView;                       // Inverse of view matrix
	matrix    g_mObject2WorldViewProjection;                       // Inverse of view matrix

	float4    g_vFrustumNearFar;              // Screen resolution
	float4    g_vFrustumParams;              // Screen resolution

	float4    g_Time;

	float4    g_ClipPlane0;
	float4    g_ClipPlane1;
	float4    g_ClipPlane2;	

	float4    g_MeshColor;
};

cbuffer cbBonePalite : register( b1 )
{
	matrix    g_BoneToModelMatrix[1024];
};
cbuffer cbEvePath: register( b2 )
{
	float g_EvePathCount;
	float     g_EvePathdummy1;
	float     g_EvePathdummy2;
	float     g_EvePathdummy3;	
	float4    g_EvePathPivots[32];
};
///////////////////////////////////////////////////////////////////////////////////////////////////
struct EVE_INPUT
{
    float3 Pos         : SV_Position;
    float3 Normal      : NORMAL;
    float4 Tangent     : TANGENT;
	float4 Color	   : COLOR0;
    float2 TexCoord0   : TEXCOORD0;
    uint4  Bones       : BLENDINDICES0;
    float4 Weights     : BLENDWEIGHT0;
    uint4  Bones1      : BLENDINDICES1;
    float4 Weights1    : BLENDWEIGHT1;
};

struct EVE_OUTPUT
{
	float2 tex_coord      : TEXCOORD1;
	float3 t              : TEXCOORD2;
	float3 b              : TEXCOORD3;
	float3 n              : TEXCOORD4;
	float3 light_pos      : TEXCOORD5;
	float3 pos            : TEXCOORD6;
    float4 clip_pos       : SV_POSITION;
	float clip            : SV_ClipDistance0;
};

EVE_OUTPUT EVE_VS( in EVE_INPUT input )
{
	EVE_OUTPUT output;
	
	float4 lightPos = float4( 20, 43, -24, 1 );                
    
	float4x4 finalMatrix = {0,0,0,0,   0,0,0,0,   0,0,0,0,   0,0,0,0};

	if(input.Weights.x != 0)
	{
	    finalMatrix += input.Weights.x * g_BoneToModelMatrix[input.Bones.x];
		if(input.Weights.y != 0) 
		{
			finalMatrix += input.Weights.y * g_BoneToModelMatrix[input.Bones.y];
			if(input.Weights.z != 0)
			{
				finalMatrix += input.Weights.z * g_BoneToModelMatrix[input.Bones.z];
				if(input.Weights.w != 0)
				{
					finalMatrix += input.Weights.w * g_BoneToModelMatrix[input.Bones.w];    
				}
			}
		}
	}
	if(input.Weights1.x != 0)
	{
	    finalMatrix += input.Weights1.x * g_BoneToModelMatrix[input.Bones1.x];
		if(input.Weights1.y != 0) 
		{
			finalMatrix += input.Weights1.y * g_BoneToModelMatrix[input.Bones1.y];
			if(input.Weights1.z != 0)
			{
				finalMatrix += input.Weights1.z * g_BoneToModelMatrix[input.Bones1.z];
				if(input.Weights1.w != 0)
				{
					finalMatrix += input.Weights1.w * g_BoneToModelMatrix[input.Bones1.w];    
				}
			}
		}
	}

    float3 ModelTangent = input.Tangent.xyz;
    float3 ModelBiTangent = cross(input.Normal, input.Tangent.xyz) * input.Tangent.w;

    float4 vAnimatedPos = mul(float4(input.Pos,                   1),finalMatrix);
    float4 vAnimatedTangent = mul(float4(ModelTangent,            0),finalMatrix);
    float4 vAnimatedBiTangent = mul(float4(ModelBiTangent,        0),finalMatrix);
    float4 vAnimatedNormal = mul(float4(input.Normal,             0),finalMatrix);

	output.clip_pos =   mul(vAnimatedPos, g_mWorldViewProjection);
	output.pos =        mul(vAnimatedPos, g_mWorldView).xyz;
	output.t = normalize(mul(vAnimatedTangent, g_mWorldView).xyz);
	output.b = normalize(mul(vAnimatedBiTangent, g_mWorldView).xyz);
	output.n = normalize(mul(vAnimatedNormal, g_mWorldView).xyz);

	output.light_pos = mul( lightPos, g_mView ).xyz;
	output.tex_coord = input.TexCoord0;

 	output.clip = dot( mul(vAnimatedPos, g_mWorld ), g_ClipPlane0 );

	return output;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
struct WINSTONS_BARRIER_INPUT
{
    float3 pos : SV_Position;
    float3 normal   : NORMAL;
    float2 tex      : TEXCOORD0;
};
struct WINSTONS_BARRIER_OUTPUT
{
	float2 tex            : TEXCOORD1;
	float4 screenAndNearFar   : TEXCOORD2;
	float3 objectPos:     TEXCOORD3;
	float4 time:          TEXCOORD4;
	float3 normal:        TEXCOORD5; 
	float3 viewPos:       TEXCOORD6; 
    float4 clip_pos       : SV_POSITION;
	float clip            : SV_ClipDistance0;
};
WINSTONS_BARRIER_OUTPUT WINSTONS_BARRIER_VS( in WINSTONS_BARRIER_INPUT i )
{
    WINSTONS_BARRIER_OUTPUT output;

    output.clip_pos = mul( float4( i.pos, 1.0 ), g_mWorldViewProjection );
    
	output.screenAndNearFar = float4(g_vFrustumParams.xy, g_vFrustumNearFar.xy);

	output.tex = i.tex*float2(6.0,12.0);

    output.objectPos = i.pos;

    output.time = g_Time;

	output.normal = normalize(mul( float4(i.normal,0), g_mWorldView ).xyz); 

	output.viewPos = mul( float4(i.pos,1), g_mWorldView ).xyz; 

 	output.clip = dot( mul(float4( i.pos, 1.0 ), g_mWorld ), g_ClipPlane0 );

    return output;
}; 

///////////////////////////////////////////////////////////////////////////////////////////////////
struct SKY_VS_INPUT
{
    float3 pos : SV_Position;
    float3 normal   : NORMAL;
    float2 tex      : TEXCOORD0;
};
struct SKY_VS_OUTPUT
{
	float3 tex            : TEXCOORD1;
    float4 clip_pos       : SV_POSITION;
	float clip            : SV_ClipDistance0;
};
SKY_VS_OUTPUT SKY_VS( in SKY_VS_INPUT i )
{
    SKY_VS_OUTPUT output;

    output.clip_pos = mul( float4( i.pos, 1.0 ), g_mWorldViewProjection ).xyww;
    
	output.tex = i.pos;

	output.clip = 1.0f;

    return output;
}; 

///////////////////////////////////////////////////////////////////////////////////////////////////
struct GROUND_VS_OUTPUT
{
    float2 tex            : TEXCOORD0;
	float3 t              : TEXCOORD1;
	float3 b              : TEXCOORD2;
	float3 n              : TEXCOORD3;  
	float3 pos            : TEXCOORD4;
	float3 light_pos      : TEXCOORD5;	
    float4 clip_pos       : SV_POSITION; // Output position
	float clip            : SV_ClipDistance0;
};

GROUND_VS_OUTPUT GROUND_VS(uint VertexID : SV_VERTEXID)
{
	GROUND_VS_OUTPUT output;

	output.clip_pos = float4(0,0,0,1);

    return output;	
}

static const float4 planeVerts[4] = 
{
	float4(0, 0, 0, 1),// LB  0
	float4(0, 1, 0, 1), // RB  2
	float4(1, 0, 0, 1), // LT  1
	float4(1, 1, 0, 1),  // RT  3
};

[maxvertexcount(4)]
void GROUND_GS(point GROUND_VS_OUTPUT pnt[1], uint primID : SV_PrimitiveID,  inout TriangleStream<GROUND_VS_OUTPUT> triStream )
{
	GROUND_VS_OUTPUT v[4];

	float4 lightPos = float4( 20, 43, -24, 1 );

	[unroll]
	for (int j = 0; j < 4; j++)
	{
		GROUND_VS_OUTPUT v;

		v.clip_pos = mul(planeVerts[j], g_mWorldViewProjection);
		v.tex = 5000*planeVerts[j].xy;

		v.t = normalize(mul(float4(-1 ,0  ,0  ,0), g_mWorldView).xyz);
		v.b = normalize(mul(float4(0  ,1  ,0  ,0), g_mWorldView).xyz);
		v.n = normalize(mul(float4(0  ,0 ,-1  ,0), g_mWorldView).xyz);

		v.pos = mul( planeVerts[j], g_mWorldView ).xyz;
		v.light_pos = mul( lightPos, g_mView ).xyz;

	 	v.clip = dot( mul( planeVerts[j], g_mWorld ), g_ClipPlane0 );

		triStream.Append(v);
	}
	
	triStream.RestartStrip();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
struct EVE_PATH_OUTPUT
{
    float4 clip_pos       : SV_POSITION; // Output position
};

EVE_PATH_OUTPUT EVE_PATH_VS(uint VertexID : SV_VERTEXID)
{
	EVE_PATH_OUTPUT output;

	output.clip_pos = float4(0,0,0,1);

	return output;
}

void EVE_PATH_tri_stream_append(float4 value, inout TriangleStream<EVE_PATH_OUTPUT> triStream)
{
	EVE_PATH_OUTPUT output;
	output.clip_pos = value;
	triStream.Append(output);
} 

[maxvertexcount(200)]
void EVE_PATH_GS(point EVE_PATH_OUTPUT pnt[1], uint primID : SV_PrimitiveID,  inout TriangleStream<EVE_PATH_OUTPUT> triStream )
{
	EVE_PATH_tri_stream_append(mul(g_EvePathPivots[0],g_mWorldViewProjection), triStream);

	[unroll]
	for (int j = 1; j < int(g_EvePathCount); j++)
	{
		EVE_PATH_tri_stream_append(mul(g_EvePathPivots[j],g_mWorldViewProjection), triStream);
		EVE_PATH_tri_stream_append(mul(g_EvePathPivots[j-1],g_mWorldViewProjection), triStream);
		triStream.RestartStrip();

		EVE_PATH_tri_stream_append(mul(g_EvePathPivots[j],g_mWorldViewProjection), triStream);
	}	
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
struct BOX_VS_OUTPUT
{
	float3 normal         : TEXCOORD1;
	float3 pos            : TEXCOORD2;
	float3 light_pos      : TEXCOORD3;
	float2 face_uv        : TEXCOORD4;

	float3 lightTBN       : TEXCOORD5;
	float3 viewTBN        : TEXCOORD6;

	float2 face_size      : TEXCOORD7;

    float4 clip_pos       : SV_POSITION; // Output position
	float clip            : SV_ClipDistance0;
};

BOX_VS_OUTPUT BOX_VS(uint VertexID : SV_VERTEXID)
{
	BOX_VS_OUTPUT output;

	output.pos = float3(0,0,0);
	output.light_pos = float3(0,0,0);
    output.normal = float3(0,0,0);
	output.clip_pos = float4(0,0,0,1);
	
	output.face_uv = float2(0, 0);
	output.face_size = float2(0, 0);
	output.lightTBN = float3(0, 0, 0);
	output.viewTBN = float3(0, 0, 0);
	output.clip = 0;

    return output;	
}

static const float4 cubeVerts[8] = 
{
	float4(0, 0, 0, 1),// LB  0
	float4(0, 1, 0, 1), // LT  1
	float4(1, 0, 0, 1), // RB  2
	float4(1, 1, 0, 1),  // RT  3
	float4(0, 0, 1, 1), // LB  4
	float4(0, 1, 1, 1),  // LT  5
	float4(1, 0, 1, 1),  // RB  6
	float4(1, 1, 1, 1)    // RT  7
};

static const float2 cubeFaceUV[4] =
{
	float2(0, 0),//
	float2(0, 1),//
	float2(1, 0),//
	float2(1, 1),//
};

static const int cubeIndices[24] =
{
	0, 1, 2, 3, // front
	7, 6, 3, 2, // right
	7, 5, 6, 4, // back
	4, 0, 6, 2, // bottom
	1, 0, 5, 4, // left
	3, 1, 7, 5  // top
};

[maxvertexcount(36)]
void BOX_GS(point BOX_VS_OUTPUT pnt[1], uint primID : SV_PrimitiveID,  inout TriangleStream<BOX_VS_OUTPUT> triStream )
{
	BOX_VS_OUTPUT v[8];
	
	float4 lightPos = float4( 20, 43, -24, 1 );
	[unroll]
	for (int j = 0; j < 8; j++)
	{
		v[j].pos = mul( cubeVerts[j], g_mWorldView ).xyz;
		v[j].light_pos = mul( lightPos, g_mView ).xyz;
		v[j].clip_pos = mul(cubeVerts[j], g_mWorldViewProjection);
		v[j].clip = dot( mul( cubeVerts[j], g_mWorld ), g_ClipPlane0 );
	}
	
	[unroll]
	for (int i = 0; i < 6; i++)
	{
		int FaceIndex = i * 4;

		float3 tangent = cubeVerts[cubeIndices[FaceIndex + 2]].xyz - cubeVerts[cubeIndices[FaceIndex + 0]].xyz;
		float3 bitangent = cubeVerts[cubeIndices[FaceIndex + 0]].xyz - cubeVerts[cubeIndices[FaceIndex + 1]].xyz;

		float3 tangent_WS = mul(tangent, (float3x3)g_mWorld);
		float3 bitangent_WS = mul(bitangent, (float3x3)g_mWorld);

		float2 UVScale = float2(length(tangent_WS), length(bitangent_WS));

		float3 normalize_tangent_WS = normalize(tangent_WS);
		float3 normalize_bitangent_WS = normalize(bitangent_WS);
		float3 normalize_normal_WS = cross(normalize_tangent_WS, normalize_bitangent_WS);
 
 		float3x3 WorldTBN = {
			normalize_tangent_WS.x, normalize_tangent_WS.y, normalize_tangent_WS.z,
			normalize_bitangent_WS.x, normalize_bitangent_WS.y, normalize_bitangent_WS.z,
			normalize_normal_WS.x, normalize_normal_WS.y, normalize_normal_WS.z
 		};

		float3 normalize_normal_VS = normalize(mul(normalize_normal_WS, (float3x3)g_mView));

		[unroll]
 		for (int j = 0; j < 4; j++)
 		{
			int INDEX = cubeIndices[FaceIndex + j];
			float3 pos_WS = mul(cubeVerts[INDEX], g_mWorld).xyz;

			v[INDEX].normal = normalize_normal_VS;
			v[INDEX].face_uv = cubeFaceUV[j];
			v[INDEX].face_size = UVScale;
			v[INDEX].lightTBN = mul(WorldTBN, lightPos.xyz - pos_WS);
			v[INDEX].viewTBN = mul(WorldTBN, g_mInvView._m30_m31_m32 - pos_WS);
 		}

		triStream.Append(v[cubeIndices[FaceIndex + 0]]);
		triStream.Append(v[cubeIndices[FaceIndex + 1]]);
		triStream.Append(v[cubeIndices[FaceIndex + 2]]);
		triStream.Append(v[cubeIndices[FaceIndex + 3]]);
		triStream.RestartStrip();
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
struct POST_PROCCESS_OUTPUT
{
	float2 tex                    : TEXCOORD1;
	float4 screenAndProjection    : TEXCOORD2;
	float2 nearAndFarZ            : TEXCOORD3;
    float4 clip_pos               : SV_POSITION; // Output position
};

POST_PROCCESS_OUTPUT POST_PROCCESS_VS(uint VertexID : SV_VERTEXID)
{
	POST_PROCCESS_OUTPUT output;

	output.clip_pos = float4(0,0,0,1);

    return output;	
}

[maxvertexcount(4)]
void POST_PROCCESS_GS(point POST_PROCCESS_OUTPUT pnt[1], uint primID : SV_PrimitiveID,  inout TriangleStream<POST_PROCCESS_OUTPUT> triStream )
{
	POST_PROCCESS_OUTPUT v[4];
	[unroll]
	for (int j = 0; j < 4; j++)
	{
		POST_PROCCESS_OUTPUT v;
		v.tex = planeVerts[j].xy;
		v.screenAndProjection = g_vFrustumParams;
		v.nearAndFarZ = g_vFrustumNearFar.xy;
		v.clip_pos = float4( (planeVerts[j].x-0.5)*2.0, -(planeVerts[j].y-0.5)*2.0, 0, 1);
		triStream.Append(v);
	}
	
	triStream.RestartStrip();
}
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////

struct UNLIT_WHITE_INPUT
{
    float3 pos : SV_Position;
};
struct UNLIT_WHITE_OUTPUT
{
    float4 clip_pos       : SV_POSITION;
	float4 color          : TEXCOORD1;
    float clip            : SV_ClipDistance0;
};
UNLIT_WHITE_OUTPUT UNLIT_WHITE_VS( in UNLIT_WHITE_INPUT i )
{
    UNLIT_WHITE_OUTPUT output;

    output.clip_pos = mul( float4( i.pos, 1.0 ), g_mWorldViewProjection );
    
	output.clip = dot( mul( float4( i.pos, 1.0 ), g_mWorld ), g_ClipPlane0 );

	output.color = g_ClipPlane2;

    return output;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

BOX_VS_OUTPUT STD_LIT_VS( in EVE_INPUT i )
{
    BOX_VS_OUTPUT output;

	float4 lightPos = float4( 20, 43, -24, 1 );

	output.normal = mul( float4(i.Normal,0.0f), g_mWorldView ).xyz;
	output.pos = mul( float4(i.Pos, 1.0f), g_mWorldView ).xyz;
	output.light_pos = mul( lightPos, g_mView ).xyz;
    output.clip_pos = mul( float4( i.Pos, 1.0 ), g_mWorldViewProjection );

	output.clip = dot( mul( float4(i.Pos, 1.0f), g_mWorld ), g_ClipPlane0 );
	output.face_uv = float2(0, 0);
	output.face_size = float2(0, 0);
	output.lightTBN = float3(0, 0, 0);
	output.viewTBN = float3(0, 0, 0);

    return output;
};

BOX_VS_OUTPUT LAMBERT_VS(in EVE_INPUT i)
{
	BOX_VS_OUTPUT output;

	float4 lightPos = float4(20, 43, -24, 1);

	output.normal = mul(float4(i.Normal, 0.0f), g_mWorldView).xyz;
	output.pos = mul(float4(i.Pos, 1.0f), g_mWorldView).xyz;
	output.light_pos = mul(lightPos, g_mView).xyz;
	output.clip_pos = mul(float4(i.Pos, 1.0), g_mWorldViewProjection);

	output.clip = dot(mul(float4(i.Pos, 1.0f), g_mWorld), g_ClipPlane0);
	output.face_uv = i.TexCoord0;
	output.face_size = float2(0, 0);
	output.lightTBN = float3(0, 0, 0);
	output.viewTBN = float3(0, 0, 0);

	return output;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////

struct ClipPosNormalTex2d2xTex4dCamPosWorldPos
{
    float2 tex1           : TEXCOORD1;
    float4 tex2           : TEXCOORD2;
    float3 waterParams    : TEXCOORD3;
	float3 extArg    	  : TEXCOORD4;
    float4 clip_pos       : SV_POSITION;
    float clip            : SV_ClipDistance0;
};

ClipPosNormalTex2d2xTex4dCamPosWorldPos WATER_VS( in EVE_INPUT input )
{
    ClipPosNormalTex2d2xTex4dCamPosWorldPos output;

    output.waterParams = float3(g_vFrustumParams.x, g_vFrustumParams.y, g_Time.z);

	output.extArg = normalize(g_mInvView._m30_m31_m32 - mul(float4( input.Pos, 1.0 ), g_mWorld ).xyz);

    output.clip_pos = mul(  float4( input.Pos, 1.0 ), g_mWorldViewProjection );

    output.tex1 = input.Pos.xz;

    output.tex2 = mul( mul( float4( input.Pos, 1.0 ), g_mWorld), g_mObject1WorldViewProjection );

    output.clip = dot( mul( float4( input.Pos, 1.0 ), g_mWorld ), g_ClipPlane0 );

    return output;
}
