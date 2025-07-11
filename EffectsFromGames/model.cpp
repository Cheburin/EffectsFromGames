#include "main.h"
#include "AnimationImpl.h"
#include <fstream>

// #include <assimp/Importer.hpp>      // C++ importer interface
// #include <assimp/scene.h>           // Output data structure
// #include <assimp/postprocess.h>     // Post processing flags

#include <locale>
#include <codecvt>
#include <string>
#include <array>
#include <map>
#include <locale> 

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

extern World GWorld;
#undef min // use __min instead
#undef max // use __max instead
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SimpleMath::Matrix aiMatrixToSimpleMathMatrix(const aiMatrix4x4& aiMe){
	XMFLOAT4X4 output;

	output._11 = aiMe.a1;
	output._12 = aiMe.a2;
	output._13 = aiMe.a3;
	output._14 = aiMe.a4;

	output._21 = aiMe.b1;
	output._22 = aiMe.b2;
	output._23 = aiMe.b3;
	output._24 = aiMe.b4;

	output._31 = aiMe.c1;
	output._32 = aiMe.c2;
	output._33 = aiMe.c3;
	output._34 = aiMe.c4;

	output._41 = aiMe.d1;
	output._42 = aiMe.d2;
	output._43 = aiMe.d3;
	output._44 = aiMe.d4;

	SimpleMath::Matrix ret = output;
	
	return ret.Transpose();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
void CreateBuffer(_In_ ID3D11Device* device, T const& data, D3D11_BIND_FLAG bindFlags, _Outptr_ ID3D11Buffer** pBuffer)
{
	assert(pBuffer != 0);

	D3D11_BUFFER_DESC bufferDesc = { 0 };

	bufferDesc.ByteWidth = (UINT)data.size() * sizeof(T::value_type);
	bufferDesc.BindFlags = bindFlags;
	bufferDesc.Usage = D3D11_USAGE_DEFAULT;

	D3D11_SUBRESOURCE_DATA dataDesc = { 0 };

	dataDesc.pSysMem = data.data();

	device->CreateBuffer(&bufferDesc, &dataDesc, pBuffer);

	//SetDebugObjectName(*pBuffer, "DirectXTK:GeometricPrimitive");
}

D3D11_INPUT_ELEMENT_DESC CharacterInputElements[9] =
{
	{ "SV_Position", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "TANGENT", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "BLENDINDICES", 0, DXGI_FORMAT_R8G8B8A8_UINT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "BLENDWEIGHT", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "BLENDINDICES", 1, DXGI_FORMAT_R8G8B8A8_UINT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "BLENDWEIGHT", 1, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
};

DirectX::ModelMeshPart* CreateModelMeshPart(ID3D11Device* device, std::function<void(std::vector<VertexPositionNormalTangentColorTextureSkinning2> & _vertices, std::vector<UINT> & _indices)> createGeometry){
	
	std::vector<VertexPositionNormalTangentColorTextureSkinning2> vertices;
	std::vector<UINT> indices;

	createGeometry(vertices, indices);

	size_t nVerts = vertices.size();

	DirectX::ModelMeshPart* modelMeshPArt(new DirectX::ModelMeshPart());

	modelMeshPArt->indexCount = indices.size();
	modelMeshPArt->startIndex = 0;
	modelMeshPArt->vertexOffset = 0;
	modelMeshPArt->vertexStride = sizeof(VertexPositionNormalTangentColorTextureSkinning2);
	modelMeshPArt->primitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
	modelMeshPArt->indexFormat = DXGI_FORMAT_R32_UINT;
	modelMeshPArt->vbDecl = std::shared_ptr<std::vector<D3D11_INPUT_ELEMENT_DESC>>(
		new std::vector<D3D11_INPUT_ELEMENT_DESC>(
		&CharacterInputElements[0],
		&CharacterInputElements[sizeof(CharacterInputElements) / sizeof(CharacterInputElements[0])]
		)
		);

	CreateBuffer(device, vertices, D3D11_BIND_VERTEX_BUFFER, modelMeshPArt->vertexBuffer.ReleaseAndGetAddressOf());

	CreateBuffer(device, indices, D3D11_BIND_INDEX_BUFFER, modelMeshPArt->indexBuffer.ReleaseAndGetAddressOf());

	return modelMeshPArt;
}
template<class T> T * create_vector4(T * data, int length)
{
	static T vector[4] = { 0, 0, 0, 0 };

	int i = 0;
	for (; i < length; i++){
		vector[i] = data[i];
	}
	for (; i < 4; i++){
		vector[i] = 0;
	}

	return vector;
}
template<class T> typename T::first_type * extract_first_from_pairs(T * data, int length)
{
	static T::first_type vector[8];

	for (int i = 0; i < length; i++){
		vector[i] = data[i].first;
	}

	return vector;
}
template<class T> typename T::second_type * extract_second_from_pairs(T * data, int length)
{
	static T::second_type vector[8];

	for (int i = 0; i < length; i++){
		vector[i] = data[i].second;
	}

	return vector;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct TransformationFrame
{
	TransformationFrame() :NextSibling(), FirstChild(), Parent(){}
	std::string Name;
	SimpleMath::Matrix Transformation;
	TransformationFrame * NextSibling;
	TransformationFrame * FirstChild;
	TransformationFrame * Parent;

	std::vector< DirectX::ModelMeshPart* > Meshes;
	std::vector< std::vector<VertexPositionNormalTangentColorTextureSkinning2> > MeshesVertex;
	std::vector< std::vector<UINT> > MeshesIndices;
};

struct CharacterSkelet
{
	CharacterSkelet() :TotalMatrix(), ReservMatrix(){};
	std::map<std::string, unsigned int> FramesNamesIndex;
	unsigned int TotalMatrix;
	unsigned int ReservMatrix;
	std::vector<SimpleMath::Matrix*> Transformation;
	std::vector<SimpleMath::Matrix> BoneOffSetTransformation;
	unsigned int findFrame(const std::string& name)
	{
		auto Iter = FramesNamesIndex.find(name);
		if (FramesNamesIndex.end() != Iter)
		{
			return (*Iter).second;
		}
		else
		{
			unsigned int OldSize = TotalMatrix;
			unsigned int NewSize = ++TotalMatrix;
			FramesNamesIndex.insert(std::pair<std::string, unsigned int>(name, OldSize));
			if (ReservMatrix <= NewSize)
			{
				ReservMatrix += 1024;
				Transformation.reserve(ReservMatrix);
				BoneOffSetTransformation.reserve(ReservMatrix);
			}
			Transformation.resize(NewSize);
			BoneOffSetTransformation.resize(NewSize);
			return OldSize;
		}
	}
};

Animation::~Animation()
{

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
void collectFrames(const aiScene* scene, int level, aiNode * node)
{
	char buffer[1024];
	std::string L;
	for (int i = 0; i < level; i++)
		L.append("-");
	sprintf(buffer, "%s %s \n", L.c_str(), node->mName.C_Str());// , delta.y);

	OutputDebugStringA(buffer);

	for (int i = 0; i < node->mNumChildren; i++)
	{
		collectFrames(scene, level + 1, node->mChildren[i]);
	}
}
*/

void collectFrames(ID3D11Device* device, CharacterSkelet * characterSkelet,  const aiScene* scene, int level, aiNode * node, TransformationFrame * parentFrame)
{
	TransformationFrame * LastChildren = 0;
	for (int i = 0; i < node->mNumChildren; i++)
	{
		//get Childeren node 
		auto ChildrenNode = node->mChildren[i];

		// new Frame
		auto Frame = new TransformationFrame();
		Frame->Transformation = aiMatrixToSimpleMathMatrix(ChildrenNode->mTransformation);
		Frame->Name = ChildrenNode->mName.C_Str();

		//index node matrix
		if (characterSkelet)
			characterSkelet->Transformation[characterSkelet->findFrame(Frame->Name)] = &(Frame->Transformation);

		//grab asimp mesh
		for (int j = 0; j < ChildrenNode->mNumMeshes; j++){
			auto Mesh = scene->mMeshes[ChildrenNode->mMeshes[j]];

			if (Mesh->GetNumUVChannels() == 0)
				throw "";

			if (Mesh->mNumUVComponents[0] != 2)
				throw "";

			if (!Mesh->mNormals)
				throw "";

			//if (!Mesh->mTangents)
			//	throw "";

			//if (!Mesh->mBitangents)
			//	throw "";

			if (characterSkelet && !Mesh->mNumBones)
				throw "";

			if (!characterSkelet && Mesh->mNumBones)
				throw "";

			std::vector<VertexPositionNormalTangentColorTextureSkinning2> __vertices;
			std::vector<UINT> __indices;

			auto FrameMesh = CreateModelMeshPart(device, [characterSkelet, Mesh, &__vertices, &__indices](std::vector<VertexPositionNormalTangentColorTextureSkinning2> & _vertices, std::vector<UINT> & _indices){
				std::map<uint32_t, std::vector<std::pair<uint32_t, float> > > VerticesSkinInfo;

				//proccess bones
				for (int k = 0; k < Mesh->mNumBones; k++)
				{
					auto MeshBone = Mesh->mBones[k];

					//index Bone OffSet matrix( Offset Matrix )
					unsigned int mBoneId = characterSkelet->findFrame(MeshBone->mName.C_Str());
					characterSkelet->BoneOffSetTransformation[mBoneId] = aiMatrixToSimpleMathMatrix(MeshBone->mOffsetMatrix);

					for (int z = 0; z < MeshBone->mNumWeights; z++)
					{
						auto & MeshBoneVertexWeight = MeshBone->mWeights[z];

						//link bone to vertex
						auto pVertexSkinInfo = VerticesSkinInfo.find(MeshBoneVertexWeight.mVertexId);
						if (pVertexSkinInfo == VerticesSkinInfo.end())
						{
							std::vector<std::pair<uint32_t, float> > VertexSkinInfo;
							VertexSkinInfo.push_back(std::pair<uint32_t, float>(mBoneId, MeshBoneVertexWeight.mWeight));
							VerticesSkinInfo.insert(std::pair<uint32_t, std::vector<std::pair<uint32_t, float> > >(MeshBoneVertexWeight.mVertexId, VertexSkinInfo));
						}
						else
						{
							(*pVertexSkinInfo).second.push_back(std::pair<uint32_t, float>(mBoneId, MeshBoneVertexWeight.mWeight));
						}

					}
				}

				//proccess vertex
				for (int k = 0; k < Mesh->mNumVertices; k++){
					std::vector<std::pair<uint32_t, float> > * VertexSkinInfo = 0;

					if (Mesh->mNumBones)
					{
						auto pVertexSkinInfo = VerticesSkinInfo.find(k);

						if (pVertexSkinInfo == VerticesSkinInfo.end())
							throw "";

						VertexSkinInfo = &(*pVertexSkinInfo).second;

						if (VertexSkinInfo->size() > 8)
							throw "";
					}

					auto & MeshVertex = Mesh->mVertices[k];
					auto & MeshUV = Mesh->mTextureCoords[0][k];
					auto & MeshTangent = Mesh->mTangents ? Mesh->mTangents[k] : aiVector3D();
					auto & MeshBitangent = Mesh->mBitangents ? Mesh->mBitangents[k] : aiVector3D();
					auto & MeshNormal = Mesh->mNormals[k];

					float _tangent[] = { MeshTangent.x, MeshTangent.y, MeshTangent.z, .0f };
					float _bitangent[] = { MeshBitangent.x, MeshBitangent.y, MeshBitangent.z, .0f };
					float _normal[] = { MeshNormal.x, MeshNormal.y, MeshNormal.z, .0f };

					VertexPositionNormalTangentColorTextureSkinning2 v;

					if (VertexSkinInfo)
					{
						uint32_t VertexSkinInfoSize = VertexSkinInfo->size();
						uint32_t * MeshIndices = extract_first_from_pairs(VertexSkinInfo->data(), VertexSkinInfoSize);;
						float * MeshWeights = extract_second_from_pairs(VertexSkinInfo->data(), VertexSkinInfoSize);;

						v.SetBlendIndices(XMUINT4(create_vector4(MeshIndices, VertexSkinInfoSize)));
						v.SetBlendWeights(SimpleMath::Vector4(create_vector4(MeshWeights, VertexSkinInfoSize)));
						if (VertexSkinInfoSize > 4)
						{
							v.SetBlendIndices2(XMUINT4(create_vector4(MeshIndices + 4, VertexSkinInfoSize - 4)));
							v.SetBlendWeights2(SimpleMath::Vector4(create_vector4(MeshWeights + 4, VertexSkinInfoSize - 4)));
						}
					}

					v.textureCoordinate = SimpleMath::Vector2(MeshUV.x, MeshUV.y);
					v.position = SimpleMath::Vector3(MeshVertex.x, MeshVertex.y, MeshVertex.z);
					auto normal = SimpleMath::Vector3(_normal);
					auto tangent = SimpleMath::Vector3(_tangent);
					auto bitangent = SimpleMath::Vector3(_bitangent);
					v.normal = normal;
					v.tangent = SimpleMath::Vector4(_tangent);
					v.tangent.w = (normal.Cross(tangent).Dot(bitangent) < 0.0F) ? -1.0F : 1.0F;
					///

					_vertices.push_back(v);
				}

				//proccess faces
				for (int k = 0; k < Mesh->mNumFaces; k++){
					auto & mFace = Mesh->mFaces[k];

					if (mFace.mNumIndices != 3)
					{
						throw "";
					}

					for (int z = 0; z < 3; z++){
						_indices.push_back(mFace.mIndices[z]);
					};
				}

				__vertices = _vertices;
				__indices = _indices;
			});

			Frame->Meshes.push_back(FrameMesh);
			Frame->MeshesVertex.push_back(__vertices);
			Frame->MeshesIndices.push_back(__indices);
		}

		// attach frame to herachy
		if (!LastChildren)
		{
			Frame->Parent = parentFrame;
			parentFrame->FirstChild = Frame;
		}
		else
		{
			Frame->Parent = LastChildren->Parent;
			LastChildren->NextSibling = Frame;
		}
		LastChildren = Frame;

		// deeper and deeper
		collectFrames(device, characterSkelet, scene, level + 1, ChildrenNode, Frame);
	}
}

void disposeFrames(TransformationFrame * Frame)
{
	if (Frame->FirstChild)
		disposeFrames(Frame->FirstChild);
	if (Frame->NextSibling)
		disposeFrames(Frame->NextSibling);

	for (int i = 0; i < Frame->Meshes.size(); i++)
	{
		delete Frame->Meshes[i];
	};

	delete Frame;
}

void calculateFramesTransformations(TransformationFrame * Frame, SimpleMath::Matrix ParentTransformation)
{
	Frame->Transformation = Frame->Transformation * ParentTransformation;
	if (Frame->FirstChild)
		calculateFramesTransformations(Frame->FirstChild, Frame->Transformation);
	if (Frame->NextSibling)
		calculateFramesTransformations(Frame->NextSibling, ParentTransformation);
}

void drawFrames(TransformationFrame * Frame, ID3D11DeviceContext* context, IEffect* effect, ID3D11InputLayout* inputLayout, std::function<void __cdecl()> setCustomState)
{
	if (Frame->FirstChild)
		drawFrames(Frame->FirstChild, context, effect, inputLayout, setCustomState);
	if (Frame->NextSibling)
		drawFrames(Frame->NextSibling, context, effect, inputLayout, setCustomState);

	for (int i = 0; i < Frame->Meshes.size(); i++)
	{
		Frame->Meshes[i]->Draw(context, effect, inputLayout, setCustomState);
	}
}

SimpleMath::Matrix* calculateAnimationPalite(CharacterSkelet * skelet)
{
	static SimpleMath::Matrix palite[1024];
	for (int i = 0; i < skelet->Transformation.size(); i++)
	{
		palite[i] = skelet->BoneOffSetTransformation[i] * (*(skelet->Transformation[i]));
	}
	return palite;
}

SimpleMath::Matrix* GetSkeletonMatrix(CharacterSkelet * skelet, int index)
{
	return skelet->Transformation[index];
}

TransformationFrame* findTransformationFrameByName(TransformationFrame * Frame, char * JointName)
{
	TransformationFrame* found = 0;
	if (std::string(Frame->Name) == JointName)
		found = Frame;
	if (!found && Frame->FirstChild)
		found = findTransformationFrameByName(Frame->FirstChild, JointName);
	if (!found && Frame->NextSibling)
		found = findTransformationFrameByName(Frame->NextSibling, JointName);
	return found;
}

void _getJointFrame(TransformationFrame * Frame, TransformationFrame * ParentFrame, std::function<void __cdecl(SimpleMath::Matrix M)> OnFrame)
{
	if (ParentFrame)
	{
		auto end = Frame->Transformation.Translation();
		auto begin = ParentFrame->Transformation.Translation();
		
		auto Forward = end - begin;
		auto ForwardLength = Forward.Length();
		auto Origin = 0.5f*(end + begin);
		static float D = 0.2f*ForwardLength;

		SimpleMath::Vector3 X(1, 0, 0);
		auto Y = Forward;
		Y.Normalize();
		if (1.0f - abs(X.Dot(Y)) < 0.001f)
		{
			X = SimpleMath::Vector3(0, 1, 0);
		}
		auto Z = X.Cross(Y);
		X = Y.Cross(Z);
		X.Normalize();
		Y.Normalize();
		Z.Normalize();

		OnFrame(SimpleMath::Matrix(
			vec4(D * X, 0.0f),
			vec4(ForwardLength * Y, 0.0f),
			vec4(D * Z, 0.0f),
			vec4(Origin, 1.0f)
		));
	}
	if (Frame->FirstChild)
		_getJointFrame(Frame->FirstChild, Frame, OnFrame);
	if (Frame->NextSibling)
		_getJointFrame(Frame->NextSibling, ParentFrame, OnFrame);
}

void getJointFrame(Character * skelet, char * JointName, std::function<void __cdecl(SimpleMath::Matrix M)> OnFrame)
{
	auto f = findTransformationFrameByName(skelet->frame, JointName);
	if (f)
	{
		_getJointFrame(f, 0, OnFrame);
	}
}

bool _IsJointOrNestedJointsIntoBox(TransformationFrame * Frame, const SimpleMath::Matrix& FromModelSpaceToWorld, int deep, const Box & box)
{
	auto Origin = SimpleMath::Vector3::Transform(Frame->Transformation.Translation(), FromModelSpaceToWorld);

	bool found = box.Containe(Origin);

	deep += deep > 0 ? -1 : 0;

	if (deep)
	{
		if (!found && Frame->FirstChild)
			found = _IsJointOrNestedJointsIntoBox(Frame->FirstChild, FromModelSpaceToWorld, deep, box);
	}

	if (!found && Frame->NextSibling)
		found = _IsJointOrNestedJointsIntoBox(Frame->NextSibling, FromModelSpaceToWorld, deep, box);

	return found;
}

bool IsJointOrNestedJointsIntoBox(Character * skelet, const SimpleMath::Matrix& FromModelSpaceToWorld, char * JointName, int deep, const Box& box)
{
	auto f = findTransformationFrameByName(skelet->frame, JointName);
	if (f)
	{
		return _IsJointOrNestedJointsIntoBox(f, FromModelSpaceToWorld, deep, box);
	}
	return false;
}

void _getChain(char * JointName, TransformationFrame * Frame, std::map<std::string, unsigned int> & FramesNamesIndex, std::vector<int> & indexes)
{
	indexes.push_back(FramesNamesIndex[std::string(Frame->Name)]);

	if (std::string(JointName) != Frame->Name && Frame->Parent)
	{
		_getChain(JointName, Frame->Parent, FramesNamesIndex, indexes);
	}
}

void _getChain(TransformationFrame * Frame, std::map<std::string, unsigned int> & FramesNamesIndex, std::vector<int> & indexes)
{
	indexes.push_back(FramesNamesIndex[std::string(Frame->Name)]);

	if (Frame->FirstChild)
	{
		_getChain(Frame->FirstChild, FramesNamesIndex, indexes);
	}
}

void GetChain(Character * character, char * JointName0, std::vector<int>& indexes)
{
	_getChain(findTransformationFrameByName(character->frame, JointName0), character->skelet->FramesNamesIndex, indexes);
}

void GetChain(Character * character, char * JointName0, char * JointName1, std::vector<int>& indexes)
{
	_getChain(JointName0, findTransformationFrameByName(character->frame, JointName1), character->skelet->FramesNamesIndex, indexes);

	std::reverse(indexes.begin(), indexes.end());
}

SimpleMath::Vector3 GetTranslationFromTransformationFrame(TransformationFrame * Frame)
{
	return Frame->Transformation.Translation();
}
SimpleMath::Vector3 GetCharacterJointTranslation(CharacterSkelet * characterSkelet, int Index)
{
	return characterSkelet->Transformation[Index]->Translation();
}

SimpleMath::Vector3 GetHandLocation(const SimpleMath::Matrix& FromModelSpaceToWorld, char* HandName)
{
	extern Character* Eve;
	return SimpleMath::Vector3::Transform(GetTranslationFromTransformationFrame(findTransformationFrameByName(Eve->frame, HandName)), FromModelSpaceToWorld);
};

SimpleMath::Vector3 GetHeadLocation(const SimpleMath::Matrix& FromModelSpaceToWorld)
{
	extern Character* Eve;
	const auto HeadEndJointLocation = SimpleMath::Vector3::Transform(GetCharacterJointTranslation(Eve->skelet, 58), FromModelSpaceToWorld);
	return HeadEndJointLocation;
}

void UpdateEveCapsuleHeight(const SimpleMath::Matrix& FromModelSpaceToWorld)
{
	extern World GWorld;
	extern Character* Eve;
	GWorld.Capsules["eve"].ab = (GetHeadLocation(FromModelSpaceToWorld) - GWorld.Capsules["eve"].origin).y - 2.0f*GWorld.Capsules["eve"].r;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Animation* makeAnimationTransition(Animation* _animation1, Animation* _animation2, double local_duration, std::function<double __cdecl(double, double)> _BlendFunction, std::function<std::pair<SimpleMath::Vector3, SimpleMath::Vector3> __cdecl(double, Animation*, Animation*)> _AdvanseFunction);
Animation* loadAnimation(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex, char * replace = nullptr);
void extractAnimationMeta(Animation * anim, bool extractHeight, double duration, std::function<SimpleMath::Matrix * __cdecl(unsigned int index)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTransformations);

Character::~Character(){
	delete skelet;
	disposeFrames(frame);
};

StaticObject::~StaticObject(){
	disposeFrames(frame);
}

void _getAllTriangles(TransformationFrame * Frame, std::vector<SimpleMath::Vector3> & result, const SimpleMath::Matrix & M)
{
	if (Frame->FirstChild)
		_getAllTriangles(Frame->FirstChild, result, M);
	if (Frame->NextSibling)
		_getAllTriangles(Frame->NextSibling, result, M);

	for (int i = 0; i < Frame->Meshes.size(); i++)
	{
		for (int j = 0; j < Frame->MeshesIndices[i].size(); j++)
		{
			//assume that frame dont have transform
			auto pos = Frame->MeshesVertex[i][Frame->MeshesIndices[i][j]].position;
			result.push_back(SimpleMath::Vector3::Transform(pos, M));
		}
	}
}

void _getAllBoxes(TransformationFrame * Frame, std::vector< std::string > & result, const SimpleMath::Matrix & M, const std::string& SpaceName, SimpleMath::Vector3& BoundBoxMin, SimpleMath::Vector3& BoundBoxMax)
{
	if (Frame->FirstChild)
		_getAllBoxes(Frame->FirstChild, result, M, SpaceName, BoundBoxMin, BoundBoxMax);
	if (Frame->NextSibling)
		_getAllBoxes(Frame->NextSibling, result, M, SpaceName, BoundBoxMin, BoundBoxMax);

	SimpleMath::Vector3 s, t;
	SimpleMath::Quaternion o;
	auto FM = Frame->Transformation * M;
	auto bDecompose = SimpleMath::Matrix(FM).Decompose(s, o, t);

	for (int i = 0; i < Frame->Meshes.size(); i++)
	{
		SimpleMath::Vector3 HalfSize = SimpleMath::Vector3(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

		for (int j = 0; j < Frame->MeshesIndices[i].size(); j++)
		{
			auto p = Frame->MeshesVertex[i][Frame->MeshesIndices[i][j]].position;
			HalfSize.x = __max(HalfSize.x, p.x);
			HalfSize.y = __max(HalfSize.y, p.y);
			HalfSize.z = __max(HalfSize.z, p.z);
		}

		if (Frame->Name == "Cube_003") break;

		std::string n = SpaceName + "_" + Frame->Name;

		Box b;

		b.origin = t;
		b.overlap = true;
		b.orientation = o;
		b.size = 2.0f*s*HalfSize;
		b.evalWorldSize();

		sprintf(DebugBuffer, "_getAllBoxes=(%f, %f, %f)(%f, %f, %f)(D=%f)(D=%f)(b=%d)\n", b.worldBackSide.x, b.worldBackSide.y, b.worldBackSide.z, b.size.x, b.size.y, b.size.z, FM.Determinant(), Frame->Transformation.Determinant(), bDecompose); Debug();

		SimpleMath::Matrix worldTransform = SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) * SimpleMath::Matrix::CreateScale(b.size) * b.getMatrix();

		//compute all ledges bound box
		{
			SimpleMath::Vector3 BoxCorners[2] = { SimpleMath::Vector3::Transform(SimpleMath::Vector3(0, 0, 0), worldTransform), SimpleMath::Vector3::Transform(SimpleMath::Vector3(1, 1, 1), worldTransform) };
			for (int i = 0; i < 2; i++)
			{
				BoundBoxMin.x = __min(BoundBoxMin.x, BoxCorners[i].x);
				BoundBoxMin.y = __min(BoundBoxMin.y, BoxCorners[i].y);
				BoundBoxMin.z = __min(BoundBoxMin.z, BoxCorners[i].z);
				BoundBoxMax.x = __max(BoundBoxMax.x, BoxCorners[i].x);
				BoundBoxMax.y = __max(BoundBoxMax.y, BoxCorners[i].y);
				BoundBoxMax.z = __max(BoundBoxMax.z, BoxCorners[i].z);
			}
		}

		GWorld.Ledges[n] = Ledge::Make(b);

		result.push_back(n);

		//assume that frame have one mesh

		break;
	}
}

void StaticObject::getAllTriangles(std::vector<SimpleMath::Vector3> &vertex, const SimpleMath::Matrix & M)
{
	_getAllTriangles(frame, vertex, M);
}

std::vector< std::string > StaticObject::EmitLedges(const std::string& SpaceName, SimpleMath::Vector3& BoundBoxMin, SimpleMath::Vector3& BoundBoxMax)
{
	std::vector< std::string > Result;

	BoundBoxMin = SimpleMath::Vector3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	BoundBoxMax = SimpleMath::Vector3(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

	_getAllBoxes(frame, Result, SimpleMath::Matrix::Identity, SpaceName, BoundBoxMin, BoundBoxMax);

	return Result;
}

StaticObject* loadStaticObject(ID3D11Device* device, char * file_name)
{
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFile(file_name, aiProcess_ConvertToLeftHanded | aiProcess_CalcTangentSpace);// | aiProcess_Triangulate | aiProcess_FlipUVs );// | aiProcess_Triangulate);// | aiProcess_FlipUVs | aiProcess_GenNormals | aiProcess_CalcTangentSpace);// | aiProcess_MakeLeftHanded);

	auto RootFrame = new TransformationFrame();
	RootFrame->Transformation = aiMatrixToSimpleMathMatrix(scene->mRootNode->mTransformation);
	RootFrame->Name = scene->mRootNode->mName.C_Str();

	collectFrames(device, nullptr, scene, 0, scene->mRootNode, RootFrame);

	auto staticObject = new StaticObject();
	staticObject->frame = RootFrame;

	return staticObject;
}

StaticObject* loadStaticObject2(ID3D11Device* device, char * file_name)
{
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFile(file_name, aiProcess_ConvertToLeftHanded);// | aiProcess_Triangulate | aiProcess_FlipUVs );// | aiProcess_Triangulate);// | aiProcess_FlipUVs | aiProcess_GenNormals | aiProcess_CalcTangentSpace);// | aiProcess_MakeLeftHanded);

	auto RootFrame = new TransformationFrame();
	RootFrame->Transformation = aiMatrixToSimpleMathMatrix(scene->mRootNode->mTransformation);
	RootFrame->Name = scene->mRootNode->mName.C_Str();

	collectFrames(device, nullptr, scene, 0, scene->mRootNode, RootFrame);

	auto staticObject = new StaticObject();
	staticObject->frame = RootFrame;

	return staticObject;
}

Character* loadCharacter(ID3D11Device* device, char * file_name)
{
	Assimp::Importer importer;

	// And have it read the given file with some example postprocessing
	// Usually - if speed is not the most important aspect for you - you'll 
	// propably to request more postprocessing than we do in this example.
	//aiProcess_Triangulate
	const aiScene* scene = importer.ReadFile(file_name, aiProcess_ConvertToLeftHanded | aiProcess_CalcTangentSpace);// | aiProcess_Triangulate | aiProcess_FlipUVs );// | aiProcess_Triangulate);// | aiProcess_FlipUVs | aiProcess_GenNormals | aiProcess_CalcTangentSpace);// | aiProcess_MakeLeftHanded);

	auto RootFrame = new TransformationFrame();
	RootFrame->Transformation = aiMatrixToSimpleMathMatrix(scene->mRootNode->mTransformation);
	RootFrame->Name = scene->mRootNode->mName.C_Str();

	auto characterSkelet = new CharacterSkelet();
	//index node matrix
	unsigned int matrixIndex = characterSkelet->findFrame(RootFrame->Name);
	characterSkelet->Transformation[matrixIndex] = &(RootFrame->Transformation);

	collectFrames(device, characterSkelet, scene, 0, scene->mRootNode, RootFrame);

	auto ret = new Character();

	ret->frame = RootFrame;
	ret->skelet = characterSkelet;

	return ret;
}

extern World GWorld;
extern SimpleMath::Vector3 gravitation;
extern SimpleMath::Vector2 input_move;
extern bool input_jump;
extern bool state_jump;
extern bool state_jump_from_wall;
extern bool state_hanging;
extern bool state_climbing;
extern bool state_falling;
extern bool state_into_water;
extern bool state_take_it;
extern bool state_ballistic_fly_to_target;
extern bool state_kneeling;
extern bool state_falling_and_hang_on;

extern bool state_idle;
extern bool state_walking_on_your_feet;

extern bool state_play_debug_animation;

//extern bool state_ready_for_moving_from_falling_and_hang_on;
extern SimpleMath::Vector3 state_hanging_Hand_Location;
extern char* state_hanging_Hand_Name;
//extern bool state_prevent_hanging;

extern bool state_BallisticFly_to_HangingIdle;
extern bool state_BallisticFly_to_HangingIdleWithOutLeg;

extern SimpleMath::Vector3 state_hanging_Toe_Location;
extern bool state_hanging_Toe_activated;
extern char* state_hanging_Toe_Name;

struct ConsumeInputJump
{
	bool operator()(){
		auto prev = input_jump;
		input_jump = false;
		return prev;
	}
};
ConsumeInputJump consumeInputJump;

Animation * createFallingAndHangOnAnimation(std::map<std::string, unsigned int> & FramesNamesIndex);

Animation * createShimmyAnimation(bool MoveRight, std::map<std::string, unsigned int> & FramesNamesIndex, std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans);

Animation * CreateBallisticFlyAnimation(std::map<std::string, unsigned int> & FramesNamesIndex, std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans);

Animation * CreateJumpFromWallAnimation(std::map<std::string, unsigned int> & FramesNamesIndex, std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans);

Animation * CreateEdgeHorizontalJumpAnimation(bool IsLeft, Animation* Pose, SimpleMath::Quaternion PoseYaw, std::map<std::string, unsigned int> & FramesNamesIndex, std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans);

Animation* CreateBracedHangHopUpAnimation(std::map<std::string, unsigned int> & FramesNamesIndex, std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans);

extern char DebugBuffer[1024];
extern World GWorld;
void Debug();

SimpleMath::Vector3 ShimmyAnimationToSegmentBasis(AnimationBase *animation, const SimpleMath::Vector3& Location);
int GetShimmyAnimationKind(AnimationBase *animation);

std::string start_hanging_Ledge_Name;
int start_hanging_Ledge_BoxIndex;

extern std::string state_hanging_Ledge_Name;
extern int state_hanging_Ledge_BoxIndex;
extern bool state_movement_is_obstructed;
extern Box state_hanging_Ledge_Box;
extern SimpleMath::Vector3 cameraForward;

extern IClimbingPathHelper* ClimbingPathHelper;

Animation* loadAnimationFromUnreal(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex);
std::vector<JointSQT>& __AnimGetJointsByTime(AnimationBase* Anim, float Time);
SimpleMath::Quaternion __AnimSubstructRootDeltaRotation(AnimationBase* Anim);

AnimationBase* DebugAnimation;

Animation* loadAnimationFromUnreal(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex);
Animation* loadAnimationFromBlender(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex);
void AnimationSetJointT(Animation * anim, int JointNum, SimpleMath::Vector3 Translation);
int IsShimmyAnimationEdgeBreakOff(AnimationBase *animation);
void FixAnimJointsOrientation(Animation* Anim, Animation* RefAnim);
void rotateHips(Animation *, SimpleMath::Quaternion);

void BracedHangHopUpAnimation_SetInitialHandsLocations(Animation* __Animation);

AnimationBase** PackGroup(AnimationBase * a1, AnimationBase * a2)
{
	static int CicleIndex = 0;
	static AnimationBase* ar[10][10];
	ar[CicleIndex % 10][0] = a1; ar[CicleIndex % 10][1] = a2;
	return &ar[CicleIndex++ % 10][0];
};

void loadAnimations(IAnimationGraph2 * Graph, CharacterSkelet * characterSkelet, TransformationFrame * frame)
{
	auto EveInvertModelTransform = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]).Invert();
	auto gravityInEveSystemCoordinates = SimpleMath::Vector3::Transform(gravitation, EveInvertModelTransform);
	gravityInEveSystemCoordinates = gravitation;

	auto getSkeletMatrix = [characterSkelet](unsigned int index){return characterSkelet->Transformation[index]; };
	auto calculateFramesTrans = [frame](){calculateFramesTransformations(frame, SimpleMath::Matrix::Identity); };

	auto TPose = loadAnimationFromBlender("Media\\Poses\\TPose.dae", characterSkelet->FramesNamesIndex);
	extractAnimationMeta(TPose, false, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("TPose", TPose);

	auto walkingAnimation = loadAnimation("Media\\Animations\\Walking.dae", characterSkelet->FramesNamesIndex);
	walkingAnimation->setRate(60 * .01 + 1.0 / 2.0);
	extractAnimationMeta(walkingAnimation, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("walking", walkingAnimation);

	auto idleAnimation = loadAnimation("Media\\Animations\\Idle.dae", characterSkelet->FramesNamesIndex);
	idleAnimation->setRate(1.0 / 16.0);
	extractAnimationMeta(idleAnimation, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("idle", idleAnimation);

	auto jumpForwardAnimation = loadAnimation("Media\\Animations\\JumpForward.dae", characterSkelet->FramesNamesIndex);
	jumpForwardAnimation->setRate(1.0 / 2.0);
	jumpForwardAnimation->setLooping(false);
	//jumpForwardAnimation->AddOffset(-gravityInEveSystemCoordinates);
	extractAnimationMeta(jumpForwardAnimation, true, .7f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("jump_forward", jumpForwardAnimation);

	auto jumpUpAnimation = loadAnimation("Media\\Animations\\JumpUp.dae", characterSkelet->FramesNamesIndex);
	jumpUpAnimation->setRate(1.0 / 2.0f);// 2.0f);
	jumpUpAnimation->setLooping(false);
	//jumpUpAnimation->AddOffset(-gravityInEveSystemCoordinates);
	extractAnimationMeta(jumpUpAnimation, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	jumpUpAnimation->TransformMetaSamples(
		0,
		[](SimpleMath::Vector4 v){
		v = SimpleMath::Vector4(v.x, 3.5f*v.y, v.z, 1.0f);
		return v;
	});
	Graph->registerAnimation("jump_up", jumpUpAnimation);

	auto Climb = loadAnimation("Media\\Animations\\Climbing.dae", characterSkelet->FramesNamesIndex);
	Climb->setRate(1.0 / 4.0);
	Climb->setLooping(false);
	//Climb->AddOffset(-gravityInEveSystemCoordinates);
	extractAnimationMeta(Climb, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("climbing", Climb);

	auto bracedHangHopUpAnimation = CreateBracedHangHopUpAnimation(characterSkelet->FramesNamesIndex, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("braced_hang_hop_up", bracedHangHopUpAnimation);

	auto fallingIdleAnimation = loadAnimation("Media\\Animations\\FallingIdle.dae", characterSkelet->FramesNamesIndex);
	fallingIdleAnimation->setRate(1.0 / 2.0f);
	extractAnimationMeta(fallingIdleAnimation, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("falling_idle", fallingIdleAnimation);

	auto treadingWaterAnimation = loadAnimation("Media\\Animations\\TreadingWater.dae", characterSkelet->FramesNamesIndex);
	treadingWaterAnimation->setRate(1.0 / 4.5f);
	//treadingWaterAnimation->AddOffset(-gravityInEveSystemCoordinates);
	extractAnimationMeta(treadingWaterAnimation, false, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("treading_water", treadingWaterAnimation);

	auto swimmingAnimation = loadAnimation("Media\\Animations\\Swimming.dae", characterSkelet->FramesNamesIndex);
	swimmingAnimation->setRate(1.0 / 4.533f);
	//swimmingAnimation->AddOffset(-gravityInEveSystemCoordinates);
	swimmingAnimation->TransformJointSamples(
		64,
		"translation",
		[](SimpleMath::Vector4 v){
		v = SimpleMath::Vector4(v.x, v.y + 35.0f, v.z, 0);
		return v;
	});
	extractAnimationMeta(swimmingAnimation, false, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("swimming", swimmingAnimation);

	auto takeItAnimation = makeAnimationPose();
	Graph->registerAnimation("take_it_pose", takeItAnimation);

	auto FreehangClimb = loadAnimation("Media\\Animations\\FreehangClimb.dae", characterSkelet->FramesNamesIndex);
	FreehangClimb->setLooping(false);
	//FreehangClimb->AddOffset(-gravityInEveSystemCoordinates);
	FreehangClimb->setRate(1.0 / 5.0f);
	extractAnimationMeta(FreehangClimb, true, 1.f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("Freehang_Climb", FreehangClimb);

	auto Kneeling = loadAnimation("Media\\Animations\\Kneeling.dae", characterSkelet->FramesNamesIndex);
	extractAnimationMeta(Kneeling, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("Kneeling", Kneeling);

	auto CrouchedWalking = loadAnimation("Media\\Animations\\CrouchedWalking.dae", characterSkelet->FramesNamesIndex);
	extractAnimationMeta(CrouchedWalking, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("Crouched_Walking", CrouchedWalking);

	auto FallingAndHangOn = createFallingAndHangOnAnimation(characterSkelet->FramesNamesIndex);
	Graph->registerAnimation("FallingAndHangOn", FallingAndHangOn);

	auto RightShimmy = createShimmyAnimation(true, characterSkelet->FramesNamesIndex, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("RightShimmy", RightShimmy);

	auto LeftShimmy = createShimmyAnimation(false, characterSkelet->FramesNamesIndex, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("LeftShimmy", LeftShimmy);
	
	auto BallisticFly = CreateBallisticFlyAnimation(characterSkelet->FramesNamesIndex, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("BallisticFly", BallisticFly);

	auto Climb_Look_Idle_L = loadAnimationFromUnreal("Media\\Animations\\Edge\\Climb_Look_Idle_L.FBX", characterSkelet->FramesNamesIndex);
	extractAnimationMeta(Climb_Look_Idle_L, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	auto Climb_Look_Idle_L_Delta_Rotation = __AnimSubstructRootDeltaRotation(Climb_Look_Idle_L);
	Graph->registerAnimation("Climb_Look_Idle_L", Climb_Look_Idle_L);

	auto Climb_Look_Idle_R = loadAnimationFromUnreal("Media\\Animations\\Edge\\Climb_Look_Idle_R.FBX", characterSkelet->FramesNamesIndex);
	extractAnimationMeta(Climb_Look_Idle_R, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	auto Climb_Look_Idle_R_Delta_Rotation = __AnimSubstructRootDeltaRotation(Climb_Look_Idle_R);
	Graph->registerAnimation("Climb_Look_Idle_R", Climb_Look_Idle_R);

	auto JumpFromWall = CreateJumpFromWallAnimation(characterSkelet->FramesNamesIndex, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("JumpFromWall", JumpFromWall);
	//AnimationBase* ShimmyAnimationGetDebugAnimation(AnimationBase *animation);
	//auto DebugAnimation = ShimmyAnimationGetDebugAnimation(LeftShimmy);
	//Graph->registerAnimation("DebugAnimation", DebugAnimation);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	auto Hanging_Idle_WithOut_Leg = loadAnimation("Media\\Animations\\Edge\\HangingIdleWithOutLeg.dae", characterSkelet->FramesNamesIndex);
	extractAnimationMeta(Hanging_Idle_WithOut_Leg, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("Hanging_Idle_WithOut_Leg", Hanging_Idle_WithOut_Leg);

	auto Jump_To_Hang_WithOut_Leg = loadAnimation("Media\\Animations\\JumpToHangWithOutLeg.dae", characterSkelet->FramesNamesIndex);
	Jump_To_Hang_WithOut_Leg->setLooping(false);
	Jump_To_Hang_WithOut_Leg->setRate(2.0);
	extractAnimationMeta(Jump_To_Hang_WithOut_Leg, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("Jump_To_Hang_WithOut_Leg", Jump_To_Hang_WithOut_Leg);

	////
	auto Hanging_Idle_With_Leg = loadAnimation("Media\\Animations\\HangingIdleWithLeg.dae", characterSkelet->FramesNamesIndex);
	extractAnimationMeta(Hanging_Idle_With_Leg, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("Hanging_Idle_With_Leg", Hanging_Idle_With_Leg);

	/*����� ����� ���� ����, ������� �� ��� ��� �� ����� ������(��� �� ������� ���� � ��������� ������)*/
	auto Jump_To_Hang_With_Leg = makeAnimationPose();
	Jump_To_Hang_With_Leg->CurrentJoints = __AnimGetJointsByTime(Hanging_Idle_With_Leg, 0.f);
	Jump_To_Hang_With_Leg->CurrentMetaChannels = Hanging_Idle_With_Leg->CurrentMetaChannels;
	Graph->registerAnimation("Jump_To_Hang_With_Leg", Jump_To_Hang_With_Leg);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	auto ActionPose_Release_And_Go_Down = loadAnimationFromBlender("Media\\Animations\\ActionPose_Release_And_Go_Down.dae", characterSkelet->FramesNamesIndex);
	AnimationSetJointT(ActionPose_Release_And_Go_Down, 64, SimpleMath::Vector3(0, 80, 0));
	extractAnimationMeta(ActionPose_Release_And_Go_Down, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("ActionPose_Release_And_Go_Down", ActionPose_Release_And_Go_Down);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	auto Climb_Fold_Hands = loadAnimationFromBlender("Media\\Animations\\Edge\\Climb_Fold_Hands.dae", characterSkelet->FramesNamesIndex);
	AnimationSetJointT(Climb_Fold_Hands, 64, SimpleMath::Vector3(0, 24, 0));
	extractAnimationMeta(Climb_Fold_Hands, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("Climb_Fold_Hands", Climb_Fold_Hands);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	auto Left_Edge_Horizontal_Jump = CreateEdgeHorizontalJumpAnimation(true, Climb_Look_Idle_L, Climb_Look_Idle_L_Delta_Rotation, characterSkelet->FramesNamesIndex, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("Left_Edge_Horizontal_Jump", Left_Edge_Horizontal_Jump);
	auto Right_Edge_Horizontal_Jump = CreateEdgeHorizontalJumpAnimation(false, Climb_Look_Idle_R, Climb_Look_Idle_R_Delta_Rotation, characterSkelet->FramesNamesIndex, getSkeletMatrix, calculateFramesTrans);
	Graph->registerAnimation("Right_Edge_Horizontal_Jump", Right_Edge_Horizontal_Jump);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void CreateLadderAnimationAndRegisterIntoAnimationGraph(IAnimationGraph2 * Graph, CharacterSkelet * characterSkelet, TransformationFrame * frame);
	CreateLadderAnimationAndRegisterIntoAnimationGraph(Graph, characterSkelet, frame);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Climb_Look_Idle_R->subscribe("onPlayingChanged", [RightShimmy](bool state){
		if (state)
		{
			auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();

			ClimbingPathHelper->LedgeToSegments();

			state_hanging_Hand_Location = ClimbingPathHelper->ToSegmentBasis(GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = LeftHandName), ClimbingPathHelper->GetCurrentSegmentIndex());
		}
	});
	Climb_Look_Idle_L->subscribe("onPlayingChanged", [LeftShimmy](bool state){
		if (state)
		{
			auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();

			ClimbingPathHelper->LedgeToSegments();

			state_hanging_Hand_Location = ClimbingPathHelper->ToSegmentBasis(GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = RightHandName), ClimbingPathHelper->GetCurrentSegmentIndex());
		}
	});
	RightShimmy->subscribe("onPlayingChanged", [RightShimmy](bool state){
		if (state)
		{
			auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();

			state_hanging_Hand_Location = GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = LeftHandName);

			state_hanging_Hand_Location = ShimmyAnimationToSegmentBasis(RightShimmy, state_hanging_Hand_Location);
		}
	});
	LeftShimmy->subscribe("onPlayingChanged", [LeftShimmy](bool state){
		if (state)
		{
			auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();

			state_hanging_Hand_Location = GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = RightHandName);

			state_hanging_Hand_Location = ShimmyAnimationToSegmentBasis(LeftShimmy, state_hanging_Hand_Location);
		}
	});
	Hanging_Idle_WithOut_Leg->subscribe("onPlayingChanged", [Graph, LeftShimmy, RightShimmy](bool state){
		if (state)
		{
			auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();

			if (Graph->getPrevAnimationName() == "RightShimmy")
			{
				state_hanging_Hand_Location = GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = RightHandName);

				state_hanging_Hand_Location = ShimmyAnimationToSegmentBasis(RightShimmy, state_hanging_Hand_Location);
			}
			else if (Graph->getPrevAnimationName() == "LeftShimmy")
			{
				state_hanging_Hand_Location = GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = LeftHandName);

				state_hanging_Hand_Location = ShimmyAnimationToSegmentBasis(LeftShimmy, state_hanging_Hand_Location);
			}
		}
	});
	Hanging_Idle_With_Leg->subscribe("onPlayingChanged", [Graph, LeftShimmy, RightShimmy](bool state){
		if (state)
		{
			auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();

			if (Graph->getPrevAnimationName() == "RightShimmy")
			{
				state_hanging_Hand_Location = GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = RightHandName);

				state_hanging_Hand_Location = ShimmyAnimationToSegmentBasis(RightShimmy, state_hanging_Hand_Location);
			}
			else if (Graph->getPrevAnimationName() == "LeftShimmy")
			{
				state_hanging_Hand_Location = GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = LeftHandName);

				state_hanging_Hand_Location = ShimmyAnimationToSegmentBasis(LeftShimmy, state_hanging_Hand_Location);
			}
		}
	});
	JumpFromWall->subscribe("onPlayingChanged", [](bool state){
		state_jump_from_wall = state;
		if (state)
		{
			start_hanging_Ledge_Name = state_hanging_Ledge_Name;
			start_hanging_Ledge_BoxIndex = state_hanging_Ledge_BoxIndex;
			//state_hanging = false;
		}
	});
	jumpUpAnimation->subscribe("onPlayingChanged", [](bool state){
		state_jump = state;
		if (state)
		{
			//state_hanging = false;
			//state_prevent_hanging = true;
		}
	});
	jumpForwardAnimation->subscribe("onPlayingChanged", [](bool state){
		state_jump = state;
		if (state)
		{
			//state_hanging = false;
			//state_prevent_hanging = true;
		}
	});
	bracedHangHopUpAnimation->subscribe("onPlayingChanged", [](bool state){
		state_jump = state;
		if (state)
		{
			start_hanging_Ledge_Name = state_hanging_Ledge_Name;
			start_hanging_Ledge_BoxIndex = state_hanging_Ledge_BoxIndex;
			//state_hanging = false;
			//state_prevent_hanging = true;
		}
	});
	Climb->subscribe("onPlayingChanged", [](bool state){
		state_climbing = state;
		if (state)
		{
			//state_hanging = false;
			//state_prevent_hanging = true;
		}
	});
	FreehangClimb->subscribe("onPlayingChanged", [](bool state){
		state_climbing = state;
		if (state)
		{
			sprintf(DebugBuffer, "FreehangClimb->subscribe onPlayingChanged true\n"); Debug();
			//state_hanging = false;
			//state_prevent_hanging = true;
		}
	});

	//static bool Jump_To_Hang_With_Leg_Playing = false;
	//Jump_To_Hang_With_Leg->subscribe("onPlayingChanged", [](bool state){
	//	Jump_To_Hang_With_Leg_Playing = state;
	//});
	
	//static bool Jump_To_Hang_WithOut_Leg_Playing = false;
	//Jump_To_Hang_WithOut_Leg->subscribe("onPlayingChanged", [](bool state){
	//	Jump_To_Hang_WithOut_Leg_Playing = state;
	//});

	static bool ready_hanging_on_wall = false;
	static bool ready_for_moving_from_falling_and_hang_on = false;
	FallingAndHangOn->subscribe("onPlayingChanged", [](bool state){
		if ((int)state == 1)
		{
			ready_for_moving_from_falling_and_hang_on = true;
		}
		if ((int)state == 2)
		{
			ready_hanging_on_wall = true;
		}
		if ((int)state == 4)
		{
			state_falling = state_falling_and_hang_on = false;
			ready_hanging_on_wall = ready_for_moving_from_falling_and_hang_on = false;
		}
	});

	walkingAnimation->subscribe("onPlayingChanged", [](bool state){
		state_walking_on_your_feet = state;
	});
	CrouchedWalking->subscribe("onPlayingChanged", [](bool state){
		state_walking_on_your_feet = state;
	});
	idleAnimation->subscribe("onPlayingChanged", [](bool state){
		state_idle = state;
	});
	Kneeling->subscribe("onPlayingChanged", [](bool state){
		state_idle = state;
	});
	ActionPose_Release_And_Go_Down->subscribe("onPlayingChanged", [](bool state){
		if (state)
		{
			auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();

			state_falling = true;
			state_hanging = false;
			start_hanging_Ledge_Name = state_hanging_Ledge_Name;
			start_hanging_Ledge_BoxIndex = state_hanging_Ledge_BoxIndex;
			state_hanging_Hand_Location = GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = LeftHandName);
		}
	});
	//Graph->createLink(DebugAnimation)
	//	.setEndPoint(idleAnimation)
	//	.reverse([](){ return state_play_debug_animation; })
	//	.setEndPoint(fallingIdleAnimation)
	//	.reverse([](){ return state_play_debug_animation; })
	//;

	Graph->createLink(idleAnimation)
		.setEndPoint(CrouchedWalking)
		.reverse([](){ return input_move.Length() == 0.0f && !state_falling && !state_kneeling; })
		.setEndPoint(Kneeling)
		.reverse([](){ return input_move.Length() == 0.0f && !state_falling && !state_kneeling; })
		.setEndPoint(walkingAnimation)
		.reverse([](){ return input_move.Length() == 0.0f && !state_falling && !state_kneeling; })
		.setEndPoint(fallingIdleAnimation)
		.reverse([](){ return input_move.Length() == 0.0f && !state_falling && !state_kneeling; })
		.setEndPoint(FreehangClimb)
		.reverse([](){ return input_move.Length() == 0.0f && !state_falling && !state_climbing && !state_kneeling; })
		.setEndPoint(Climb)
		.reverse([](){ return input_move.Length() == 0.0f && !state_falling && !state_climbing && !state_kneeling; })
		.setEndPoint(jumpUpAnimation)
		.reverse([](){ return input_move.Length() == 0.0f && !state_falling && !state_jump && !state_kneeling; })
		.setEndPoint(jumpForwardAnimation)
		.reverse([](){ return input_move.Length() == 0.0f && !state_falling && !state_jump && !state_kneeling; })
		.setEndPoint(FallingAndHangOn)
		.reverse([](){ return input_move.Length() == 0.0f && !state_falling && !state_falling_and_hang_on; })
	;

	Graph->createLink(walkingAnimation)
		.setEndPoint(Kneeling)
		.reverse([](){ return input_move.Length() > 0.0f && !state_falling && !state_kneeling; })
		.setEndPoint(CrouchedWalking)
		.reverse([](){ return input_move.Length() > 0.0f && !state_falling && !state_kneeling; })
		.setEndPoint(idleAnimation)
		.reverse([](){ return input_move.Length() > 0.0f && !state_falling && !state_kneeling; })
		.setEndPoint(fallingIdleAnimation)
		.reverse([](){ return input_move.Length() > 0.0f && !state_falling && !state_kneeling; })
		.setEndPoint(FreehangClimb)
		.reverse([](){ return input_move.Length() > 0.0f && !state_falling && !state_climbing && !state_kneeling; })
		.setEndPoint(Climb)
		.reverse([](){ return input_move.Length() > 0.0f && !state_falling && !state_climbing && !state_kneeling; })
		.setEndPoint(jumpUpAnimation)
		.reverse([](){ return input_move.Length() > 0.0f && !state_falling && !state_jump && !state_kneeling; })
		.setEndPoint(jumpForwardAnimation)
		.reverse([](){ return input_move.Length() > 0.0f && !state_falling && !state_jump && !state_kneeling; })
		;

	Graph->createLink(fallingIdleAnimation)
		.setEndPoint(idleAnimation)
		.reverse([](){ return state_falling; })
		.setEndPoint(walkingAnimation)
		.reverse([](){ return state_falling; })
		.setEndPoint(jumpUpAnimation)
		.reverse([](){ return state_falling; })
		.setEndPoint(jumpForwardAnimation)
		.reverse([](){ return state_falling; })
	;

	Graph->createLink(jumpUpAnimation)
		.setEndPoint(idleAnimation)
		.reverse([](){ return consumeInputJump(); })
		;

	Graph->createLink(jumpForwardAnimation)
		.setEndPoint(walkingAnimation)
		.reverse([](){ return consumeInputJump(); })
		;

	Graph->createLink(bracedHangHopUpAnimation)
		.setEndPoint(Hanging_Idle_With_Leg)
		.reverse([bracedHangHopUpAnimation](){
			if (consumeInputJump())
			{
				BracedHangHopUpAnimation_SetInitialHandsLocations(bracedHangHopUpAnimation);
				return true;
			}
			return false;
		})
		;

	Graph->createLink(Kneeling)
		.setEndPoint(idleAnimation)
		.reverse([](){ return input_move.Length() == 0.0f && !state_falling && state_kneeling; })
		.setEndPoint(CrouchedWalking)
		.reverse([](){ return input_move.Length() == 0.0f && !state_falling && state_kneeling; })
		.setEndPoint(FreehangClimb)
		.reverse([](){ return input_move.Length() == 0.0f && !state_falling && state_kneeling; })
		;

	Graph->createLink(CrouchedWalking)
		.setEndPoint(Kneeling)
		.reverse([](){ return input_move.Length() > 0.0f && !state_falling && state_kneeling; })
		.setEndPoint(walkingAnimation)
		.reverse([](){ return input_move.Length() > 0.0f && !state_falling && state_kneeling; })
		.setEndPoint(FreehangClimb)
		.reverse([](){ return input_move.Length() > 0.0f && !state_falling && !state_climbing && state_kneeling; })
		;

	Graph->createLink(FallingAndHangOn)
		.setEndPoint(CrouchedWalking)
		.reverse([](){ return state_falling_and_hang_on; })
		.setEndPoint(walkingAnimation)
		.reverse([](){ return state_falling_and_hang_on; })
		;

	Graph->createLink(BallisticFly)
		.setEndPoint(jumpForwardAnimation)
		.reverse([](){ return state_ballistic_fly_to_target; })
		;

	Graph->createLink(treadingWaterAnimation)
		.setEndPoint(fallingIdleAnimation)
		.reverse([](){ return state_into_water; })
		.setEndPoint(jumpForwardAnimation)
		.reverse([](){ return state_into_water; })
		.setEndPoint(swimmingAnimation)
		.reverse([](){ return input_move.Length() == 0.0f; })
		;

	Graph->createLink(swimmingAnimation)
		.setEndPoint(treadingWaterAnimation)
		.reverse([](){ return input_move.Length() > 0.0f; })
		;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	auto FromCapsuleForwardToAnimHips = [](Animation* Anim, SimpleMath::Quaternion Delta)
	{
		sprintf(DebugBuffer, "Model::FromCapsuleForwardToAnimHips\n"); Debug();
		Simulation::UpdateCapsuleRotation_SetParams(Delta, SimpleMath::Quaternion::Concatenate(Delta, GWorld.Capsules["eve"].orientation));
	};
	auto FromCapsuleForwardToLedgeForward = []()
	{
		sprintf(DebugBuffer, "Model::FromCapsuleForwardToLedgeForward\n"); Debug();
		auto V1 = GWorld.Capsules["eve"].getMatrix().Right();
		auto V2 = -state_hanging_Ledge_Box.worldBackSide;
		TDeltaRotation DeltaRotation(V1, V2);
		Simulation::UpdateCapsuleRotation_SetParams(DeltaRotation.Delta, state_hanging_Ledge_Box.GetPawnOrientation());
	};
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 	Graph->createLink(Climb_Look_Idle_L)
 		.setEndPoint(Hanging_Idle_With_Leg)
		.reverse([FromCapsuleForwardToAnimHips, Climb_Look_Idle_L_Delta_Rotation, Climb_Look_Idle_L](){
			const auto Forward = GWorld.Capsules["eve"].getMatrix().Right();
			if (input_move.x < 0.0f && Forward.Cross(cameraForward).y > 0.f)
			{
				FromCapsuleForwardToAnimHips(Climb_Look_Idle_L, Climb_Look_Idle_L_Delta_Rotation);
				return true;
			}
			return false;
		})
		;

	Graph->createLink(Climb_Look_Idle_R)
		.setEndPoint(Hanging_Idle_With_Leg)
		.reverse([FromCapsuleForwardToAnimHips, Climb_Look_Idle_R_Delta_Rotation, Climb_Look_Idle_R](){
			const auto Forward = GWorld.Capsules["eve"].getMatrix().Right();
			if (input_move.x < 0.0f && Forward.Cross(cameraForward).y < 0.f)
			{
				FromCapsuleForwardToAnimHips(Climb_Look_Idle_R, Climb_Look_Idle_R_Delta_Rotation);
				return true;
			}
			return false;
		})
		;

	Graph->createLink(JumpFromWall)
		.setEndPoint(Climb_Look_Idle_R)
		.reverse([](){
			return consumeInputJump();
		})
		.setEndPoint(Climb_Look_Idle_L)
		.reverse([](){
			return consumeInputJump();
		})
		;

	Graph->createLink(idleAnimation)
		.setEndPoint(JumpFromWall)
		.reverse([](){
			return !state_jump_from_wall;
		})
		;

	Graph->createLink(Hanging_Idle_WithOut_Leg)
		.setEndPoint(FallingAndHangOn)
		.reverse([](){ return ready_hanging_on_wall; })
		.setEndPoint(RightShimmy)
		.reverse([RightShimmy](){ return fabs(input_move.y) == 0.0f && GetShimmyAnimationKind(RightShimmy) == 1; })
		.setEndPoint(LeftShimmy)
		.reverse([LeftShimmy](){ return fabs(input_move.y) == 0.0f && GetShimmyAnimationKind(LeftShimmy) == 1; })
		//.setEndPoint(Jump_To_Hang_WithOut_Leg)
		//.reverse([Graph](){ return !Graph->getAnimationBlend()->isPlaying(); })
		;

	Graph->createLinks(2, PackGroup(Left_Edge_Horizontal_Jump, Right_Edge_Horizontal_Jump))
		.setEndPoint(PackGroup(LeftShimmy, RightShimmy))
		.reverse([Graph](){
			bool GetEdgeHorizontalJumpAnimation_NextPose(IAnimationGraph2 * Graph, Animation* Anim, bool bStarting);

			if (consumeInputJump())
			{
				//�������
				//{
					extern bool simulation_state_manual_control;
					//simulation_state_manual_control = true;
				//}

				state_jump = true;
				state_hanging_Toe_activated = true;
				state_hanging_Toe_Name = Graph->GetCurrentConditionNodeName() == "Left_Edge_Horizontal_Jump" ? "RightToe_End" : "LeftToe_End";
				//GetHandLocation ��������� �� ������ �������, �� ���� �� ������ ������, ������������� �������� ����� ����� ������
				state_hanging_Toe_Location = GetHandLocation(SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix(), state_hanging_Toe_Name);

				start_hanging_Ledge_Name = state_hanging_Ledge_Name;
				start_hanging_Ledge_BoxIndex = state_hanging_Ledge_BoxIndex;

				return GetEdgeHorizontalJumpAnimation_NextPose(Graph, (Animation*)(Graph->GetCurrentConditionNodeAnimation()), true);
			}
			else
			{
				return false;
			}
		})
		;

	Graph->createLinks(2, PackGroup(Left_Edge_Horizontal_Jump, Right_Edge_Horizontal_Jump))
		.setEndPoint(PackGroup(Left_Edge_Horizontal_Jump, Right_Edge_Horizontal_Jump))
		.reverse([Graph](){
			bool GetEdgeHorizontalJumpAnimation_NextPose(IAnimationGraph2 * Graph, Animation* Anim, bool bStarting);

			return GetEdgeHorizontalJumpAnimation_NextPose(Graph, (Animation*)(Graph->GetCurrentConditionNodeAnimation()), false);
		})
		;

	Graph->createLinks(2, PackGroup(Jump_To_Hang_With_Leg, Jump_To_Hang_With_Leg))
		.setEndPoint(PackGroup(Left_Edge_Horizontal_Jump, Right_Edge_Horizontal_Jump))
		.reverse([FromCapsuleForwardToLedgeForward](){
			auto bRet = start_hanging_Ledge_Name != state_hanging_Ledge_Name || start_hanging_Ledge_BoxIndex != state_hanging_Ledge_BoxIndex;
			if (state_hanging && bRet)
			{
				auto LeftHand = GetHandLocation(SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix(), "LeftHand");
				auto RightHand = GetHandLocation(SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix(), "RightHand");
				sprintf(DebugBuffer, "Left_Edge_Horizontal_Jump -> Jump_To_Hang_With_Leg LeftHand == [%f %f %f] RightHand == [%f %f %f]\n", LeftHand.x, LeftHand.y, LeftHand.z, RightHand.x, RightHand.y, RightHand.z); Debug();
				//EdgeHorizontalJumpAnimation::resetPosesStates -111.211891 17.973904 -34.255638 -111.211891 17.973906 -34.255638
				//Left_Edge_Horizontal_Jump -> Jump_To_Hang_With_Leg -111.211899 17.974941 -39.226669

				FromCapsuleForwardToLedgeForward();
				state_jump = false;
				return true;
			}
			return false; 
		})
		;

	Graph->createLink(Hanging_Idle_With_Leg)
		.setEndPoint(Jump_To_Hang_With_Leg)
		.reverse([Graph](){ 
			auto PrevAnimName = Graph->getPrevAnimationName();
			return
				(
					PrevAnimName == "BallisticFly" || PrevAnimName == "JumpFromWall" ||
					PrevAnimName == "Left_Edge_Horizontal_Jump" || PrevAnimName == "Right_Edge_Horizontal_Jump"
				)
				&& !Graph->getAnimationBlend()->isPlaying();
			})
	;
	Graph->createLink(Hanging_Idle_WithOut_Leg)
		.setEndPoint(Jump_To_Hang_WithOut_Leg)
		.reverse([Graph](){
			auto PrevAnimName = Graph->getPrevAnimationName();
			return
				(
				PrevAnimName == "BallisticFly" || PrevAnimName == "JumpFromWall" ||
				PrevAnimName == "Left_Edge_Horizontal_Jump" || PrevAnimName == "Right_Edge_Horizontal_Jump"
				)
				&& !Graph->getAnimationBlend()->isPlaying();
		})
	;

	Graph->createLink(Climb_Fold_Hands)
		.setEndPoint(RightShimmy)
		.reverse([RightShimmy](){ 
			bool bRet = fabs(input_move.y) == 0.0f && IsShimmyAnimationEdgeBreakOff(RightShimmy); 
			if (bRet){
				auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
				state_hanging_Hand_Location = ClimbingPathHelper->ToSegmentBasis(GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = LeftHandName), ClimbingPathHelper->GetCurrentSegmentIndex());
			}
			return bRet;
		})
		.setEndPoint(LeftShimmy)
		.reverse([LeftShimmy](){
			bool bRet = fabs(input_move.y) == 0.0f && IsShimmyAnimationEdgeBreakOff(LeftShimmy);
			if (bRet){
				auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
				state_hanging_Hand_Location = ClimbingPathHelper->ToSegmentBasis(GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = RightHandName), ClimbingPathHelper->GetCurrentSegmentIndex());
			}
			return bRet;
		})
		;

	Graph->createLink(Hanging_Idle_With_Leg)
		.setEndPoint(Climb_Fold_Hands)
		.reverse([Graph](){
			bool bRet = fabs(input_move.y) == 0.0f && !Graph->getAnimationBlend()->isPlaying();
			if (bRet){
				auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
				if(state_hanging_Hand_Name == RightHandName)
					state_hanging_Hand_Location = ClimbingPathHelper->ToSegmentBasis(GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = LeftHandName), ClimbingPathHelper->GetCurrentSegmentIndex());
				else
					state_hanging_Hand_Location = ClimbingPathHelper->ToSegmentBasis(GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name = RightHandName), ClimbingPathHelper->GetCurrentSegmentIndex());
			}
			return bRet;
		})
		;

	Graph->createLink(Hanging_Idle_With_Leg)
		.setEndPoint(RightShimmy)
		.reverse([RightShimmy](){ return fabs(input_move.y) == 0.0f && GetShimmyAnimationKind(RightShimmy) == 2; })
		.setEndPoint(LeftShimmy)
		.reverse([LeftShimmy](){ return fabs(input_move.y) == 0.0f && GetShimmyAnimationKind(LeftShimmy) == 2; })
		.setEndPoint(FallingAndHangOn)
		.reverse([FallingAndHangOn](){ 
			auto TargetForward = -state_hanging_Ledge_Box.worldBackSide;
			TargetForward.Normalize();
			auto A = TargetForward.Dot(GWorld.Capsules["eve"].getMatrix().Right());
			return state_movement_is_obstructed && 0.99999f < A && A < 1.00001f;
		})
		.setEndPoint(Climb_Look_Idle_L)
		.reverse([FromCapsuleForwardToLedgeForward](){
			if (input_move.x == 0.0f)
			{
				FromCapsuleForwardToLedgeForward();
				return true;
			}
			return false;
		})
		.setEndPoint(Climb_Look_Idle_R)
		.reverse([FromCapsuleForwardToLedgeForward](){
			if (input_move.x == 0.0f)
			{
				FromCapsuleForwardToLedgeForward();
				return true;
			}
			return false;
		})
		.setEndPoint(bracedHangHopUpAnimation)
		.reverse([](){ return start_hanging_Ledge_Name != state_hanging_Ledge_Name || start_hanging_Ledge_BoxIndex != state_hanging_Ledge_BoxIndex; })
		.setEndPoint(jumpUpAnimation)
		.reverse([](){ return state_jump && state_hanging; })
		//.setEndPoint(Jump_To_Hang_With_Leg)
		//.reverse([Graph](){ return (Graph->getPrevAnimationName() == "BallisticFly" || Graph->getPrevAnimationName() == "JumpFromWall") && !Graph->getAnimationBlend()->isPlaying(); })
		.setEndPoint(ActionPose_Release_And_Go_Down)
		.reverse([](){ 
			auto bRet = start_hanging_Ledge_Name != state_hanging_Ledge_Name || start_hanging_Ledge_BoxIndex != state_hanging_Ledge_BoxIndex; 
			if (bRet)
			{
				state_falling = false;
			}
			return bRet;
		})
		;

	Graph->createLink(ActionPose_Release_And_Go_Down)
		.setEndPoint(Hanging_Idle_With_Leg)
		.reverse([Graph](){ return state_kneeling; })
		;

	Graph->createLink(Climb)
		.setEndPoint(Hanging_Idle_With_Leg)
		.reverse([](){ return input_move.x > 0.0f; })
		;

	Graph->createLink(FreehangClimb)
		.setEndPoint(FallingAndHangOn)
		.reverse([](){ return fabs(input_move.x) > 0.0f && ready_for_moving_from_falling_and_hang_on; })
		.setEndPoint(Hanging_Idle_WithOut_Leg)
		.reverse([](){ return fabs(input_move.x) > 0.0f; })
		;

	Graph->createLink(RightShimmy)
		.setEndPoint(FallingAndHangOn)
		.reverse([](){ return input_move.y > 0.0f && ready_for_moving_from_falling_and_hang_on; })
		.setEndPoint(Hanging_Idle_WithOut_Leg)
		.reverse([](){ return input_move.y > 0.0f; })
		.setEndPoint(Hanging_Idle_With_Leg)
		.reverse([](){ return input_move.y > 0.0f; })
		;

	Graph->createLink(LeftShimmy)
		.setEndPoint(FallingAndHangOn)
		.reverse([](){ return input_move.y < 0.0f && ready_for_moving_from_falling_and_hang_on; })
		.setEndPoint(Hanging_Idle_WithOut_Leg)
		.reverse([](){ return input_move.y < 0.0f; })
		.setEndPoint(Hanging_Idle_With_Leg)
		.reverse([](){ return input_move.y < 0.0f; })
		;

	Graph->createLink(Jump_To_Hang_WithOut_Leg)
		.setEndPoint(BallisticFly)
		.reverse([](){ return state_hanging && state_BallisticFly_to_HangingIdleWithOutLeg; })
		;

	Graph->createLink(Jump_To_Hang_With_Leg)
		.setEndPoint(BallisticFly)
		.reverse([](){ return state_hanging && state_BallisticFly_to_HangingIdle; })
		;

	Graph->createLink(Jump_To_Hang_WithOut_Leg)
		.setEndPoint(JumpFromWall)
		.reverse([](){ return state_hanging && state_BallisticFly_to_HangingIdleWithOutLeg; })
		;

	Graph->createLink(Jump_To_Hang_With_Leg)
		.setEndPoint(JumpFromWall)
		.reverse([](){ return state_hanging && state_BallisticFly_to_HangingIdle; })
		;
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	Graph->start(idleAnimation);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	auto _DebugAnimation = loadAnimationFromBlender("Media\\Animations\\Edge\\Edge_Right_Horizontal_Jump_Pose_1.dae", characterSkelet->FramesNamesIndex);

	//AnimationSetJointT(_DebugAnimation, 64, SimpleMath::Vector3(0,80,0));
	//rotateHips(_DebugAnimation, SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix::CreateRotationY(.0f*PI / 180.0)));
	FixAnimJointsOrientation(_DebugAnimation, TPose);

	extractAnimationMeta(_DebugAnimation, true, 1.0f, getSkeletMatrix, calculateFramesTrans);

	_DebugAnimation->reset();

	DebugAnimation = _DebugAnimation;
}