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

void FillGrid_Indexed(std::vector<VertexPositionNormalTangentColorTextureSkinning2> & _vertices, std::vector<UINT> & _indices, DWORD dwWidth, DWORD dwLength,
	_In_opt_ std::function<float __cdecl(SimpleMath::Vector3)> setHeight)
{
	// Fill vertex buffer
	for (DWORD i = 0; i <= dwLength; ++i)
	{
		for (DWORD j = 0; j <= dwWidth; ++j)
		{
			VertexPositionNormalTangentColorTextureSkinning2    pVertex;
			pVertex.position.x = ((float)j / dwWidth);
			pVertex.position.z = ((float)i / dwLength);
			pVertex.position.y = setHeight(pVertex.position);
			_vertices.push_back(pVertex);
		}
	}

	// Fill index buffer
	int index = 0;
	for (DWORD i = 0; i < dwLength; ++i)
	{
		for (DWORD j = 0; j < dwWidth; ++j)
		{
			_indices.push_back((DWORD)(i     * (dwWidth + 1) + j));
			_indices.push_back((DWORD)((i + 1) * (dwWidth + 1) + j));
			_indices.push_back((DWORD)(i     * (dwWidth + 1) + j + 1));

			_indices.push_back((DWORD)(i     * (dwWidth + 1) + j + 1));
			_indices.push_back((DWORD)((i + 1) * (dwWidth + 1) + j));
			_indices.push_back((DWORD)((i + 1) * (dwWidth + 1) + j + 1));
		}
	}
}
