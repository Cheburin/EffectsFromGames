#include "main.h"
#include "AnimationImpl.h"
#include <fstream>

#include <locale>
#include <codecvt>
#include <string>
#include <array>
#include <map>
#include <locale> 

Ledge ClimbingPath::Create(const Box* box1, const Box* box2)
{
	/*
	struct VertexMeta
	{
		struct BoxLink
		{
			unsigned short BoxIndex;
			unsigned short VertexIndex;
		};

		SimpleMath::Vector3 Location;

	};
	*/

	std::vector< SimpleMath::Vector3 > vVertexLocation;

	std::vector< std::set<unsigned int> > vVertexBoxIndices; 

	std::vector< std::map<unsigned int, unsigned int> > vBoxWinding;

	auto ExtractVertexFromBox = [&vVertexLocation, &vVertexBoxIndices, &vBoxWinding](unsigned short BoxIndex, const Box& box) //&Vertex, &BoxVertexIndex
	{
		std::map<unsigned int, unsigned int> Winding;

		std::set<unsigned int> BoxIndices;

		BoxIndices.insert(BoxIndex);

		SimpleMath::Vector2 RelativeBoxVertexLocation[] = {
			SimpleMath::Vector2(-1, -1),
			SimpleMath::Vector2(-1, 1),
			SimpleMath::Vector2(1, 1),
			SimpleMath::Vector2(1, -1)
		};

		auto BoxBasis = box.getMatrix();

		auto HalfSize = box.getHalfSize();

		auto X_Basis_Vector = HalfSize.x * BoxBasis.Right();
		auto Z_Basis_Vector = HalfSize.z * BoxBasis.Backward();

		auto Origin = BoxBasis.Translation() + HalfSize.y * BoxBasis.Up();

		auto PreviousBoxesVertexCount = vVertexLocation.size();

		int CurrentBoxVertexCount = 0;

		int VertexIndices[4];

		for (int i = 0; i < 4; i++)
		{
			auto Location = Origin + RelativeBoxVertexLocation[i].x * X_Basis_Vector + RelativeBoxVertexLocation[i].y * Z_Basis_Vector;

			//исключим на первом этапе дубли вершин
			int VertexIndex = 0;
			for (VertexIndex = 0; VertexIndex < PreviousBoxesVertexCount; VertexIndex++)
			{
				if ((vVertexLocation[VertexIndex] - Location).LengthSquared() < 0.001f)
					break;
			}

			if (VertexIndex == PreviousBoxesVertexCount)
			{
				vVertexLocation.push_back(Location);

				VertexIndices[i] = VertexIndex + CurrentBoxVertexCount++;
				vVertexBoxIndices.push_back(BoxIndices);
			}
			else
			{
				VertexIndices[i] = VertexIndex;
				vVertexBoxIndices[VertexIndex].insert(BoxIndex);
			}
		}

		for (int i = 0; i < 4; i++)
		{
			Winding.insert(std::pair<unsigned int, unsigned int>(VertexIndices[i], VertexIndices[(i + 1) % 4]));
		}

		vBoxWinding.push_back(Winding);
	};

	auto IsVertexOnTheEdge = [&vVertexLocation](int Index, int EdgeIndex1, int EdgeIndex2)
	{
		if (Index != EdgeIndex1 && Index != EdgeIndex2)
		{
			auto A = vVertexLocation[EdgeIndex1];
			auto B = vVertexLocation[EdgeIndex2];
			auto C = vVertexLocation[Index];
			auto AB = B - A;
			auto AC = C - A;
			auto t = AB.Dot(AC) / AB.Dot(AB);
			auto projConAB = t * AB + A;
			return 0.0f < t && t < 1.0f && (projConAB - C).LengthSquared() < 0.001f;
		}
		return false;
	};

	auto WeldBoxes = [&vVertexLocation, &vVertexBoxIndices, &vBoxWinding, &IsVertexOnTheEdge]()//&Vertex, &BoxRelation, &BoxVertexIndex
	{
		//лежат ли вершины бокса на ребрах других боксов?
		for (int i = 0; i < vBoxWinding.size(); i++)
		{
			//вершины бокса
			for (auto j = vBoxWinding[i].begin(); j != vBoxWinding[i].end(); j++)
			{
				//другие боксы
				for (int k = 0; k < vBoxWinding.size(); k++)
				{
					//исключаем самого себя
					if (i != k)
					{
						//ребра другого бокса
						for (auto m = vBoxWinding[k].begin(); m != vBoxWinding[k].end(); m++)
						{
							//в нашем случае вершина может лежать только на одном ребре
							if (IsVertexOnTheEdge(j->first, m->first, m->second))
							{
								auto SaveEdge = *m;
								vBoxWinding[k].erase(SaveEdge.first);
								vBoxWinding[k].insert(std::pair<unsigned int, unsigned int>(SaveEdge.first, j->first));
								vBoxWinding[k].insert(std::pair<unsigned int, unsigned int>(j->first, SaveEdge.second));
								vVertexBoxIndices[j->first].insert(k);
								break;
							}
						}
					}
				}
			}
		}
	};

	auto GetVertexFirstBoxIndex = [&vVertexBoxIndices](int VertexIndex)
	{
		for (auto r = vVertexBoxIndices[VertexIndex].begin(); r != vVertexBoxIndices[VertexIndex].end(); r++)
			return *r;

		throw "Bad Box Index";

		return (unsigned int)0;
	};


	auto NextVertexBoxIndex = [&vVertexBoxIndices](int VertexIndex, unsigned int BoxIndex)
	{
		for (auto r = vVertexBoxIndices[VertexIndex].begin(); r != vVertexBoxIndices[VertexIndex].end(); r++)
			if (BoxIndex != *r)
				return *r;

		throw "Bad Next Box Index";

		return (unsigned int)0;
	};

	auto NextBoxAlignedVertexIndex = [&vVertexBoxIndices, &vBoxWinding](int VertexIndex, int BoxIndex)
	{
		auto i = vBoxWinding[BoxIndex].find(VertexIndex);

		if (vBoxWinding[BoxIndex].end() == i)
			throw "Bad Next Box Aligned Vertex Index";

		return i->second;
	};

	auto CompileResultPath = [&vVertexLocation, &vVertexBoxIndices, &vBoxWinding, &GetVertexFirstBoxIndex, &NextBoxAlignedVertexIndex, &NextVertexBoxIndex]()
	{
		std::vector<SimpleMath::Vector3> Result;

		unsigned int BoxIndex;

		int StartVertexIndex = 0;

		for (; StartVertexIndex < vVertexLocation.size(); StartVertexIndex++)
		{
			if (vVertexBoxIndices[StartVertexIndex].size() == 1)
			{
				break;
			}
		}

		if (StartVertexIndex < vVertexLocation.size())
		{
			BoxIndex = GetVertexFirstBoxIndex(StartVertexIndex);

			Result.push_back(vVertexLocation[StartVertexIndex]);

			for (int VertexIndex = NextBoxAlignedVertexIndex(BoxIndex, StartVertexIndex); true;)
			{
				if (VertexIndex == StartVertexIndex)
					break;

				Result.push_back(vVertexLocation[VertexIndex]);

				if (vVertexBoxIndices[VertexIndex].size() != 1)
				{
					BoxIndex = NextVertexBoxIndex(VertexIndex, BoxIndex);
				}

				VertexIndex = NextBoxAlignedVertexIndex(VertexIndex, BoxIndex);
			}
		}

		return Result;
	};

	std::vector<Box> Owners;

	if (box1 != nullptr)
	{
		ExtractVertexFromBox(0, *box1);
		Owners.push_back(*box1);
	}

	if (box2 != nullptr)
	{
		ExtractVertexFromBox(1, *box2);
		Owners.push_back(*box2);
	}

	WeldBoxes();

	auto Points = CompileResultPath();

	auto Boxes = std::vector< Box2 >();
	for (int i = 0; i < Points.size(); i++)
	{
		auto A = Points[i];
		auto B = Points[(i + 1) % Points.size()];
		auto Center = 0.5f * (B + A);
		auto X = B - A;
		auto XExtent = X.Length();
		X.Normalize();
		auto Y = SimpleMath::Vector3(0, 1, 0);
		auto Z = X.Cross(Y);

		Box2 box = Box2();
		box.overlap = true;
		box.origin = Center;
		box.orientation = SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix(X, Y, Z));
		box.size = SimpleMath::Vector3(XExtent, 0.4, 0.4);
		box.eval();

		Boxes.push_back(box);
	}

	Ledge Path = { Owners, Boxes };

	return Path;
};

Ledge ClimbingPath::Create(const Box* box1)
{
	return ClimbingPath::Create(box1, nullptr);
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct HandClimbingState
{
	int Index;

	bool Forward;

	float Distantce[2];

	float HandAdvanse;

	float HalfHandSize;

	float RegularHandAdvanse;

	std::vector<SimpleMath::Vector3>& Path;

	HandClimbingState(std::vector<SimpleMath::Vector3>& NewPath);

	void Init(const SimpleMath::Vector3& Hand);

	void AdvanseHand(SimpleMath::Vector3& Hand, HandClimbingState& SlaveState);
	
	// , SimpleMath::Vector3 Target, HandState& TheHandState);
};

HandClimbingState::HandClimbingState(std::vector<SimpleMath::Vector3>& NewPath)
	:Path(NewPath)
{

}

void HandClimbingState::Init(const SimpleMath::Vector3& hand)
{

}

void HandClimbingState::AdvanseHand(SimpleMath::Vector3& MovableHand, HandClimbingState& SlaveState)
{
	auto IsOutOfSegment = []()
	{
		return false;
	};

	auto IsFirstOnTheWay = []()
	{
		return false;
	};

	auto AdvanceMovableHandOnRegularDistantce = []()
	{
	};

	auto SetStateHandAdvanseToRegular = []()
	{
	};

	auto SetMovableHandOnStartNextSegment = []()
	{
	};

	auto SetSlaveStateHandAdvanseToSuitOnEndCurrentSegment = []()
	{
	};

	if (IsFirstOnTheWay() && IsOutOfSegment())
	{
		SetMovableHandOnStartNextSegment();

		SetSlaveStateHandAdvanseToSuitOnEndCurrentSegment();
	}
	else
	{
		AdvanceMovableHandOnRegularDistantce();

		SetStateHandAdvanseToRegular();
	}
}
