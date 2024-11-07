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


//collision core
SimpleMath::Vector2 getDistanceBetweenSkewLines(SimpleMath::Vector3 S1, SimpleMath::Vector3 V1, SimpleMath::Vector3 S2, SimpleMath::Vector3 V2){
	using namespace SimpleMath;
	float c = 1.0f / (pow(V1.Dot(V2), 2.0f) - V1.Dot(V1)*V2.Dot(V2));
	float a = (S2 - S1).Dot(V1);
	float b = (S2 - S1).Dot(V2);
	Vector2 ret;
	ret.x = c*(-V2.Dot(V2)*a + V1.Dot(V2)*b);
	ret.y = c*(-V1.Dot(V2)*a + V1.Dot(V1)*b);
	return ret;
}
bool getLowestRoot(float a, float b, float c, float maxR, float* root)
{
	// Check if a solution exists
	float determinant = b*b - 4.0f*a*c;
	// If determinant is negative it means no solutions.
	if (determinant < 0.0f) return false;
	// calculate the two roots: (if determinant == 0 then
	// x1==x2 but lets disregard that slight optimization)
	float sqrtD = sqrt(determinant);
	float r1 = (-b - sqrtD) / (2 * a);
	float r2 = (-b + sqrtD) / (2 * a);
	// Sort so x1 <= x2
	if (r1 > r2) {
		float temp = r2;
		r2 = r1;
		r1 = temp;
	}
	//*root = r1;
	//return true;
	// Get lowest root:
	if (0.0f <= r1 && r1 <= maxR) {
		*root = r1;
		return true;
	}
	// It is possible that we want x2 - this can happen
	// if x1 < 0
	if (0.0f <= r2 && r2 <= maxR) {
		*root = r2;
		return true;
	}

	// No (valid) solutions
	return false;
}
SimpleMath::Vector4 vec4(SimpleMath::Vector3 v3, float w)
{
	return SimpleMath::Vector4(v3.x, v3.y, v3.z, w);
}
namespace {
	SimpleMath::Vector3 normalize(SimpleMath::Vector3 v)
	{
		v.Normalize();
		return v;
	}
}
SimpleMath::Vector3 projectOnVector(const SimpleMath::Vector3 & vector, const SimpleMath::Vector3 & onVector)
{
	return (onVector.Dot(vector) / onVector.Dot(onVector)) * onVector;
}
SimpleMath::Vector2 projectOnZ(const SimpleMath::Vector3& v)
{
	return SimpleMath::Vector2(v.x, v.z);
}
struct Cylinder{
	SimpleMath::Vector3 A;
	SimpleMath::Vector3 B;
	float r;
	Cylinder(float _1, float _2, float _3, float _4, float _5, float _6, float _r)
	{
		A.x = _1;
		A.y = _2;
		A.z = _3;
		B.x = _4;
		B.y = _5;
		B.z = _6;
		r = _r;
	}
};
struct Line{
	SimpleMath::Vector3 A;
	SimpleMath::Vector3 B;
	Line(float _1, float _2, float _3, float _4, float _5, float _6)
	{
		A.x = _1;
		A.y = _2;
		A.z = _3;
		B.x = _4;
		B.y = _5;
		B.z = _6;
	}
};
float correctMistake(float t, float r, const SimpleMath::Vector4 & plane, const SimpleMath::Vector3 & prev_origin, const SimpleMath::Vector3 & origin)
{
	auto dist_prev_origin = plane.Dot(vec4(prev_origin, 1.0f));
	auto dist_origin = plane.Dot(vec4(origin, 1.0f));
	if (dist_origin < r)
	{
		t = t - (r - dist_origin) / (dist_prev_origin - dist_origin) * t;
	}
	return t < 0.0f ? 0.0f : t;
}
float correctMistake(float t, float r, const SimpleMath::Vector3 & worldDistance, float worldDistanceLength, const SimpleMath::Vector3 & origin, const SimpleMath::Vector3 & impactPoint)
{
	auto impactPointRel = impactPoint - origin;
	//if exists some penetration due float not accuracy, correct this
	if (impactPointRel.Dot(impactPointRel) < r*r)
	{
		auto target_proj_r = projectOnVector(r*normalize(impactPointRel), worldDistance).Length();
		auto proj_r = projectOnVector(impactPointRel, worldDistance).Length();
		if (proj_r > 0)
		{
			t = t - (target_proj_r - proj_r) / worldDistanceLength;
		}
		else
		{
			t = t - (abs(target_proj_r) - abs(proj_r) + 2.0f*abs(proj_r)) / worldDistanceLength;
		}
	}
	return t < 0.0f ? 0.0f : t;
}
float offsetBySmallDistantce(float t, float worldDistanceLength)
{
	t = t - 0.05f / worldDistanceLength;
	return t < 0.0f ? 0.0f : t;
}
bool collisionWithEdge(const Cylinder& cylinder, const Line& line, const SimpleMath::Vector3 & worldDistance, SimpleMath::Vector3 & impactPoint, SimpleMath::Vector3 & impactNormal, float & impactT)
{
	impactT = 1.0f;

	bool collision = false;

	auto lineBA = line.B - line.A;
	auto cylinderBA = cylinder.B - cylinder.A;
	auto cylinderRcylinderR = cylinder.r*cylinder.r;
	auto worldDistanceWorldDistance = worldDistance.Dot(worldDistance);
	auto worldDistanceLength = sqrt(worldDistanceWorldDistance);
	SimpleMath::Vector3 vCylinderX[] = { cylinder.A, cylinder.B };
	SimpleMath::Vector3 vLineX[] = { line.A, line.B };

	SimpleMath::Matrix ToCylinderLocalSpace;
	{
		// std make frame
		// trivial impl
		SimpleMath::Vector3 X(1, 0, 0);
		auto Y = cylinderBA;
		Y.Normalize();

		// check X collinearity
		if (1.0f - abs(X.Dot(Y)) < 0.001f)
		{
			X = SimpleMath::Vector3(0, 1, 0);
		}

		auto Z = X.Cross(Y);
		X = Y.Cross(Z);

		X.Normalize();
		Y.Normalize();
		Z.Normalize();
		// Cylinder it is Y
		ToCylinderLocalSpace = SimpleMath::Matrix(X, Y, Z);
		ToCylinderLocalSpace = ToCylinderLocalSpace.Transpose();
	}

	SimpleMath::Vector2 projCylinderA = projectOnZ(SimpleMath::Vector3::Transform(cylinder.A, ToCylinderLocalSpace));
	SimpleMath::Vector2 vProjLineX[] = { projectOnZ(SimpleMath::Vector3::Transform(line.A, ToCylinderLocalSpace)), projectOnZ(SimpleMath::Vector3::Transform(line.B, ToCylinderLocalSpace)) };
	SimpleMath::Vector2 projWorldDistance = projectOnZ(SimpleMath::Vector3::TransformNormal(worldDistance, ToCylinderLocalSpace));
	SimpleMath::Vector2 projLineBA = projectOnZ(SimpleMath::Vector3::TransformNormal(lineBA, ToCylinderLocalSpace));
	auto projWorldDistanceProjWorldDistance = projWorldDistance.Dot(projWorldDistance);

	float last_t = 1.0f;

	//WITH EDGE(begin)////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//----Cylinder Ends Sphera's 
	{
		float t = 1.0f, a, b, c;

		bool rootFound = false;
		int lastRootFoundOnIndex;

		auto VA = worldDistance.Dot(lineBA);
		auto AA = lineBA.Dot(lineBA);
		auto VV = worldDistanceWorldDistance;

		a = VV - pow(VA, 2.0f) / AA;

		for (int i = 0; i < 2; i++)
		{
			auto S = vCylinderX[i] - line.A;
			auto SA = S.Dot(lineBA);
			auto SS = S.Dot(S);
			auto SV = S.Dot(worldDistance);
			auto VA = worldDistance.Dot(lineBA);

			b = 2.0f*(SV - (SA*VA) / AA);
			c = SS - cylinderRcylinderR - (SA*SA) / AA;

			if (getLowestRoot(a, b, c, t, &t))
			{
				rootFound = true;
				lastRootFoundOnIndex = i;
			}
		}

		if (rootFound)
		{
			auto cylinderX = vCylinderX[lastRootFoundOnIndex];

			auto newCylinderX = cylinderX + t*worldDistance;

			auto l = (newCylinderX - line.A).Dot(lineBA) / lineBA.Dot(lineBA);

			if (.0f <= l&&l <= 1.0f)
			{
				auto lineX = line.A + l*lineBA;

				auto normalX = normalize(newCylinderX - lineX);

				if (lastRootFoundOnIndex == 0)
				{
					collision = cylinderBA.Dot(normalX) >= .0f;
				}
				else
				{
					collision = cylinderBA.Dot(normalX) <= .0f;
				}

				if (collision)
				{
					t = correctMistake(t, cylinder.r, vec4(normalX, -normalX.Dot(lineX)), cylinderX, newCylinderX);
					t = offsetBySmallDistantce(t, worldDistanceLength);

					impactNormal = normalX;
					impactPoint = lineX;
					impactT = last_t = t;
				}
			}
		}
	}
	//----Cylinder WITHOUT Ends Sphera's 
	//collide with line by self (2d)
	//check when |_d| near _epsilon, equation of line is degenerate, skip this
	if (projLineBA.Length() != 0.0f)
	{
		//make line equation
		SimpleMath::Vector3 pLine(projLineBA.y, -projLineBA.x, vProjLineX[0].y*projLineBA.x - vProjLineX[0].x*projLineBA.y);
		pLine *= (1.0 / sqrt(pLine.x*pLine.x + pLine.y*pLine.y));
		//swept circle into XZ
		auto lineDotS = pLine.Dot(SimpleMath::Vector3(projCylinderA.x, projCylinderA.y, 1.0f));
		auto lineDotV = pLine.Dot(SimpleMath::Vector3(projWorldDistance.x, projWorldDistance.y, 0.0f));
		//test sign of half space
		float sign = lineDotS < 0.0f ? -1.0f : 1.0f;

		float t = 1.0f;
		bool rootFound = false;
		SimpleMath::Vector2 skewLinesArgs;
		SimpleMath::Vector3 newCylinderA;

		//test if collision ocure
		if (abs(lineDotV) != 0.0f)
		{
			t = (cylinder.r * sign - lineDotS) / lineDotV;
			if (0.0f <= t&&t <= 1.0f)
			{
				auto newCylinderA = cylinder.A + t*worldDistance;

				skewLinesArgs = getDistanceBetweenSkewLines(newCylinderA, cylinderBA, line.A, lineBA);

				//check only if impact possess line cut (with finit cylinder)
				if (.0f <= skewLinesArgs.x&&skewLinesArgs.x <= 1.0f && .0f <= skewLinesArgs.y&&skewLinesArgs.y <= 1.0f)
				{
					rootFound = true;
				}
			}
		}

		if (rootFound)
		{
			auto lineX = line.A + skewLinesArgs.y * lineBA;

			auto newCylinderX = newCylinderA + skewLinesArgs.x * cylinderBA;

			auto cylinderX = newCylinderX - worldDistance;

			auto normalX = normalize(newCylinderX - lineX);

			t = correctMistake(t, cylinder.r, vec4(normalX, -normalX.Dot(lineX)), cylinderX, newCylinderX);
			t = offsetBySmallDistantce(t, worldDistanceLength);

			if (t < last_t)
			{
				collision = true;

				impactNormal = normalX;
				impactPoint = lineX;
				impactT = t;
			}
		}
	}
	//WITH EDGE(end)//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if (collision)
		return collision;

	last_t = 1.0f;

	//WITH VERTEX(begin)//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//----Cylinder Ends Sphera's 
	//collide end spheres with line points (3d)
	{
		float t = 1, a, b, c;

		bool rootFound = false;
		int lastRootFoundOnIndex1, lastRootFoundOnIndex2;

		a = worldDistanceWorldDistance;

		for (int i = 0; i < 2; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				auto d = vCylinderX[i] - vLineX[j];

				b = 2.0f*worldDistance.Dot(d);
				c = d.Dot(d) - cylinderRcylinderR;

				if (getLowestRoot(a, b, c, t, &t))
				{
					rootFound = true;
					lastRootFoundOnIndex1 = j;
					lastRootFoundOnIndex2 = i;
				}
			}
		}

		if (rootFound)
		{
			auto newCylinderX = vCylinderX[lastRootFoundOnIndex2] + t*worldDistance;

			auto lineX = vLineX[lastRootFoundOnIndex1];

			auto normalX = normalize(newCylinderX - lineX);

			if (lastRootFoundOnIndex2 == 0)
			{
				collision = cylinderBA.Dot(normalX) >= .0f;
			}
			else
			{
				collision = cylinderBA.Dot(normalX) <= .0f;
			}

			if (collision)
			{
				t = correctMistake(t, cylinder.r, worldDistance, worldDistanceLength, newCylinderX, lineX);
				t = offsetBySmallDistantce(t, worldDistanceLength);

				impactNormal = normalX;
				impactPoint = lineX;
				impactT = last_t = t;
			}
		}
	}
	//----Cylinder WITHOUT Ends Sphera's 
	//All to CylinderLocalSpace
	//project line to XZ , project distance to XZ , project cylinder.A to XZ
	//collide with line end points (2d)
	{
		float t = 1, a, b, c;

		bool rootFound = false;
		int lastRootFoundOnIndex;

		a = projWorldDistanceProjWorldDistance;

		for (int i = 0; i < 2; i++)
		{
			auto d = projCylinderA - vProjLineX[i];

			b = 2.0f*projWorldDistance.Dot(d);
			c = d.Dot(d) - cylinderRcylinderR;

			if (getLowestRoot(a, b, c, t, &t))
			{
				rootFound = true;
				lastRootFoundOnIndex = i;
			}
		}

		if (rootFound)
		{
			auto newCylinderA = cylinder.A + t*worldDistance;

			auto lineX = vLineX[lastRootFoundOnIndex];

			auto c = (lineX - newCylinderA).Dot(cylinderBA) / cylinderBA.Dot(cylinderBA);

			if (.0f <= c&&c <= 1.0f)
			{
				auto newCylinderX = newCylinderA + c * cylinderBA;

				auto normalX = normalize(newCylinderX - lineX);

				t = correctMistake(t, cylinder.r, worldDistance, worldDistanceLength, newCylinderX, lineX);
				t = offsetBySmallDistantce(t, worldDistanceLength);

				if (t < last_t)
				{
					collision = true;

					impactNormal = normalX;
					impactPoint = lineX;
					impactT = t;
				}
			}
		}
	}
	//WITH VERTEX(end)///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	return collision;
}
bool collisionWithFace(const Cylinder & cylinder, const SimpleMath::Matrix& faceBasis, const SimpleMath::Vector2& faceHalfSize, const SimpleMath::Vector3 & worldDistance, SimpleMath::Vector3 & impactPoint, SimpleMath::Vector3 & impactNormal, float & impactT){
	impactT = 1.0f;

	bool collision = false;

	SimpleMath::Vector4 pFace = vec4(faceBasis.Backward(), -faceBasis.Backward().Dot(faceBasis.Translation()));
	float vFaceDotS[2] = { pFace.Dot(vec4(cylinder.A, 1.0f)), pFace.Dot(vec4(cylinder.B, 1.0f)) };

	{

		if (vFaceDotS[0] > cylinder.r && vFaceDotS[1] > cylinder.r)
		{
			float t = 1.0f;
			int index = 0;

			bool rootFound = false;

			auto faceDotV = pFace.Dot(vec4(worldDistance, 0.0f));

			for (int i = 0; i < 2; i++)
			{
				if (abs(faceDotV) != 0.0f)
				{
					auto faceDotS = vFaceDotS[i];

					float last_t = (cylinder.r - faceDotS) / faceDotV;

					if (0.0f <= last_t && last_t <= t)
					{
						rootFound = true;
						t = last_t;
						index = i;
					}
				}
			}

			if (rootFound)
			{
				SimpleMath::Vector3 vCylinderX[2] = { cylinder.A, cylinder.B };

				auto worldDistanceLength = worldDistance.Length();

				auto cylinderX = vCylinderX[index];

				auto newCylinderX = cylinderX + t*worldDistance;

				auto relNewCylinderX = newCylinderX - faceBasis.Translation();

				auto x_coord = faceBasis.Right().Dot(relNewCylinderX);

				auto y_coord = faceBasis.Up().Dot(relNewCylinderX);

				if (abs(x_coord) <= faceHalfSize.x)
				{
					if (abs(y_coord) <= faceHalfSize.y)
					{
						collision = true;

						t = correctMistake(t, cylinder.r, pFace, cylinderX, newCylinderX);
						t = offsetBySmallDistantce(t, worldDistanceLength);

						impactNormal = faceBasis.Backward();
						impactPoint = x_coord*faceBasis.Right() + y_coord*faceBasis.Up() + faceBasis.Translation();
						impactT = t;
					}
				}
			}
		}
	}

	return collision;
}
bool collisionWithBox(const Cylinder & cylinder, const SimpleMath::Matrix& boxBasis, const SimpleMath::Vector3& boxHalfSize, const SimpleMath::Vector3 & worldDistance, SimpleMath::Vector3 & impactPoint, SimpleMath::Vector3 & impactNormal, float & impactT)
{
	bool collison = false;

	SimpleMath::Vector3 vFaceNormal[] = { boxBasis.Right(), boxBasis.Up(), boxBasis.Backward() };
	float vBoxHalfSize[] = { boxHalfSize.x, boxHalfSize.y, boxHalfSize.z };

	impactT = 1.0f;

	SimpleMath::Vector3 _impactPoint;
	SimpleMath::Vector3 _impactNormal;
	SimpleMath::Vector3 lastImpactPoint;
	SimpleMath::Vector3 lastImpactNormal;

	float _t = 1.0f;
	float last_t;

	for (int i = 0; i < 3; i++)
	{
		auto faceNormal = vFaceNormal[i];
		auto faceOffset = vBoxHalfSize[i];

		SimpleMath::Vector3 vFaceTB[] = { vFaceNormal[(i + 1) % 3], vFaceNormal[(i + 2) % 3] };
		float vFaceHalfSize[] = { vBoxHalfSize[(i + 1) % 3], vBoxHalfSize[(i + 2) % 3] };

		for (int j = 0; j < 2; j++)
		{
			faceNormal *= -1.0f;

			if (faceNormal.Dot(worldDistance) < .0f)
			{
				bool lastCollison = false;

				auto faceOrigin = boxBasis.Translation() + faceOffset * faceNormal;

				{
					SimpleMath::Matrix faceBasis(
						vec4(vFaceTB[0], 0),
						vec4(vFaceTB[1], 0),
						vec4(faceNormal, 0),
						vec4(faceOrigin, 1)
						);

					SimpleMath::Vector2 faceHalfSize(
						vFaceHalfSize[0],
						vFaceHalfSize[1]
						);

					lastCollison = collisionWithFace(cylinder, faceBasis, faceHalfSize, worldDistance, lastImpactPoint, lastImpactNormal, last_t);

					if (lastCollison)
					{
						if (last_t < _t)
						{
							collison = true;

							_impactPoint = lastImpactPoint;
							_impactNormal = faceNormal;
							_t = last_t;
						}
					}
				}

				if (!lastCollison){
					SimpleMath::Vector3 edgeNormal;

					for (int k = 0; k < 2; k++)
					{
						auto baseLineA = faceOrigin + vFaceHalfSize[k % 2] * vFaceTB[k % 2];
						auto baseLineB = faceOrigin - vFaceHalfSize[k % 2] * vFaceTB[k % 2];

						auto lineOffset = vFaceHalfSize[(k + 1) % 2] * vFaceTB[(k + 1) % 2];

						for (int l = 0; l < 2; l++)
						{
							lineOffset *= -1.0f;

							auto lineA = baseLineA + lineOffset;
							auto lineB = baseLineB + lineOffset;

							Line line(lineA.x, lineA.y, lineA.z, lineB.x, lineB.y, lineB.z);

							lastCollison = collisionWithEdge(cylinder, line, worldDistance, lastImpactPoint, lastImpactNormal, last_t);

							{
								auto midPoint = 0.5f*(line.A + line.B);
								auto tangent0 = midPoint - faceOrigin;
								tangent0.Normalize();
								if ((line.A - lastImpactPoint).LengthSquared() < 0.001f || (line.B - lastImpactPoint).LengthSquared() < 0.001f)
								{
									auto tangent1 = lastImpactPoint - midPoint;
									tangent1.Normalize();
									edgeNormal = (faceNormal);// +tangent0 + tangent1);
									edgeNormal.Normalize();
								}
								else
								{
									edgeNormal = (faceNormal);// +tangent0);
									edgeNormal.Normalize();
								}
							}

							if (lastCollison)
							{
								if (last_t < _t)
								{
									collison = true;

									_impactPoint = lastImpactPoint;
									_impactNormal = edgeNormal;
									_t = last_t;
								}
							}
						}
					}
				}

			}
		}
	}

	if (collison)
	{
		impactPoint = _impactPoint;
		impactNormal = _impactNormal;
		impactT = _t;
	}

	return collison;
}
extern World GWorld;
bool physCollisionWithBox(const Capsule& ca, const Box& box, const SimpleMath::Vector3 & worldDistance, SimpleMath::Vector3 & impactPoint, SimpleMath::Vector3 & impactNormal, float & t)
{
	SimpleMath::Vector3 A, B;

	ca.getAB(A, B);

	return collisionWithBox(Cylinder(A.x, A.y, A.z, B.x, B.y, B.z, ca.r), box.getMatrix(), box.getHalfSize(), worldDistance, impactPoint, impactNormal, t);
}
void physCollision(const bool sweep, Capsule& ca, const SimpleMath::Vector3 & worldDistance)
{
	SimpleMath::Vector3 impactPoint;
	SimpleMath::Vector3 impactNormal;

	float min_t = 1.0f, t;

	auto box_iter = GWorld.Boxes.begin();
	auto total_boxes = GWorld.Boxes.size();
	for (int i = 0; i < total_boxes; i++)
	{
		if (physCollisionWithBox(ca, (box_iter++)->second, worldDistance, impactPoint, impactNormal, t))
		{
			if (t < min_t)
			{
				min_t = t;
			}
		}
	}

	auto worldDistanceOffset = min_t*worldDistance;

	ca.origin += worldDistanceOffset;

	if (!sweep)
	{
		return;
	}

	{
		float remaining_t = 1.0 - min_t;

		if (remaining_t > 0.0f)
		{
			auto perpWorldDir = worldDistance - worldDistance.Dot(impactNormal) * impactNormal;
			perpWorldDir.Normalize();

			auto remainingWorldDistance = worldDistance.Length() * perpWorldDir;

			min_t = 1.0f;

			auto box_iter = GWorld.Boxes.begin();
			auto total_boxes = GWorld.Boxes.size();
			for (int i = 0; i < total_boxes; i++)
			{
				if (physCollisionWithBox(ca, (box_iter++)->second, remainingWorldDistance, impactPoint, impactNormal, remaining_t))
				{
					if (remaining_t < min_t)
					{
						min_t = remaining_t;
					}
				}
			}

			auto remainingWorldDistanceOffset = min_t*remainingWorldDistance;

			ca.origin += remainingWorldDistanceOffset;
		}
	}
}
//collision core

// коллизия между
// орентированными капсула, прямоуголный объем, сфера, цилиндер, мешь с последующем слайдом

using namespace SimpleMath;

void World::Collision(const SimpleMath::Vector3& S, const SimpleMath::Vector3& V, Capsule& ca, HitInfo& hi)
{

	physCollision(true, ca, V);

	//ca.origin = S + V;
}
