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
	float r;
	float cylinderRcylinderR;
	SimpleMath::Vector3 A;
 	SimpleMath::Vector3 B;
	SimpleMath::Vector3 BA;
	SimpleMath::Vector3 vX[2];
	SimpleMath::Matrix ToCylinderLocalSpace;
	Cylinder(float _1, float _2, float _3, float _4, float _5, float _6, float _r)
	{
		r = _r;
		cylinderRcylinderR = r*r;

		A.x = _1;
		A.y = _2;
		A.z = _3;
		B.x = _4;
		B.y = _5;
		B.z = _6;

		BA = B - A;
		vX[0] = A;
		vX[1] = B;

		{
			// std make frame
			// trivial impl
			SimpleMath::Vector3 X(1, 0, 0);
			auto Y = BA;
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
	}
};
struct Line{
	SimpleMath::Vector3 A;
	SimpleMath::Vector3 B;
	SimpleMath::Vector3 BA;
	SimpleMath::Vector3 vX[2];
	Line(float _1, float _2, float _3, float _4, float _5, float _6)
	{
		A.x = _1;
		A.y = _2;
		A.z = _3;
		B.x = _4;
		B.y = _5;
		B.z = _6;
		
		BA = B - A;
		vX[0] = A;
		vX[1] = B;
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
float offsetBySmallDistantce(float t, float worldDistanceLength)
{
	t = t - 0.05f / worldDistanceLength;
	return t < 0.0f ? 0.0f : t;
}
bool collisionWithEdge(const Cylinder& cy, const Line& li, const SimpleMath::Vector3 & worldDistance, SimpleMath::Vector3 & impactPoint, SimpleMath::Vector3 & impactNormal, float & impactT)
{
	impactT = 1.0f;

	bool collision = false;

	auto worldDistanceWorldDistance = worldDistance.Dot(worldDistance);
	auto worldDistanceLength = sqrt(worldDistanceWorldDistance);

	SimpleMath::Vector2 projCylinderA = projectOnZ(SimpleMath::Vector3::Transform(cy.A, cy.ToCylinderLocalSpace));
	SimpleMath::Vector2 vProjLineX[] = { projectOnZ(SimpleMath::Vector3::Transform(li.A, cy.ToCylinderLocalSpace)), projectOnZ(SimpleMath::Vector3::Transform(li.B, cy.ToCylinderLocalSpace)) };
	SimpleMath::Vector2 projWorldDistance = projectOnZ(SimpleMath::Vector3::TransformNormal(worldDistance, cy.ToCylinderLocalSpace));
	SimpleMath::Vector2 projLineBA = projectOnZ(SimpleMath::Vector3::TransformNormal(li.BA, cy.ToCylinderLocalSpace));

	float last_t = 1.0f;

	//WITH EDGE(begin)////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//----Cylinder Ends Sphera's 
	{
		float t = 1.0f, a, b, c;

		bool rootFound = false;
		int lastRootFoundOnIndex;

		auto VA = worldDistance.Dot(li.BA);
		auto AA = li.BA.Dot(li.BA);
		auto VV = worldDistanceWorldDistance;

		a = VV - pow(VA, 2.0f) / AA;

		for (int i = 0; i < 2; i++)
		{
			auto S = cy.vX[i] - li.A;
			auto SA = S.Dot(li.BA);
			auto SS = S.Dot(S);
			auto SV = S.Dot(worldDistance);
			auto VA = worldDistance.Dot(li.BA);

			b = 2.0f*(SV - (SA*VA) / AA);
			c = SS - cy.cylinderRcylinderR - (SA*SA) / AA;

			if (getLowestRoot(a, b, c, t, &t))
			{
				rootFound = true;
				lastRootFoundOnIndex = i;
			}
		}

		if (rootFound)
		{
			auto cylinderX = cy.vX[lastRootFoundOnIndex];

			auto newCylinderX = cylinderX + t*worldDistance;

			auto l = (newCylinderX - li.A).Dot(li.BA) / li.BA.Dot(li.BA);

			if (.0f <= l&&l <= 1.0f)
			{
				auto lineX = li.A + l*li.BA;

				auto normalX = normalize(newCylinderX - lineX);

				if (lastRootFoundOnIndex == 0)
				{
					collision = cy.BA.Dot(normalX) >= .0f;
				}
				else
				{
					collision = cy.BA.Dot(normalX) <= .0f;
				}

				if (collision)
				{
					t = correctMistake(t, cy.r, vec4(normalX, -normalX.Dot(lineX)), cylinderX, newCylinderX);
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
			t = (cy.r * sign - lineDotS) / lineDotV;
			if (0.0f <= t&&t <= 1.0f)
			{
				newCylinderA = cy.A + t*worldDistance;

				skewLinesArgs = getDistanceBetweenSkewLines(newCylinderA, cy.BA, li.A, li.BA);

				//check only if impact possess line cut (with finit cylinder)
				if (.0f <= skewLinesArgs.x&&skewLinesArgs.x <= 1.0f && .0f <= skewLinesArgs.y&&skewLinesArgs.y <= 1.0f)
				{
					rootFound = true;
				}
			}
		}

		if (rootFound)
		{
			auto cylinderX = cy.A + skewLinesArgs.x * cy.BA;

			auto newCylinderX = newCylinderA + skewLinesArgs.x * cy.BA;

			auto lineX = li.A + skewLinesArgs.y * li.BA;

			auto normalX = normalize(newCylinderX - lineX);

			t = correctMistake(t, cy.r, vec4(normalX, -normalX.Dot(lineX)), cylinderX, newCylinderX);
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

	return collision;
}
bool collisionWithVertex(const Cylinder& cy, const SimpleMath::Vector3& ve, const SimpleMath::Vector3 & worldDistance, SimpleMath::Vector3 & impactPoint, SimpleMath::Vector3 & impactNormal, float & impactT)
{
	impactT = 1.0f;

	bool collision = false;

	auto worldDistanceWorldDistance = worldDistance.Dot(worldDistance);
	auto worldDistanceLength = sqrt(worldDistanceWorldDistance);

	SimpleMath::Vector2 projVe = projectOnZ(SimpleMath::Vector3::Transform(ve, cy.ToCylinderLocalSpace));

	SimpleMath::Vector2 projCylinderA = projectOnZ(SimpleMath::Vector3::Transform(cy.A, cy.ToCylinderLocalSpace));
	SimpleMath::Vector2 projWorldDistance = projectOnZ(SimpleMath::Vector3::TransformNormal(worldDistance, cy.ToCylinderLocalSpace));
	auto projWorldDistanceProjWorldDistance = projWorldDistance.Dot(projWorldDistance);

	float last_t = 1.0f;

	//WITH VERTEX(begin)//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//----Cylinder Ends Sphera's 
	//collide end spheres with line points (3d)
	{
		float t = 1, a, b, c;

		bool rootFound = false;
		int lastRootFoundOnIndex;

		a = worldDistanceWorldDistance;

		for (int i = 0; i < 2; i++)
		{
			auto d = cy.vX[i] - ve;

			b = 2.0f*worldDistance.Dot(d);
			c = d.Dot(d) - cy.cylinderRcylinderR;

			if (getLowestRoot(a, b, c, t, &t))
			{
				rootFound = true;
				lastRootFoundOnIndex = i;
			}
		}

		if (rootFound)
		{
			auto cylinderX = cy.vX[lastRootFoundOnIndex];

			auto newCylinderX = cylinderX + t*worldDistance;

			auto lineX = ve;

			auto normalX = normalize(newCylinderX - lineX);

			if (lastRootFoundOnIndex == 0)
			{
				collision = cy.BA.Dot(normalX) >= .0f;
			}
			else
			{
				collision = cy.BA.Dot(normalX) <= .0f;
			}

			if (collision)
			{
				t = correctMistake(t, cy.r, vec4(normalX, -normalX.Dot(lineX)), cylinderX, newCylinderX);
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

		{
			auto d = projCylinderA - projVe;

			b = 2.0f*projWorldDistance.Dot(d);
			c = d.Dot(d) - cy.cylinderRcylinderR;

			if (getLowestRoot(a, b, c, t, &t))
			{
				rootFound = true;
			}
		}

		if (rootFound)
		{
			auto newCylinderA = cy.A + t*worldDistance;

			auto lineX = ve;

			auto c = (lineX - newCylinderA).Dot(cy.BA) / cy.BA.Dot(cy.BA);

			if (.0f <= c&&c <= 1.0f)
			{
				auto cylinderX = cy.A + c * cy.BA;

				auto newCylinderX = newCylinderA + c * cy.BA;

				auto normalX = normalize(newCylinderX - lineX);

				t = correctMistake(t, cy.r, vec4(normalX, -normalX.Dot(lineX)), cylinderX, newCylinderX);
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
	impactT = 1.0f;

	bool _collison = false;
 
	SimpleMath::Vector3 _impactPoint;
	SimpleMath::Vector3 _impactNormal;
	float _t = 1.0f;

	bool lastCollison = false;

	SimpleMath::Vector3 lastImpactPoint;
	SimpleMath::Vector3 lastImpactNormal;
	float last_t;

	SimpleMath::Vector3 vFaceNormal[] = { boxBasis.Right(), boxBasis.Up(), boxBasis.Backward() };
	float vBoxHalfSize[] = { boxHalfSize.x, boxHalfSize.y, boxHalfSize.z };

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
				auto faceOrigin = boxBasis.Translation() + faceOffset * faceNormal;

				{
					lastCollison = false;
				}

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

					if (collisionWithFace(cylinder, faceBasis, faceHalfSize, worldDistance, lastImpactPoint, lastImpactNormal, last_t))
					{
						if (last_t < _t)
						{
							lastCollison = true;

							_collison = true;
							_impactPoint = lastImpactPoint;
							_impactNormal = faceNormal;
							_t = last_t;
						}
					}
				}

				if (!lastCollison)
				{
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

							if (collisionWithEdge(cylinder, line, worldDistance, lastImpactPoint, lastImpactNormal, last_t))
							{
								if (last_t < _t)
								{
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
									lastCollison = true;

									_collison = true;
									_impactPoint = lastImpactPoint;
									_impactNormal = edgeNormal;
									_t = last_t;
								}
							}
						}
					}
				}

				if (!lastCollison)
				{
					SimpleMath::Vector3 vertexNormal;

					auto baseLineA = faceOrigin + vFaceHalfSize[0] * vFaceTB[0];
					auto baseLineB = faceOrigin - vFaceHalfSize[0] * vFaceTB[0];

					auto lineOffset = vFaceHalfSize[1] * vFaceTB[1];
					
					for (int l = 0; l < 2; l++)
					{
						lineOffset *= -1.0f;

						SimpleMath::Vector3 lineX[] = { baseLineA + lineOffset, baseLineB + lineOffset };

						for (int k = 0; k < 2; k++)
						{
							if (collisionWithVertex(cylinder, lineX[k], worldDistance, lastImpactPoint, lastImpactNormal, last_t))
							{
								if (last_t < _t)
								{
									{
										auto midPoint = 0.5f*(lineX[0] + lineX[1]);
										auto tangent0 = midPoint - faceOrigin;
										tangent0.Normalize();
										auto tangent1 = lastImpactPoint - midPoint;
										tangent1.Normalize();
										vertexNormal = (faceNormal);// +tangent0 + tangent1);
										vertexNormal.Normalize();
									}
									lastCollison = true;

									_collison = true;
									_impactPoint = lastImpactPoint;
									_impactNormal = vertexNormal;
									_t = last_t;
								}
							}
						}
					}
				}
			}
		}
	}

	if (_collison)
	{
		impactPoint = _impactPoint;
		impactNormal = _impactNormal;
		impactT = _t;
	}

	return _collison;
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
