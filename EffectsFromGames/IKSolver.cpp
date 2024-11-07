#include "main.h"
#include "AnimationImpl.h"
#include <fstream>

#include <locale>
#include <codecvt>
#include <string>
#include <array>
#include <map>
#include <locale> 

extern char DebugBuffer[1024];
void Debug();

namespace{
	struct PosAccesor
	{
		JointSQT & j;
		PosAccesor(JointSQT & _j) :j(_j){};
		PosAccesor& operator = (const SimpleMath::Vector3 & v)
		{
			j.X[2].x = v.x;
			j.X[2].y = v.y;
			j.X[2].z = v.z;
			return *this;
		}
		operator SimpleMath::Vector3() const
		{
			return SimpleMath::Vector3(j.X[2].x, j.X[2].y, j.X[2].z);
		}
	};
	SimpleMath::Vector3 operator * (const float& a, const PosAccesor& b)
	{
		return a * SimpleMath::Vector3(b);
	}
	SimpleMath::Vector3 operator + (const PosAccesor& a, const PosAccesor& b)
	{
		return SimpleMath::Vector3(a) + b;
	}
	SimpleMath::Vector3 operator - (const PosAccesor& a, const PosAccesor& b)
	{
		return SimpleMath::Vector3(a) - b;
	}
	struct PosAccesor pos(JointSQT & j)
	{
		return PosAccesor(j);
	}

	struct OrientAccesor
	{
		JointSQT & j;
		OrientAccesor(JointSQT & _j) :j(_j){};
		OrientAccesor& operator = (const SimpleMath::Quaternion & v)
		{
			j.X[1] = v;
			return *this;
		}
		operator SimpleMath::Quaternion()
		{
			return SimpleMath::Quaternion(j.X[1]);
		}
		void Normalize()
		{
			auto Quat = SimpleMath::Quaternion(j.X[1]);
			Quat.Normalize();
			j.X[1] = Quat;
		}
	};
	SimpleMath::Quaternion inverse(const SimpleMath::Quaternion q)
	{
		SimpleMath::Quaternion res;
		q.Inverse(res);
		return res;
	}
	void quatToAxisAngle(const SimpleMath::Quaternion q, float & angle, SimpleMath::Vector3 & axis)
	{
		float Angle;
		XMVECTOR Axis;
		XMQuaternionToAxisAngle(&Axis, &Angle, q);
		angle = Angle;
		axis = Axis;
		axis.Normalize();
	}
	OrientAccesor orient(JointSQT & j)
	{
		return OrientAccesor(j);
	}

	SimpleMath::Vector3 normalize(SimpleMath::Vector3 v)
	{
		v.Normalize();
		return v;
	}
}

IKSolverInterface::~IKSolverInterface()
{
}

void JointHelpers::AnimToChain(const std::vector<int> & chain, AnimationBase * Anim, std::vector<JointSQT> & Joints)
{
	for (int i = 0; i < chain.size(); i++)
	{
		Joints[i] = Anim->CurrentJoints[chain[i]];
	}
}

void JointHelpers::ChainToAnim(const std::vector<int> & chain, std::vector<JointSQT> & Joints, AnimationBase * Anim)
{
	for (int i = 0; i < chain.size(); i++)
	{
		Anim->CurrentJoints[chain[i]] = Joints[i];
	}
}

void JointHelpers::localToModel(int Count, std::vector<JointSQT> & Joints)
{
	JointSQT & Joint = Joints[0];

	SimpleMath::Vector3 Position1 = pos(Joint);
	SimpleMath::Quaternion Orientation1 = orient(Joint);

	for (int i = 1; i < Count; i++)
	{
		JointSQT & Joint = Joints[i];

		SimpleMath::Vector3 Position2 = Position1 + SimpleMath::Vector3::Transform(pos(Joint), Orientation1);
		SimpleMath::Quaternion Orientation2 = SimpleMath::Quaternion::Concatenate(Orientation1, orient(Joint)); /// check order

		pos(Joint) = Position2;
		orient(Joint) = Orientation2;

		Position1 = Position2;
		Orientation1 = Orientation2;
	}
}

void JointHelpers::modelToLocal(int Count, std::vector<JointSQT> & Joints)
{
	JointSQT & Joint = Joints[0];

	SimpleMath::Vector3 Position1 = pos(Joint);
	SimpleMath::Quaternion Orientation1 = orient(Joint);

	for (int i = 1; i < Count; i++)
	{
		JointSQT & Joint = Joints[i];

		SimpleMath::Vector3 Position2 = pos(Joint);
		SimpleMath::Quaternion Orientation2 = orient(Joint);

		pos(Joint) = SimpleMath::Vector3::Transform(Position2 - Position1, inverse(Orientation1));
		orient(Joint) = SimpleMath::Quaternion::Concatenate(inverse(Orientation1), Orientation2); /// check order

		Position1 = Position2;
		Orientation1 = Orientation2;
	}
}

void GetRelativeJoint(JointSQT & Joint1, JointSQT & Joint2)
{
	SimpleMath::Vector3 Position1 = pos(Joint1);
	SimpleMath::Quaternion Orientation1 = orient(Joint1);

	SimpleMath::Vector3 Position2 = pos(Joint2);
	SimpleMath::Quaternion Orientation2 = orient(Joint2);

	pos(Joint2) = SimpleMath::Vector3::Transform(Position2 - Position1, inverse(Orientation1));
	orient(Joint2) = SimpleMath::Quaternion::Concatenate(inverse(Orientation1), Orientation2); /// check order
}

void JointHelpers::fixJointRotation(SimpleMath::Vector3 oldDir, SimpleMath::Vector3 newDir, JointSQT & Joint)
{
	oldDir.Normalize();
	newDir.Normalize();

	auto Axis = oldDir.Cross(newDir);

	if (XMVector3Equal(Axis, XMVectorZero()))
	{
		return;
	}

	auto Angle = atan2(Axis.Length(), oldDir.Dot(newDir));

	Axis.Normalize();

	//sprintf(DebugBuffer, "fixJointRotation [%f] %f %f %f %f %f %f\n", Angle, oldDir.x, oldDir.y, oldDir.z, newDir.x, newDir.y, newDir.z); Debug();

	auto DeltaRotation = SimpleMath::Quaternion::CreateFromAxisAngle(Axis, Angle);

	DirectX::XMMATRIX JointFrame = SimpleMath::Matrix::CreateFromQuaternion(orient(Joint));

	JointFrame.r[0] = SimpleMath::Vector3::Transform(JointFrame.r[0], DeltaRotation);
	JointFrame.r[1] = SimpleMath::Vector3::Transform(JointFrame.r[1], DeltaRotation);
	JointFrame.r[2] = SimpleMath::Vector3::Transform(JointFrame.r[2], DeltaRotation);

	orient(Joint) = SimpleMath::Quaternion::CreateFromRotationMatrix(JointFrame);
}

SimpleMath::Vector3 JointHelpers::transformFromFirstToLastJointFrame(const std::vector<int> & indexChain, AnimationBase * anim, const SimpleMath::Vector3 & position)
{
	static std::vector<JointSQT> chain;
	static bool init = false;

	if (!init)
	{
		chain.resize(1024);
		init = true;
	}

	AnimToChain(indexChain, anim, chain);
	localToModel(indexChain.size(), chain);

	return SimpleMath::Vector3::Transform(position, chain[indexChain.size() - 1].matrix().Invert());
}

struct FABRIK : public IKSolverInterface
{
	int chain0Size;
	float sum_distantce0;
	std::vector<float> distantce0;
	std::vector<JointSQT> chain0;

	~FABRIK(){
	}

	void init(Character * character)
	{
		chain0Size = 0;
		chain0.resize(1024);
		distantce0.resize(1024);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void getDistantce0()
	{
		sum_distantce0 = 0.0f;

		JointSQT & Joint = chain0[0];

		SimpleMath::Vector3 Position1 = pos(Joint);

		for (int i = 1; i < chain0Size; i++)
		{
			JointSQT & Joint = chain0[i];

			SimpleMath::Vector3 Position2 = pos(Joint);

			float Dist = (Position2 - Position1).Length();
			distantce0[i-1] = Dist;
			sum_distantce0 += Dist;

			Position1 = Position2;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	std::vector<JointSQT>& chainRef(int chain)
	{
		return chain0;
	}

	void setChainSize(int chain, int size)
	{
		chain0Size = size;
	}

	void solve(int ChainSize, SimpleMath::Vector3 target, int total_iter, float epsilon)
	{
		chain0Size = ChainSize;

		getDistantce0();

		////////////////////////////////////////////////////////////////////////////////////////////////////

		const int chainN = chain0Size - 1;

		auto & Joint0 = chain0[0];
		auto & JointN = chain0[chainN];

		SimpleMath::Vector3 base = pos(Joint0);

		float distantceToTarget = (target - base).Length();

		auto diff = (pos(JointN) - target).Length();

		////////////////////////////////////////////////////////////////////////////////////////////////////

		//check distance between root and target
		if (distantceToTarget > sum_distantce0)
		{
			//sprintf(DebugBuffer, "IK Solver target is unreachable \n"); Debug();

			SimpleMath::Vector3 prev = pos(Joint0);

			for (int i = 0; i < chainN; i++)
			{
				auto & Joint1 = chain0[i];
				auto & Joint2 = chain0[i + 1];
				auto & Dist = distantce0[i];

				auto oldOrientDir = pos(Joint2) - prev;
				auto newOrientDir = target - pos(Joint1);
				JointHelpers::fixJointRotation(oldOrientDir, newOrientDir, Joint1);

				auto lambda = Dist / newOrientDir.Length();
				auto newPos = (1.0f - lambda) * pos(Joint1) + lambda * target;
				
				prev = pos(Joint2);
				pos(Joint2) = newPos;
			}
		}

		if (distantceToTarget <= sum_distantce0)
		{
			//sprintf(DebugBuffer, "IK Solver target is reachable \n"); Debug();

			SimpleMath::Vector3 prev;

			for (int i = 0; diff > epsilon && i < total_iter; i++)
			{
				//forward
				prev = pos(JointN);
				pos(JointN) = target;

				for (int i = chainN - 1; i >= 0; i--)
				{
					auto & Joint1 = chain0[i];
					auto & Joint2 = chain0[i + 1];
					auto & Dist = distantce0[i];

					auto oldOrientDir = prev - pos(Joint1);
					auto newOrientDir = pos(Joint2) - pos(Joint1);
					JointHelpers::fixJointRotation(oldOrientDir, newOrientDir, Joint1);

					auto lambda = Dist / newOrientDir.Length();
					auto newPos = (1.0f - lambda) * pos(Joint2) + lambda * pos(Joint1);

					prev = pos(Joint1);
					pos(Joint1) = newPos;
				}

				//backward
				prev = pos(Joint0);
				pos(Joint0) = base;

				for (int i = 0; i < chainN; i++)
				{
					auto & Joint1 = chain0[i];
					auto & Joint2 = chain0[i + 1];
					auto & Dist = distantce0[i];

					auto oldOrientDir = pos(Joint2) - prev;
					auto newOrientDir = pos(Joint2) - pos(Joint1);
					JointHelpers::fixJointRotation(oldOrientDir, newOrientDir, Joint1);

					auto lambda = Dist / newOrientDir.Length();
					auto newPos = (1.0f - lambda) * pos(Joint1) + lambda * pos(Joint2);

					prev = pos(Joint2);
					pos(Joint2) = newPos;
				}

				diff = (pos(JointN) - target).Length();
			}
		}
	}

};


IKSolverInterface* makeIKSolver(Character * character)
{
	auto ret = new FABRIK();
	ret->init(character);
	return ret;
}