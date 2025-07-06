#include "main.h"
#include "AnimationImpl.h"
#include <fstream>

#include <locale>
#include <codecvt>
#include <string>
#include <array>
#include <map>
#include <locale> 

std::vector<JointSQT>& __AnimGetJointsByTime(AnimationBase* Anim, float Time);

extern JointsRefsChainCollection jointsRefsChains;

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

void JointHelpers::ChainToAnim(int StartIndex, int FinishIndex, const std::vector<int> & chain, std::vector<JointSQT> & Joints, AnimationBase * Anim)
{
	for (int i = StartIndex; i < FinishIndex + 1; i++)
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

void JointHelpers::localToModel(int StartIndex, int FinishIndex, std::vector<JointSQT> & Joints)
{
	JointSQT & Joint = Joints[StartIndex];

	SimpleMath::Vector3 Position1 = pos(Joint);
	SimpleMath::Quaternion Orientation1 = orient(Joint);

	for (int i = StartIndex + 1; i < FinishIndex + 1; i++)
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

void JointHelpers::modelToLocal(int StartIndex, int FinishIndex, std::vector<JointSQT> & Joints)
{
	JointSQT & Joint = Joints[StartIndex];

	SimpleMath::Vector3 Position1 = pos(Joint);
	SimpleMath::Quaternion Orientation1 = orient(Joint);

	for (int i = StartIndex + 1; i < FinishIndex + 1; i++)
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

void GetAbsoluteJoint(JointSQT & Joint1, JointSQT & Joint2)
{
	SimpleMath::Vector3 Position1 = pos(Joint1);
	SimpleMath::Quaternion Orientation1 = orient(Joint1);

	SimpleMath::Vector3 Position2 = pos(Joint2);
	SimpleMath::Quaternion Orientation2 = orient(Joint2);

	pos(Joint2) = Position1 + SimpleMath::Vector3::Transform(Position2, Orientation1);
	orient(Joint2) = SimpleMath::Quaternion::Concatenate(Orientation1, Orientation2); /// check order
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
	float sum_distantce0;
	std::vector<std::vector<float>> distantces;
	std::vector<std::vector<JointSQT>> chains;

	~FABRIK(){
	}

	void init(Character * character)
	{
		chains.resize(4);
		chains[0].resize(1024);
		chains[1].resize(1024);
		chains[2].resize(1024);
		chains[3].resize(1024);

		distantces.resize(4);
		distantces[0].resize(1024);
		distantces[1].resize(1024);
		distantces[2].resize(1024);
		distantces[3].resize(1024);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void getDistantce(int index, int ChainStart, int ChainFinish)
	{
		sum_distantce0 = 0.0f;

		auto& chain = chains[index];
		auto& distantce = distantces[index];

		JointSQT & Joint = chain[ChainStart];

		SimpleMath::Vector3 Position1 = pos(Joint);

		for (int i = ChainStart + 1; i < ChainFinish + 1; i++)
		{
			JointSQT & Joint = chain[i];

			SimpleMath::Vector3 Position2 = pos(Joint);

			float Dist = (Position2 - Position1).Length();
			distantce[i - 1] = Dist;
			sum_distantce0 += Dist;

			Position1 = Position2;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	std::vector<JointSQT>& chainRef(int chain)
	{
		return chains[chain];
	}

	//void setChainSize(int chain, int size)
	//{
	//	chain0Size = size;
	//}

	void solve(int index, int ChainStart, int ChainFinish, SimpleMath::Vector3 target, int total_iter, float epsilon)
	{
		getDistantce(index, ChainStart, ChainFinish);

		////////////////////////////////////////////////////////////////////////////////////////////////////

		auto& chain = chains[index];
		auto& distantce = distantces[index];

		////////////////////////////////////////////////////////////////////////////////////////////////////

		auto & Joint0 = chain[ChainStart];
		auto & JointN = chain[ChainFinish];

		SimpleMath::Vector3 base = pos(Joint0);

		float distantceToTarget = (target - base).Length();

		auto diff = (pos(JointN) - target).Length();

		////////////////////////////////////////////////////////////////////////////////////////////////////

		//check distance between root and target
		if (distantceToTarget > sum_distantce0)
		{
			//sprintf(DebugBuffer, "IK Solver target is unreachable \n"); Debug();

			SimpleMath::Vector3 prev = pos(Joint0);

			for (int i = ChainStart; i < ChainFinish; i++)
			{
				auto & Joint1 = chain[i];
				auto & Joint2 = chain[i + 1];
				auto & Dist = distantce[i];

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

				for (int i = ChainFinish - 1; i >= ChainStart; i--)
				{
					auto & Joint1 = chain[i];
					auto & Joint2 = chain[i + 1];
					auto & Dist = distantce[i];

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

				for (int i = ChainStart; i < ChainFinish; i++)
				{
					auto & Joint1 = chain[i];
					auto & Joint2 = chain[i + 1];
					auto & Dist = distantce[i];

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

std::vector<JointSQT> GetChainsJoints_MS(Animation* Anim, const std::vector<int>& Chain)
{
	std::vector<JointSQT> ChainsJoints;
	ChainsJoints.resize(256);

	auto Joints = ::__AnimGetJointsByTime(Anim, .0f);
	for (int i = 0; i < Chain.size(); i++){ ChainsJoints[i] = Joints[Chain[i]]; }
	JointHelpers::localToModel(Chain.size(), ChainsJoints);

	return ChainsJoints;
}

std::vector<JointSQT> GetChainsJoints_LS(Animation* Anim, const std::vector<int>& Chain)
{
	std::vector<JointSQT> ChainsJoints;
	ChainsJoints.resize(256);

	auto Joints = ::__AnimGetJointsByTime(Anim, .0f);
	for (int i = 0; i < Chain.size(); i++){ ChainsJoints[i] = Joints[Chain[i]]; }

	return ChainsJoints;
}

SimpleMath::Quaternion RotateHipsJoint(SimpleMath::Vector3 oldDir, SimpleMath::Vector3 newDir, JointSQT& Joint)
{
	oldDir.Normalize();
	newDir.Normalize();

	auto Axis = oldDir.Cross(newDir);

	if (XMVector3Equal(Axis, XMVectorZero()))
	{
		return orient(Joint);
	}

	auto Angle = atan2(Axis.Length(), oldDir.Dot(newDir));

	Axis.Normalize();

	return SimpleMath::Quaternion::Concatenate(SimpleMath::Quaternion::CreateFromAxisAngle(Axis, Angle), orient(Joint));
}

void AnimTransformJointSamples(Animation* Anim, int JointIndex, JointSQT& J)
{
	Anim->TransformJointSamples(JointIndex, "rotation", [&J](SimpleMath::Vector4 v){ return J.X[1]; });
	Anim->TransformJointSamples(JointIndex, "translation", [&J](SimpleMath::Vector4 v){ return J.X[2]; });
}

void FixAnimChainsJointsOrientation(Animation* Anim, Animation* RefAnim, const std::vector<int>& Chain, int LowBound, int HighBound)
{
	std::vector<JointSQT> ChainsJoints = GetChainsJoints_MS(Anim, Chain);
	std::vector<JointSQT> RefChainsJoints = GetChainsJoints_MS(RefAnim, Chain);

	for (int i = LowBound; i < HighBound; i++)
	{
		auto & RefJoint1 = RefChainsJoints[i];
		auto & RefJoint2 = RefChainsJoints[i + 1];

		auto & Joint1 = ChainsJoints[i];
		auto & Joint2 = ChainsJoints[i + 1];

		auto oldDir = pos(RefJoint2) - pos(RefJoint1);
		auto newDir = pos(Joint2) - pos(Joint1);

		{
			orient(Joint1) = RotateHipsJoint(oldDir, newDir, RefJoint1);
			/*
			oldDir.Normalize();
			newDir.Normalize();

			auto Axis = oldDir.Cross(newDir);

			if (XMVector3Equal(Axis, XMVectorZero()))
			{
				continue;
			}

			auto Angle = atan2(Axis.Length(), oldDir.Dot(newDir));

			Axis.Normalize();

			auto DeltaRotation = SimpleMath::Quaternion::CreateFromAxisAngle(Axis, Angle);

			orient(Joint1) = SimpleMath::Quaternion::Concatenate(DeltaRotation, orient(RefJoint1));
			*/
		}
	}

	JointHelpers::modelToLocal(Chain.size(), ChainsJoints);

	for (int i = 0; i < Chain.size(); i++)
	{
		AnimTransformJointSamples(Anim, Chain[i], ChainsJoints[i]);
		/*Anim->TransformJointSamples(
			Chain[i],
			"rotation",
			[&ChainsJoints, i](SimpleMath::Vector4 v){
				return orient(ChainsJoints[i]);
			}
		);
		Anim->TransformJointSamples(
			Chain[i],
			"translation",
			[&ChainsJoints, i](SimpleMath::Vector4 v){
				SimpleMath::Vector3 v_out = pos(ChainsJoints[i]);
				return SimpleMath::Vector4(v_out.x, v_out.y, v_out.z, 1.f);
			}
		);
		*/
	}
}

void FixAnimChainsHipsJointOrientation(Animation* Anim, Animation* RefAnim)
{
	/*
	std::vector<JointSQT> ChainsJoints = GetChainsJoints_MS(Anim, jointsRefsChains.HipsHeadTop_End);
	std::vector<JointSQT> RefChainsJoints = GetChainsJoints_MS(RefAnim, jointsRefsChains.HipsHeadTop_End);

	auto & RefJointHips = RefChainsJoints[0];
	auto & RefJointSpine = RefChainsJoints[1];

	auto & JointHips = ChainsJoints[0];
	auto & JointSpine = ChainsJoints[1];

	orient(JointHips) = orient(RefJointHips);
	orient(JointSpine) = orient(RefJointSpine);

	JointHelpers::modelToLocal(jointsRefsChains.HipsHeadTop_End.size(), ChainsJoints);

	{
		auto index = jointsRefsChains.HipsHeadTop_End[0];
		Anim->TransformJointSamples(
			jointsRefsChains.HipsHeadTop_End[0],
			"rotation",
			[&ChainsJoints, index](SimpleMath::Vector4 v){
				return orient(ChainsJoints[index]);
			}
		);
	}
	{
		auto index = jointsRefsChains.HipsHeadTop_End[1];
		Anim->TransformJointSamples(
			jointsRefsChains.HipsHeadTop_End[1],
			"rotation",
			[&ChainsJoints, index](SimpleMath::Vector4 v){
				return orient(ChainsJoints[index]);
			}
		);
	}
	*/

	std::vector<JointSQT> ChainsJoints = GetChainsJoints_LS(Anim, jointsRefsChains.HipsHeadTop_End);
	std::vector<JointSQT> ChainsJointsLeft = GetChainsJoints_LS(Anim, jointsRefsChains.HipsLeftToe_End);
	std::vector<JointSQT> ChainsJointsRight = GetChainsJoints_LS(Anim, jointsRefsChains.HipsRightToe_End);

	std::vector<JointSQT> RefChainsJoints = GetChainsJoints_LS(RefAnim, jointsRefsChains.HipsHeadTop_End);

	GetAbsoluteJoint(ChainsJoints[0], ChainsJoints[1]);
	GetAbsoluteJoint(ChainsJointsLeft[0], ChainsJointsLeft[1]);
	GetAbsoluteJoint(ChainsJointsRight[0], ChainsJointsRight[1]);

	GetAbsoluteJoint(RefChainsJoints[0], RefChainsJoints[1]);

	auto HipsJoint = RefChainsJoints[0];
	orient(HipsJoint) = RotateHipsJoint(pos(RefChainsJoints[1]) - pos(RefChainsJoints[0]), pos(ChainsJoints[1]) - pos(ChainsJoints[0]), HipsJoint);

	ChainsJoints[0] = ChainsJointsLeft[0] = ChainsJointsRight[0] = HipsJoint;

	GetRelativeJoint(ChainsJoints[0], ChainsJoints[1]);
	GetRelativeJoint(ChainsJointsLeft[0], ChainsJointsLeft[1]);
	GetRelativeJoint(ChainsJointsRight[0], ChainsJointsRight[1]);

	AnimTransformJointSamples(Anim, jointsRefsChains.HipsHeadTop_End[1], ChainsJoints[1]);

	AnimTransformJointSamples(Anim, jointsRefsChains.HipsLeftToe_End[1], ChainsJointsLeft[1]);
	AnimTransformJointSamples(Anim, jointsRefsChains.HipsRightToe_End[1], ChainsJointsRight[1]);

	AnimTransformJointSamples(Anim, jointsRefsChains.HipsHeadTop_End[0], ChainsJoints[0]);
}

void FixAnimChainsHandChildJoinstOrientation(Animation* Anim, Animation* RefAnim, int HandIndex)
{
	JointSQT HandJoint;

	auto jointsRefsChainsHipsHand = HandIndex == 0 ? jointsRefsChains.HipsLeftHand : jointsRefsChains.HipsRightHand;

	int HandJointIndex = jointsRefsChainsHipsHand.size() - 1;
	int FirstHandChildJointIndex = jointsRefsChainsHipsHand.size();

	auto Hips_Middle_Finger_Chain = jointsRefsChainsHipsHand;
	Hips_Middle_Finger_Chain.insert(Hips_Middle_Finger_Chain.end(), jointsRefsChains.Hand_Childs[HandIndex][2].begin(), jointsRefsChains.Hand_Childs[HandIndex][2].end());

	{
		std::vector<JointSQT> ChainsJoints = GetChainsJoints_MS(Anim, Hips_Middle_Finger_Chain);
		std::vector<JointSQT> RefChainsJoints = GetChainsJoints_MS(RefAnim, Hips_Middle_Finger_Chain);

		HandJoint = RefChainsJoints[HandJointIndex];
		orient(HandJoint) = RotateHipsJoint(pos(RefChainsJoints[FirstHandChildJointIndex]) - pos(RefChainsJoints[HandJointIndex]), pos(ChainsJoints[FirstHandChildJointIndex]) - pos(ChainsJoints[HandJointIndex]), HandJoint);
	}

	for (int i = 0; i < 5; i++)
	{
		auto Hips_Finger_Chain = jointsRefsChainsHipsHand;
		Hips_Finger_Chain.insert(Hips_Finger_Chain.end(), jointsRefsChains.Hand_Childs[HandIndex][i].begin(), jointsRefsChains.Hand_Childs[HandIndex][i].end());

		std::vector<JointSQT> ChainsJoints = GetChainsJoints_MS(Anim, Hips_Finger_Chain);
		std::vector<JointSQT> RefChainsJoints = GetChainsJoints_MS(RefAnim, Hips_Finger_Chain);

		orient(ChainsJoints[HandJointIndex]) = orient(HandJoint);
		for (int j = FirstHandChildJointIndex; j < Hips_Finger_Chain.size() - 1; j++)
		{
			auto & RefJoint1 = RefChainsJoints[j];
			auto & RefJoint2 = RefChainsJoints[j + 1];

			auto & Joint1 = ChainsJoints[j];
			auto & Joint2 = ChainsJoints[j + 1];

			auto oldDir = pos(RefJoint2) - pos(RefJoint1);
			auto newDir = pos(Joint2) - pos(Joint1);

			orient(Joint1) = RotateHipsJoint(oldDir, newDir, RefJoint1);
		}

		JointHelpers::modelToLocal(Hips_Finger_Chain.size(), ChainsJoints);
		for (int k = FirstHandChildJointIndex; k < Hips_Finger_Chain.size() - 1; k++)
		{
			AnimTransformJointSamples(Anim, Hips_Finger_Chain[k], ChainsJoints[k]);
		}
	}

	{
		std::vector<JointSQT> ChainsJoints = GetChainsJoints_MS(Anim, Hips_Middle_Finger_Chain);

		orient(ChainsJoints[HandJointIndex]) = orient(HandJoint);

		JointHelpers::modelToLocal(Hips_Middle_Finger_Chain.size(), ChainsJoints);
		AnimTransformJointSamples(Anim, Hips_Middle_Finger_Chain[HandJointIndex], ChainsJoints[HandJointIndex]);
	}
}

void FixAnimJointsOrientation(Animation* Anim, Animation* RefAnim)
{

	FixAnimChainsJointsOrientation(Anim, RefAnim, jointsRefsChains.HipsRightHand, jointsRefsChains.HipsRightHand.size() - 3, jointsRefsChains.HipsRightHand.size() - 1);
	FixAnimChainsJointsOrientation(Anim, RefAnim, jointsRefsChains.HipsLeftHand, jointsRefsChains.HipsLeftHand.size() - 3, jointsRefsChains.HipsLeftHand.size() - 1);

	FixAnimChainsJointsOrientation(Anim, RefAnim, jointsRefsChains.HipsRightToe_End, jointsRefsChains.HipsRightToe_End.size() - 5, jointsRefsChains.HipsRightToe_End.size() - 1);
	FixAnimChainsJointsOrientation(Anim, RefAnim, jointsRefsChains.HipsLeftToe_End, jointsRefsChains.HipsLeftToe_End.size() - 5, jointsRefsChains.HipsLeftToe_End.size() - 1);

	FixAnimChainsJointsOrientation(Anim, RefAnim, jointsRefsChains.HipsHeadTop_End, 1, jointsRefsChains.HipsHeadTop_End.size() - 1);

	FixAnimChainsHipsJointOrientation(Anim, RefAnim);

	FixAnimChainsHandChildJoinstOrientation(Anim, RefAnim, 0);
	FixAnimChainsHandChildJoinstOrientation(Anim, RefAnim, 1);
}