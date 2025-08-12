#include "main.h"
#include "AnimationImpl.h"
#include <fstream>

#include <locale>
#include <codecvt>
#include <string>
#include <array>
#include <map>
#include <locale> 

extern bool	EquipPistols;
extern class AnimationBase* DebugAnimation;
extern JointsRefsChainCollection jointsRefsChains;
extern IAnimationGraph2 * EveAnimationGraph;
#undef min // use __min instead
#undef max // use __max instead

void attachSocket(Character* CharacterObject, char* socketName, char* destFrameName);

extern Character * Eve;

namespace
{
	bool _equipPistols = false;
	float _time = 0.f;
	float _timeToBlend = 0.2f;
	bool  _isPlaying = false;

	std::vector<int> GetHandsJointsIndex()
	{
		std::vector<int> Joints;
		Joints.insert(Joints.end(), jointsRefsChains[0].end() - 3, jointsRefsChains[0].end());
		for (int i = 0; i < 2; i++)
		{
			Joints.insert(Joints.end(), jointsRefsChains[i].end() - 3, jointsRefsChains[i].end());
			for (int j = 0; j < 5; j++)
			{
				Joints.insert(Joints.end(), jointsRefsChains.Hand_Childs[i][j].begin(), jointsRefsChains.Hand_Childs[i][j].end());
			}
		}
		return Joints;
	}
};

void CombineAnimation(float deltaTime)
{
	if (_equipPistols != EquipPistols)
	{
		_isPlaying = true;
		_equipPistols = EquipPistols;
		((Animation*)DebugAnimation)->setRate(_equipPistols?1.5:-1.5);
		DebugAnimation->setPlaying(true);
	}

	//auto _time = ((Animation*)DebugAnimation)->getLocTime();
	auto rate = ((Animation*)DebugAnimation)->getRate();
	if (_isPlaying)
	{
		auto _new_time = _time + rate*deltaTime;

		auto t = _time / _timeToBlend;
		t = std::max(0.f, std::min(1.f, t));

		if (t < 1.f)
		{
			DebugAnimation->sample(0.f);
		}
		else
		{
			SimpleMath::Vector3 deltaTranslation;
			SimpleMath::Quaternion deltaRotation;
			DebugAnimation->advanse(deltaTime, deltaTranslation, deltaRotation);
		}

		static std::vector<int> JointsIndex = GetHandsJointsIndex();
		auto DestAnimation = EveAnimationGraph->getPlayingAnimation();
		auto SrcAnimation = DebugAnimation;

		for (int i = 0; i < JointsIndex.size(); i++)
		{
			auto & joint1 = DestAnimation->CurrentJoints[JointsIndex[i]];
			auto & joint2 = SrcAnimation->CurrentJoints[JointsIndex[i]];

			auto & joint = joint1;

			joint[0] = SimpleMath::Vector4::Lerp(joint1[0], joint2[0], t);
			joint[1] = SimpleMath::Quaternion::Lerp(joint1[1], joint2[1], t);
			joint[2] = SimpleMath::Vector4::Lerp(joint1[2], joint2[2], t);
		}

		if (0.f < rate && _time <= _timeToBlend && _timeToBlend <= _new_time)
		{
			attachSocket(Eve, "LeftGun", "LeftHand");
			attachSocket(Eve, "RightGun", "RightHand");
		}
		else if (rate < 0.f && _new_time <= _timeToBlend && _timeToBlend <= _time)
		{
			attachSocket(Eve, "LeftGun", "LeftLegHolster");
			attachSocket(Eve, "RightGun", "RightLegHolster");
		}

		_time = _new_time;
		_time = std::max(0.f, std::min(1.f + _timeToBlend, _time));
	}
};

