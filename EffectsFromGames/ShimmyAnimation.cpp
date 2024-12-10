#include "main.h"
#include "AnimationImpl.h"
#include <fstream>

#include <locale>
#include <codecvt>
#include <string>
#include <array>
#include <map>
#include <locale> 

extern JointsRefsChainCollection jointsRefsChains;
extern char* state_hanging_Hand_Name;
extern SimpleMath::Vector3 state_hanging_Hand_Location;

extern std::string state_hanging_Ledge_Name;
extern Box state_hanging_Ledge_Box;
extern int state_hanging_Ledge_BoxIndex;
extern IAnimationGraph2 * EveAnimationGraph;
extern bool state_movement_is_obstructed;

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

Animation* loadAnimation(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex, char * replace = nullptr);
void extractAnimationMeta(Animation * anim, bool extractHeight, double duration, std::function<SimpleMath::Matrix * __cdecl(unsigned int index)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTransformations);
extern SimpleMath::Vector3 gravitation;
extern World GWorld;

extern IClimbingPathHelper* ClimbingPathHelper;

extern char DebugBuffer[1024];
void Debug();

extern SimpleMath::Vector3 RightHandHangOnLocation;
extern SimpleMath::Vector3 LeftHandHangOnLocation;

SimpleMath::Vector3 PredictHandHangLocation;

struct ShimmyAnimation : public Animation
{
	std::vector<SimpleMath::Vector3> Path;
	HandClimbingState LeftHandState;
	HandClimbingState RightHandState;

	SimpleMath::Vector3 LeftHandLocation;
	SimpleMath::Vector3 RightHandLocation;

	//SimpleMath::Vector3 LeftHandTarget;
	//SimpleMath::Vector3 RightHandTarget;

	bool PhaseLeftHand;
	bool PhaseRightHand;

	int PoseIdex;
	int PoseMaxIndex;
	double PoseTime;
	double PoseMaxTime;
	std::vector<JointSQT> PhasePoses[1000];
	double PhasePosesTimes[1000];

	AnimationRep* Impl;
	Animation* Shimmy;
	Animation* BracedHangShimmy;
	Animation* CurrentAnimation;
	int ShimmyKind;
	float ShimmyMoveRightCatchLedgeTime;
	float ShimmyMoveLeftCatchLedgeTime;
	//Animation* PreviousAnimation;
	std::function<void __cdecl(bool state)> onPlayingChanged;

	int MasterHandIndex;
	int SlaveHandIndex;

	Animation* Corner_Inside_Hanging;
	Animation* Corner_Outside_Hanging;

	float AnimationTime;
	float BlendK;

	SimpleMath::Vector3 HoldHandLocationWhileBlend;
	SimpleMath::Vector3 SlaveHandLocationWhileBlend;

	std::vector<JointSQT> ChainsJoints;

	std::vector<JointSQT> PreviousJoints;

	int BlendHandIndex;
	//bool PhaseStarted;
	//bool PhaseInProgress;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//1 Персонаж видет поворот кромки платформы зарание и следовательно может зарание скорректировать куда поставить руку
	//2 Есть две фазы движения, их назовем одноименно (так же как имена джойнов рук)
	//3 Огибание угла будет состоять из 4 ключивых и 2 вспомагательных поз
	//4 На сколько правая рука движется на столько левая догоняет
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void GeneratePoses()
	{
	}
	void LerpPoses(double k)
	{
	}
	bool IsPhaseEnd()
	{
		//1 Две стратегии по времени или по оверлапу
		//Выбираем по времени
	}
	void StartNewPhase()
	{
		//static HandClimbingPath::HandState HandState;
		std::swap(PhaseLeftHand, PhaseRightHand);

		if (PhaseLeftHand)
		{
			LeftHandState.AdvanseHand(LeftHandLocation, RightHandState);// , PhaseLeftHand ? LeftHandTarget : RightHandTarget, HandState);
		}
		if (PhaseRightHand)
		{
			RightHandState.AdvanseHand(RightHandLocation, LeftHandState);// , PhaseLeftHand ? LeftHandTarget : RightHandTarget, HandState);
		}

		//чекним и перегенерим позы при необходимости
		//if (!IsPosesValid())
		//{
		GeneratePoses();
		//}
		//if (PhaseStarted)
		//{
		//	PhaseInProgress = true;
		//	PhaseStarted = false;
		//}
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	ShimmyAnimation(bool MoveRight, std::map<std::string, unsigned int> & FramesNamesIndex, std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans)
		:LeftHandState(Path), RightHandState(Path)
	{
		Impl = new AnimationRep();

		Shimmy = loadAnimation("Media\\Animations\\Edge\\Shimmy.dae", FramesNamesIndex);
		Shimmy->setLooping(true);
		Shimmy->setRate(MoveRight ? 1.0f : -1.0f);
		extractAnimationMeta(Shimmy, true, 1.0f, getSkeletMatrix, calculateFramesTrans);

		BracedHangShimmy = loadAnimation("Media\\Animations\\Edge\\BracedHangShimmy.dae", FramesNamesIndex);
		BracedHangShimmy->setLooping(true);
		BracedHangShimmy->setRate(MoveRight ? 1.0f : -1.0f);
		extractAnimationMeta(BracedHangShimmy, true, 1.0f, getSkeletMatrix, calculateFramesTrans);

		auto LoadAnimation1 = [](std::map<std::string, unsigned int> & FramesNamesIndex, std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans, bool MoveRight, bool IsLooping, char * AnimationPath)
		{
			void rotateHips(Animation *, SimpleMath::Quaternion);
			Animation* loadAnimationFromUnreal(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex);

			auto Animation = loadAnimationFromUnreal(AnimationPath, FramesNamesIndex);
			////
			if(!MoveRight)
			{
				rotateHips(Animation, SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix::CreateRotationY(-90.0*PI / 180.0)));
			}
			////
			Animation->setLooping(IsLooping);
			Animation->setRate(MoveRight ? 1.f : -1.f);
			extractAnimationMeta(Animation, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
			return Animation;
		};
		auto LoadAnimation2 = [](std::map<std::string, unsigned int> & FramesNamesIndex, std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans, bool MoveRight, bool IsLooping, char * AnimationPath)
		{
			void rotateHips(Animation *, SimpleMath::Quaternion);
			Animation* loadAnimationFromUnreal(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex);

			auto Animation = loadAnimationFromUnreal(AnimationPath, FramesNamesIndex);
			////
			if (!MoveRight)
			{
				rotateHips(Animation, SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix::CreateRotationY(90.0*PI / 180.0)));
			}
			////
			Animation->setLooping(IsLooping);
			Animation->setRate(MoveRight ? 1.f : -1.f);
			extractAnimationMeta(Animation, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
			return Animation;
		};
		Corner_Inside_Hanging = LoadAnimation1(FramesNamesIndex, getSkeletMatrix, calculateFramesTrans, MoveRight, false, "Media\\Animations\\Edge\\Climb_Walk_Corner_Inside_Hanging_R1.FBX");
		Corner_Outside_Hanging = LoadAnimation2(FramesNamesIndex, getSkeletMatrix, calculateFramesTrans, MoveRight, false, "Media\\Animations\\Edge\\Climb_Walk_Corner_Outside_Hanging_R2.FBX");

		MasterHandIndex = 0;
		SlaveHandIndex = 1;

		if (MoveRight)
		{
			std::swap(MasterHandIndex, SlaveHandIndex);
		}
		
		CurrentMetaChannels = Shimmy->CurrentMetaChannels;
		CurrentJoints = Shimmy->CurrentJoints;

		ChainsJoints.resize(256);

		PreviousJoints = Shimmy->CurrentJoints;

		ShimmyKind = 1;
		ShimmyMoveRightCatchLedgeTime = 0.4;
		ShimmyMoveLeftCatchLedgeTime = 0.64;
	}
	~ShimmyAnimation()
	{
		delete Impl;

		delete Shimmy;

		delete BracedHangShimmy;

		delete Corner_Inside_Hanging;

		delete Corner_Outside_Hanging;
	}

	Animation* ChooseShimmy()
	{
		Animation* NewAnimation = Shimmy;
		ShimmyKind = 1;
		ShimmyMoveRightCatchLedgeTime = 0.4;
		ShimmyMoveLeftCatchLedgeTime = 0.64;

		if (state_movement_is_obstructed)
		{
			NewAnimation = BracedHangShimmy;
			ShimmyKind = 2;
			ShimmyMoveRightCatchLedgeTime = 10.f / 35.f;
			ShimmyMoveLeftCatchLedgeTime = 10.f / 35.f;
		}

		return NewAnimation;
	}

	void advanse(double elapsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation)
	{
		SimpleMath::Vector3 ModelSpaceDeltaDeltaTranslation = SimpleMath::Vector3::Zero;

		Impl->prev_frameNo = Impl->frameNo;

		auto modelTransform = ClimbingPathHelper->GetEveModelMatrix();

		if (CurrentAnimation == Corner_Outside_Hanging && BlendK < 1.f)
		{
			CurrentAnimation->sample(CurrentAnimation->getRate() < 0.f ? 1.f : 0.f);
		}
		else
		{
			SimpleMath::Vector3 getModelSpaceDeltaTranslationFromAnimation2(AnimationBase* Anim);
			CurrentAnimation->advanse(elapsedTime, SimpleMath::Vector3(), SimpleMath::Quaternion());
			ModelSpaceDeltaDeltaTranslation = getModelSpaceDeltaTranslationFromAnimation2(CurrentAnimation);
		}
		LerpPoses(BlendK);

		SimpleMath::Vector3 Hands[2];
		Hands[0] = SimpleMath::Vector3::Transform(ModelSpaceDeltaDeltaTranslation + CurrentMetaChannels[1], modelTransform);
		Hands[1] = SimpleMath::Vector3::Transform(ModelSpaceDeltaDeltaTranslation + CurrentMetaChannels[2], modelTransform);

		DeltaRotation = SimpleMath::Quaternion::Identity;
		DeltaTranslation = SimpleMath::Vector3::TransformNormal(ModelSpaceDeltaDeltaTranslation, modelTransform);

		auto Ret = ClimbingPathHelper->Advanse(this, AnimationTime, Impl->global_time, elapsedTime, MasterHandIndex, Hands[MasterHandIndex], SlaveHandIndex, Hands[SlaveHandIndex], CurrentJoints[64][1], DeltaRotation, DeltaTranslation);

		Animation* NewAnimation = CurrentAnimation;

		if (Ret == IClimbingPathHelper::EPath::Corner_Inside)
		{
			BlendHandIndex = MasterHandIndex;

			NewAnimation = Corner_Inside_Hanging;
		}
		else if (Ret == IClimbingPathHelper::EPath::Corner_Outside)
		{
			BlendHandIndex = MasterHandIndex;

			NewAnimation = Corner_Outside_Hanging;
		}
		else if (Ret == IClimbingPathHelper::EPath::Edge_Straight)
		{
			BlendHandIndex = SlaveHandIndex;

			NewAnimation = ChooseShimmy();
		}

		if (CurrentAnimation != NewAnimation)
		{
			PreviousJoints = CurrentJoints;

			CurrentAnimation = NewAnimation;

			BlendK = 0.f;
			AnimationTime = 0.f;

			CurrentAnimation->reset();
			CurrentAnimation->setPlaying(true);

			HoldHandLocationWhileBlend = Hands[BlendHandIndex];
			SlaveHandLocationWhileBlend = Hands[SlaveHandIndex];
		}
		else
		{
			BlendK = min(AnimationTime / 0.250f, 1.f);

			if (BlendK < 1.f)
			{
				const auto delta = (HoldHandLocationWhileBlend - Hands[BlendHandIndex]);
				DeltaTranslation += delta;
				const auto slave = Hands[SlaveHandIndex] + delta;
				DeltaTranslation += ClimbingPathHelper->ToSegmentBasis(slave, ClimbingPathHelper->GetCurrentSegmentIndex()) - slave;
			}
		}
		//ClimbingPathHelper->UpdateSegmantCoords(SimpleMath::Vector3::Transform(LeftHand, modelTransform), SimpleMath::Vector3::Transform(RightHand, modelTransform));

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//sprintf(DebugBuffer, "ShimmyAnimation %f %f %f\n", DeltaTranslation.x, DeltaTranslation.y, DeltaTranslation.z); Debug();
		//sprintf(DebugBuffer, "ShimmyAnimation AnimationTime %f %f %f\n", AnimationTime, BlendK, CurrentAnimation->getLocTime()); Debug();

		Impl->frameNo += 1;
		Impl->global_time += elapsedTime;
		AnimationTime += elapsedTime;

		//LerpPoses(PoseTime / PoseMaxTime);

		//if (IsPhaseEnd())
		//{
		//	StartNewPhase();
		//}
	}
	SimpleMath::Vector3 ModelSpaceRightHand()
	{
		SimpleMath::Vector3 vec4ToVec3(SimpleMath::Vector4 v4);

		JointHelpers::AnimToChain(jointsRefsChains[0], this, ChainsJoints);

		JointHelpers::localToModel(jointsRefsChains[0].size(), ChainsJoints);

		return 0.01f*vec4ToVec3(ChainsJoints[jointsRefsChains[0].size() - 1][2]);
	}
	SimpleMath::Vector3 ModelSpaceLeftHand()
	{
		SimpleMath::Vector3 vec4ToVec3(SimpleMath::Vector4 v4);

		JointHelpers::AnimToChain(jointsRefsChains[1], this, ChainsJoints);

		JointHelpers::localToModel(jointsRefsChains[1].size(), ChainsJoints);

		return 0.01f*vec4ToVec3(ChainsJoints[jointsRefsChains[1].size() - 1][2]);
	}
	void LerpPoses(float K)
	{
		auto size = CurrentJoints.size();

		for (int i = 0; i < size; i++)
		{
			auto & joint1 = PreviousJoints[i];
			auto & joint2 = CurrentAnimation->CurrentJoints[i];
			auto & joint = CurrentJoints[i];

			joint[0] = SimpleMath::Vector4::Lerp(joint1[0], joint2[0], K);
			joint[1] = SimpleMath::Quaternion::Lerp(joint1[1], joint2[1], K);
			joint[2] = SimpleMath::Vector4::Lerp(joint1[2], joint2[2], K);
		}
		
		CurrentMetaChannels[1] = ModelSpaceLeftHand();
		CurrentMetaChannels[2] = ModelSpaceRightHand();
	}
	void subscribe(char * eventName, std::function<void __cdecl(bool state)> callback)
	{
		onPlayingChanged = callback;
	}
	bool getPlaying()
	{
		return Impl->playing;
	}
	void setPlaying(bool value)
	{
		if (value)
		{
			ClimbingPathHelper->LedgeToSegments();
		}

		CurrentAnimation->setPlaying(value);

		if (Impl->playing != value)
		{
			Impl->playing = value;
			if (onPlayingChanged)
				onPlayingChanged(Impl->playing);
		}
	}
	bool isPlaying()
	{
		return Impl->playing || Impl->frameNo != Impl->prev_frameNo;
	}
	void reset()
	{
		Impl->prev_frameNo = Impl->frameNo = 0;

		Impl->global_time = .0;

		Impl->playing = false;

		Shimmy->reset();

		Corner_Inside_Hanging->reset();

		CurrentAnimation = ChooseShimmy();

		PreviousJoints = CurrentAnimation->CurrentJoints;

		AnimationTime = 0.250f;

		BlendK = 0.f;

		ClimbingPathHelper->ResetStates();
	}
	void AddOffset(SimpleMath::Vector3 value)
	{
	}
	double getRate()
	{
		return 1; 
	}
	void setRate(double speed)
	{
	};
	int getFrame()
	{
		return 0; 
	}
	double getLocTime(){
		return 0; 
	};
	bool IsLoop(){
		return false;
	};
	void TransformMetaSamples(int channelId, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f)
	{
	}
	void TransformJointSamples(int jointId, char* sampleKind, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f)
	{
	}
	void setLooping(bool loop)
	{
	}
};

Animation * createShimmyAnimation(bool MoveRight, std::map<std::string, unsigned int> & FramesNamesIndex, std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans)
{
	return new ShimmyAnimation(MoveRight, FramesNamesIndex, getSkeletMatrix, calculateFramesTrans);
}

//кастомный переход в айдловую анимацию
namespace
{
	SimpleMath::Vector3 normalize(SimpleMath::Vector3 v)
	{
		v.Normalize();

		return v;
	}
	SimpleMath::Matrix frame(const SimpleMath::Vector3& x, const SimpleMath::Vector3& y, const SimpleMath::Vector3& z, const SimpleMath::Vector3& o)
	{
		return SimpleMath::Matrix(
			vec4(x, 0.0f),
			vec4(y, 0.0f),
			vec4(z, 0.0f),
			vec4(o, 1.0f)
		);
	}
}

IClimbingPathHelper::~IClimbingPathHelper()
{
}

struct FClimbingPathHelper : IClimbingPathHelper
{
	struct FSegment
	{
		bool bInit;

		SimpleMath::Vector3 A;
		SimpleMath::Vector3 B;

		SimpleMath::Vector3 Basis;

		void UpdateBasis()
		{
			Basis = B - A;
		}

		void OffsetSegment(const SimpleMath::Vector3& Offset)
		{
			auto X = normalize(B - A);
			auto Y = SimpleMath::Vector3(0, 1, 0);
			auto Z = X.Cross(Y);

			A = SimpleMath::Vector3::Transform(Offset, frame(X, Y, Z, A));

			B = SimpleMath::Vector3::Transform(Offset, frame(X, Y, Z, B));

			Basis = B - A;
		}

		SimpleMath::Plane GetPlane()
		{
			auto X = normalize(B - A);
			auto Y = SimpleMath::Vector3(0, 1, 0);
			auto Z = X.Cross(Y);

			return SimpleMath::Plane(A, Z);
		}

		SimpleMath::Quaternion MakeCharacterOrientation()
		{
			auto Y = SimpleMath::Vector3(0, 1, 0);
			auto Z = normalize(B - A);
			auto X = Y.Cross(Z);

			return SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix(X, Y, Z));
		}
	};

	struct FCorner
	{
		static EPath ComputePathState(const FSegment& SegmentA, const FSegment& SegmentB)
		{
			static auto Y = SimpleMath::Vector3(0, 1, 0);
			float F = SegmentA.Basis.Cross(SegmentB.Basis).Dot(Y);
			if (-.00001f < F && F < .00001f)
			{
				sprintf(DebugBuffer, "EPath::Edge_Straight\n"); Debug();
				return EPath::Edge_Straight;
			}
			else if (F < .0f)
			{
				sprintf(DebugBuffer, "EPath::Corner_Inside\n"); Debug();
				return EPath::Corner_Inside;
			}
			else if (.0f < F)
			{
				sprintf(DebugBuffer, "EPath::Pre_Corner_Outside\n"); Debug();
				return EPath::Pre_Corner_Outside;
			}
		}
	};

	/////
	FSegment DummySegment;
	/////
	SimpleMath::Vector3 SegmentOffset;
	/////
	int CurrentSegmentIndex;

	std::vector<FSegment> Segments;
	/////

	/////
	EPath CurrentPathState;

	bool InitSlaveHandOffsetSpeed;

	float SlaveHandOffsetSpeed;

	float StartYaw;

	float YawAccumulate;

	SimpleMath::Quaternion PrevRootRotation;

	float PrevGlobalTime;

	int NewSegmentIndex;
	////
	int Outside_Hand_Anchor_Index;
	SimpleMath::Vector3 Next_Corner_Outside_Hand_Anchor;
	SimpleMath::Vector3 Corner_Outside_Hand_Anchor;

	void IntersectSegments(FSegment& First, FSegment& Second)
	{
		SimpleMath::Vector2 getDistanceBetweenSkewLines(SimpleMath::Vector3 S1, SimpleMath::Vector3 V1, SimpleMath::Vector3 S2, SimpleMath::Vector3 V2);

		auto Res = getDistanceBetweenSkewLines(First.A, First.Basis, Second.A, Second.Basis);

		First.B = Second.A = (Second.A + Res.y * Second.Basis);

		First.UpdateBasis();

		Second.UpdateBasis();
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	~FClimbingPathHelper(){}

	FClimbingPathHelper(){ CurrentSegmentIndex = -1; }

	void LedgeToSegments()
	{
		Segments.clear();

		auto& Ledge = GWorld.Ledges[state_hanging_Ledge_Name];

		for (auto& Box : Ledge.Boxes)
		{
			auto A = Box.origin, B = Box.origin;
			A -= 0.5f*Box.worldForward;
			B += 0.5f*Box.worldForward;
			FSegment Segment = { false, A, B, B - A };
			Segments.push_back(Segment);
		}

		auto& CurrentSegment = Segments[NewSegmentIndex = CurrentSegmentIndex = state_hanging_Ledge_BoxIndex];

		auto X = normalize(CurrentSegment.Basis);
		auto Y = SimpleMath::Vector3(0, 1, 0);
		auto Z = X.Cross(Y);
		
		SegmentOffset.x = 0.f;

		SegmentOffset.y = Y.Dot(state_hanging_Hand_Location - CurrentSegment.A);

		SegmentOffset.z = Z.Dot(state_hanging_Hand_Location - CurrentSegment.A);

		CurrentSegment.OffsetSegment(SegmentOffset);

		CurrentSegment.bInit = true;

		CurrentPathState = EPath::Edge_Straight;
	}

	float GetSegmentCoord(const SimpleMath::Vector3& Location, int TheCurrentSegmentIndex)
	{
		auto& CurrentSegment = Segments[TheCurrentSegmentIndex];

		return CurrentSegment.Basis.Dot(Location - CurrentSegment.A) / CurrentSegment.Basis.Dot(CurrentSegment.Basis);
	}
	SimpleMath::Vector3 ToSegmentBasis(const SimpleMath::Vector3 & Location, int TheCurrentSegmentIndex)
	{
		if (CurrentSegmentIndex == -1)
			return SimpleMath::Vector3();

		auto& CurrentSegment = Segments[TheCurrentSegmentIndex];

		Simulation::AddLedge(state_hanging_Ledge_Name, GWorld.Ledges[state_hanging_Ledge_Name].Boxes[TheCurrentSegmentIndex], TheCurrentSegmentIndex);

		return GetSegmentCoord(Location, TheCurrentSegmentIndex) * CurrentSegment.Basis + CurrentSegment.A;
	}
	int GetCurrentSegmentIndex()
	{
		return CurrentSegmentIndex;
	}

	void ResetStates()
	{
		PrevGlobalTime = -1.f;

		InitSlaveHandOffsetSpeed = false;

		YawAccumulate = 0.f;
	}
	void EnsureSegmentChanged()
	{
		if (CurrentSegmentIndex != NewSegmentIndex)
		{
			sprintf(DebugBuffer, "ShimmyAnimation Corner Change BoxIndex %d %d\n", CurrentSegmentIndex, NewSegmentIndex); Debug();

			state_hanging_Ledge_Box = GWorld.Ledges[state_hanging_Ledge_Name].Boxes[NewSegmentIndex];

			state_hanging_Ledge_BoxIndex = NewSegmentIndex;

			CurrentSegmentIndex = NewSegmentIndex;
		}
	}

	EPath Advanse(Animation* Sender, float LocalTime, float GlobalTime, float DeltaTime, int MasterHandIndex, SimpleMath::Vector3 MasterHandLocation, int SlaveHandIndex, SimpleMath::Vector3 SlaveHandLocation, DirectX::XMFLOAT4& RootRotation, SimpleMath::Quaternion& DeltaRotation, SimpleMath::Vector3& DeltaTranslation)
	{
		auto& Owner = *(((ShimmyAnimation *)Sender));
		auto& CurrentSegment = Segments[CurrentSegmentIndex];

		/// Next And Prev Segment Index
		auto NextSegmentIndex = (CurrentSegmentIndex + 1) % Segments.size();

		auto PrevSegmentIndex = CurrentSegmentIndex == 0 ? Segments.size() - 1 : CurrentSegmentIndex - 1;

		/// Offset Neighbours Segment
		if (!Segments[NextSegmentIndex].bInit)
		{
			Segments[NextSegmentIndex].OffsetSegment(SegmentOffset);

			Segments[NextSegmentIndex].bInit = true;
		}
		if (!Segments[PrevSegmentIndex].bInit)
		{
			Segments[PrevSegmentIndex].OffsetSegment(SegmentOffset);

			Segments[PrevSegmentIndex].bInit = true;
		}
		if ((CurrentSegment.B - Segments[NextSegmentIndex].A).LengthSquared() != 0.0f)
		{
			IntersectSegments(CurrentSegment, Segments[NextSegmentIndex]);
		}
		if ((Segments[PrevSegmentIndex].B - CurrentSegment.A).LengthSquared() != 0.0f)
		{
			IntersectSegments(Segments[PrevSegmentIndex], CurrentSegment);
		}

		//auto SegmentCoord = GetSegmentCoord(MasterHandLocation);

		//int NewSegmentIndex = -1;

		//auto NewSegmentBeginPoint = SimpleMath::Vector3::Zero;

		//sprintf(DebugBuffer, "Animation->Shimmy %f\n", ((ShimmyAnimation *)Sender)->CurrentAnimation->getLocTime()); Debug();

		if (CurrentPathState == EPath::Edge_Straight)
		{
			const auto AnimationLocalTime = ((ShimmyAnimation *)Sender)->CurrentAnimation->getLocTime();
			const auto IsLoop = ((ShimmyAnimation *)Sender)->CurrentAnimation->IsLoop();
			//if ( (GlobalTime - PrevGlobalTime) > 1.0f)
			if (PrevGlobalTime == -1.f || IsLoop)
			{
				PrevGlobalTime = AnimationLocalTime;

				auto PredictSegmentCoord = PredictMasterHandSegmentCoordAfterFinishMovementInThisCycle(Sender, DeltaTranslation, MasterHandIndex);

				NewSegmentIndex = CurrentSegmentIndex;

				if (1.0f < PredictSegmentCoord)
				{
					NewSegmentIndex = NextSegmentIndex;
				}
				else if (PredictSegmentCoord < 0.f)
				{
					NewSegmentIndex = PrevSegmentIndex;
				}

				//sprintf(DebugBuffer, "Climb Helper Advanse %d %f %f\n", CurrentSegmentIndex, PredictSegmentCoord, GetSegmentCoord(MasterHandLocation)); Debug();

				if (NewSegmentIndex != CurrentSegmentIndex)
				{
					StartYaw  = Quat().decompose0(RootRotation).yaw0;

					PrevRootRotation = RootRotation;

					if (MasterHandIndex == 0)
						return CurrentPathState = FCorner::ComputePathState(Segments[CurrentSegmentIndex], Segments[NewSegmentIndex]);
					else
						return CurrentPathState = FCorner::ComputePathState(Segments[NewSegmentIndex], Segments[CurrentSegmentIndex]);
				}
			}

			if (CurrentSegmentIndex != NewSegmentIndex)
			{
				//auto Animation = ((ShimmyAnimation *)Sender);
				//auto modelTransform = GetEveModelMatrix();
				//auto HandLocation = SampleMeta(Animation->Shimmy, Animation->Shimmy->getLocTime(), MasterHandIndex + 1);
				//const auto& Box = GWorld.Ledges[state_hanging_Ledge_Name].Boxes[NewSegmentIndex];
				//if (Box.Containe(DeltaTranslation + SimpleMath::Vector3::Transform(HandLocation, modelTransform)))
				if (MasterHandIndex == 1 && Owner.ShimmyMoveRightCatchLedgeTime < AnimationLocalTime)
				{
					sprintf(DebugBuffer, "Animation->Shimmy MasterHandIndex == 1 %f\n", AnimationLocalTime); Debug();
					EnsureSegmentChanged();
				}
				else if (MasterHandIndex == 0 && AnimationLocalTime < Owner.ShimmyMoveLeftCatchLedgeTime)
				{
					sprintf(DebugBuffer, "Animation->Shimmy MasterHandIndex == 0 %f\n", AnimationLocalTime); Debug();
					EnsureSegmentChanged();
				}
			}

			if (0.250f <= LocalTime)
			{
				SimpleMath::Vector3 handLocation;
				if (MasterHandIndex == 1)
				{
					if (AnimationLocalTime < Owner.ShimmyMoveRightCatchLedgeTime)
					{
						handLocation = SlaveHandLocation;
					}
					else
					{
						handLocation = MasterHandLocation;
					}
				}
				else if (MasterHandIndex == 0)
				{
					if (AnimationLocalTime < Owner.ShimmyMoveLeftCatchLedgeTime)
					{
						handLocation = MasterHandLocation;
					}
					else
					{
						handLocation = SlaveHandLocation;
					}
				}
				auto Index = CurrentSegmentIndex;
				const auto RelCoord = GetSegmentCoord(handLocation, CurrentSegmentIndex);
				if (1.f < RelCoord) Index = NextSegmentIndex;
				else if (RelCoord < 0.f) Index = PrevSegmentIndex;
				DeltaTranslation += ToSegmentBasis(handLocation, Index) - handLocation;
			}

			GWorld.Capsules["eve"].orientation = Segments[CurrentSegmentIndex].MakeCharacterOrientation();

			DeltaRotation = SimpleMath::Quaternion::Identity;
		};

		if (CurrentPathState == EPath::Corner_Inside)
		{
			if (0.250f <= LocalTime && LocalTime < 0.5f)
 			{
				if (!InitSlaveHandOffsetSpeed)
				{
					SlaveHandOffsetSpeed = GetDistanceFromMasterHandToEdge(Sender, DeltaTranslation, MasterHandIndex, NewSegmentIndex) / (.5f - LocalTime);
					InitSlaveHandOffsetSpeed = true;
					sprintf(DebugBuffer, "ShimmyAnimation SlaveHandOffsetSpeed %f\n", SlaveHandOffsetSpeed); Debug();
				}

				DeltaTranslation += SlaveHandOffsetSpeed*DeltaTime*(MasterHandIndex == 0?1.f:-1.f)*normalize(CurrentSegment.Basis);

				DeltaTranslation += ToSegmentBasis(SlaveHandLocation, CurrentSegmentIndex) - SlaveHandLocation;
			}
			else if (0.5f <= LocalTime && LocalTime < 1.0f)
			{
				EnsureSegmentChanged();

				DeltaTranslation += ToSegmentBasis(MasterHandLocation, NewSegmentIndex) - MasterHandLocation;
			}
			
			Quat Q;

			//sprintf(DebugBuffer, "ShimmyAnimation Corner_Inside Tick %f %f %f %f\n", LocalTime, YawAccumulate, StartYaw, Quat().decompose(StartYaw, PrevRootRotation, RootRotation).DeltaYaw); Debug();

			YawAccumulate += Q.decompose(StartYaw, PrevRootRotation, RootRotation).DeltaYaw;

			PrevRootRotation = RootRotation;

			DeltaRotation = Q.DeltaRotation;

			RootRotation = Q.RootRotation;

			if (1.0f <= LocalTime)
			{
				YawAccumulate = 0.f;

				GWorld.Capsules["eve"].orientation = Segments[CurrentSegmentIndex].MakeCharacterOrientation();

				DeltaRotation = SimpleMath::Quaternion::Identity;

				RootRotation = Q.RootRotation;

				return CurrentPathState = EPath::Edge_Straight;
			}
		}

		if (CurrentPathState == EPath::Corner_Outside || CurrentPathState == EPath::Pre_Corner_Outside)
		{
			if (CurrentPathState == EPath::Pre_Corner_Outside)
			{
				//sprintf(DebugBuffer, "Corner_Outside Wait Master Hand To Arrive To Corner\n"); Debug();
				//if (0.250f <= LocalTime)
				DeltaTranslation += ToSegmentBasis(SlaveHandLocation, CurrentSegmentIndex) - SlaveHandLocation;
				if (GetSegmentCoord(MasterHandLocation, CurrentSegmentIndex) <= .05f || .95f <= GetSegmentCoord(MasterHandLocation, CurrentSegmentIndex))
				{
					sprintf(DebugBuffer, "EPath::Corner_Outside\n"); Debug();
					return CurrentPathState = EPath::Corner_Outside;
				}
			}
			else {
				SimpleMath::Vector3 Hands[2] = { SlaveHandLocation, MasterHandLocation };
				auto t = LocalTime - 0.250f;
				if (t <= 0.f)
				{
					//sprintf(DebugBuffer, "Corner_Outside Blend From Edge To Corner\n"); Debug();
					Corner_Outside_Hand_Anchor = SlaveHandLocation;
					Outside_Hand_Anchor_Index = 0;
					SlaveHandOffsetSpeed = Outside_GetDistanceFromMasterHandToEdge(Sender, DeltaTranslation, MasterHandIndex, NewSegmentIndex) / (8.5f / 24.f);
				}
				else if (0.0f < t && t <= (8.5f / 24.f))
				{
					//sprintf(DebugBuffer, "Corner_Outside Offset Slave Hand\n"); Debug();
					Outside_Hand_Anchor_Index = 0;
					Next_Corner_Outside_Hand_Anchor = Hands[1];
					Corner_Outside_Hand_Anchor += SlaveHandOffsetSpeed*DeltaTime*(MasterHandIndex == 0 ? 1.f : -1.f)*normalize(CurrentSegment.Basis);
					DeltaTranslation += ToSegmentBasis(Corner_Outside_Hand_Anchor, CurrentSegmentIndex) - Hands[0];
				}
				else if ((8.5f / 24.f) < t && t <= (12.52f / 24.f)) // (0.250f + 8.5f / 24.f) < LocalTime && (0.250f + 12.52f / 24.f) > LocalTime)
				{
					//sprintf(DebugBuffer, "Corner_Outside Hold Master Hand\n"); Debug();
					if (Outside_Hand_Anchor_Index != 1)
					{
						Outside_Hand_Anchor_Index = 1;
						Corner_Outside_Hand_Anchor = Next_Corner_Outside_Hand_Anchor;
					}
					Next_Corner_Outside_Hand_Anchor = Hands[0];
					DeltaTranslation += ToSegmentBasis(Corner_Outside_Hand_Anchor, NewSegmentIndex) - Hands[1];
					//DeltaTranslation += ToSegmentBasis(MasterHandLocation) - MasterHandLocation;
				}
				else if ((12.52f / 24.f) < t && t <= (17.0f / 24.f))//((0.250f + 12.52f / 24.f) < LocalTime && (0.250f + 17.f / 24.f) > LocalTime)
				{
					//sprintf(DebugBuffer, "Corner_Outside Hold Slave Hand\n"); Debug();
					if (Outside_Hand_Anchor_Index != 0)
					{
						Outside_Hand_Anchor_Index = 0;
						Corner_Outside_Hand_Anchor = Next_Corner_Outside_Hand_Anchor;
					}
					Next_Corner_Outside_Hand_Anchor = Hands[1];
					DeltaTranslation += ToSegmentBasis(Corner_Outside_Hand_Anchor, CurrentSegmentIndex) - Hands[0];
					//DeltaTranslation += ToSegmentBasis(MasterHandLocation) - MasterHandLocation;
				}
				else if ((17.0f / 24.f) < t)//((0.250f + 17.f / 24.f) < LocalTime)
				{
					//sprintf(DebugBuffer, "Corner_Outside Hold Master Hand\n"); Debug();
					EnsureSegmentChanged();
					DeltaTranslation += ToSegmentBasis(Next_Corner_Outside_Hand_Anchor, NewSegmentIndex) - Hands[1];
					//DeltaTranslation += ToSegmentBasis(MasterHandLocation) - MasterHandLocation;
				}
			}
			Quat Q;

			YawAccumulate += Q.decompose(StartYaw, PrevRootRotation, RootRotation).DeltaYaw;

			PrevRootRotation = RootRotation;

			DeltaRotation = Q.DeltaRotation;

			RootRotation = Q.RootRotation;

			if (CurrentPathState == EPath::Corner_Outside && 1.250f <= LocalTime)
			{
				YawAccumulate = 0.f;

				GWorld.Capsules["eve"].orientation = Segments[CurrentSegmentIndex].MakeCharacterOrientation();

				DeltaRotation = SimpleMath::Quaternion::Identity;

				RootRotation = Q.RootRotation;

				return CurrentPathState = EPath::Edge_Straight;
			}
		}

		return CurrentPathState;
		/*
		/// Check if needed new segment
		if (1.0f < SegmentCoord)
		{
			NewSegmentIndex = NextSegmentIndex;

			NewSegmentBeginPoint = CurrentSegment.B;
		}
		else if (SegmentCoord < 0.f)
		{
			NewSegmentIndex = PrevSegmentIndex;

			NewSegmentBeginPoint = CurrentSegment.A;
		}

		/// Jump To New Segmment
		if (NewSegmentIndex != -1)
		{
			auto NewSegmentBasis = Segments[NewSegmentIndex].Basis;

			auto RotationAxis = normalize(CurrentSegment.Basis).Cross(normalize(NewSegmentBasis));

			if (RotationAxis.Length() > 0.00001f)
			{
				auto Angle = atan2(RotationAxis.Length(), normalize(CurrentSegment.Basis).Dot(normalize(NewSegmentBasis)));

				DeltaRotation = SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, Angle);

				DeltaTranslation += (NewSegmentBeginPoint - (SimpleMath::Vector3::Transform(SlaveHandLocation - Origin, DeltaRotation) + Origin));
			}

			CurrentSegmentIndex = NewSegmentIndex;

			(SlaveHandIndex == 0 ? LeftHandHangOnLocation : RightHandHangOnLocation) = NewSegmentBeginPoint;
		}

		return EPath::Edge_Straight;
		*/
	}

	std::vector<SimpleMath::Vector3> GetPath()
	{
		std::vector<SimpleMath::Vector3> Ret;
		for (int i = 0; i < Segments.size(); i++)
		{
			Ret.push_back(Segments[i].A);
		}
		return Ret;
	}

	SimpleMath::Vector3 SampleMeta(Animation * Anim, float time, unsigned int chanel)
	{
		SimpleMath::Vector3 sampleAnimationMeta(Animation * anim, unsigned int chanel, float time);

		return sampleAnimationMeta(Anim, chanel, time);
	}

	float GetDistanceToEdgePlane(SimpleMath::Vector3 Location, unsigned int EdgeIndex)
	{
		return Segments[EdgeIndex].GetPlane().Dot(SimpleMath::Vector4(Location.x, Location.y, Location.z, 1.f));
	}

	SimpleMath::Matrix GetEveModelMatrix()
	{
		auto RevertYaw = SimpleMath::Quaternion::CreateFromYawPitchRoll(-YawAccumulate, 0.f, 0.f);
	
		auto Q = SimpleMath::Matrix::CreateFromQuaternion(SimpleMath::Quaternion::Concatenate(RevertYaw, GWorld.Capsules["eve"].orientation));
		auto T = SimpleMath::Matrix::CreateTranslation(GWorld.Capsules["eve"].origin);

		return SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * Q * T;
	}
	
	float PredictMasterHandSegmentCoordAfterFinishMovementInThisCycle(Animation* Sender, SimpleMath::Vector3 DeltaTranslation, unsigned int channel)
	{
		SimpleMath::Vector3 HipsDelta, HandLocation, HandDelta = SimpleMath::Vector3::Zero;

		ShimmyAnimation * Animation = ((ShimmyAnimation *)Sender);
		SimpleMath::Matrix modelTransform = GetEveModelMatrix();
		bool BlendIsPlaying = EveAnimationGraph->getAnimationBlend()->isPlaying();
		float BlendTime = BlendIsPlaying ? 0.125f : 0.f;

		auto ShimmyAnimation = Animation->ShimmyKind == 1 ? Animation->Shimmy : Animation->BracedHangShimmy;

		float StartTime = .0f;
		float HandStopTime = Animation->ShimmyMoveRightCatchLedgeTime;
		int MetaIndex[] = { 1, 2 };
		if (channel == 0)
		{
			BlendTime = 1.f - BlendTime;

			StartTime = 1.f;
			HandStopTime = Animation->ShimmyMoveLeftCatchLedgeTime;
			std::swap(MetaIndex[0], MetaIndex[1]);
		}

		if (BlendIsPlaying)
		{
			const auto TargetLocation = SimpleMath::Vector3::Transform(state_hanging_Hand_Location, modelTransform.Invert());

			const auto HipsDelta = SampleMeta(ShimmyAnimation, BlendTime, 0) - SampleMeta(ShimmyAnimation, StartTime, 0);
			const auto HandLocation = SampleMeta(ShimmyAnimation, BlendTime, MetaIndex[0]) + HipsDelta;

			HandDelta = HipsDelta + (TargetLocation - HandLocation);
		}
		HipsDelta = SampleMeta(ShimmyAnimation, HandStopTime, 0) - SampleMeta(ShimmyAnimation, BlendTime, 0);
		HandLocation = SampleMeta(ShimmyAnimation, HandStopTime, MetaIndex[1]) + HipsDelta;
		HandLocation += HandDelta;

		float t = GetSegmentCoord(DeltaTranslation + SimpleMath::Vector3::Transform(HandLocation, modelTransform), CurrentSegmentIndex);
		sprintf(DebugBuffer, "Predict |HandDelta|(%f) |DeltaTranslation|(%f) getLocTime(%f) channel(%d) BlendTime(%f)\n", HandDelta.Length(), DeltaTranslation.Length(), Animation->Shimmy->getLocTime(), channel, BlendTime); Debug();
		PredictHandHangLocation = t * Segments[CurrentSegmentIndex].Basis + Segments[CurrentSegmentIndex].A;

		return t;
	}

	float GetDistanceFromMasterHandToEdge(Animation* Sender, SimpleMath::Vector3 DeltaTranslation, unsigned int chanel, unsigned int EdgeIndex)
	{
		float AnimationComputeLocalTime(Animation * anim, float time);

		auto Animation = ((ShimmyAnimation *)Sender);
		
		auto modelTransform = GetEveModelMatrix();

		auto HandLocation = SampleMeta(Animation->Corner_Inside_Hanging, 0.5f, chanel + 1) + (SampleMeta(Animation->Corner_Inside_Hanging, 0.5f, 0) - SampleMeta(Animation->Corner_Inside_Hanging, Animation->Corner_Inside_Hanging->getLocTime(), 0));

		return GetDistanceToEdgePlane(DeltaTranslation + SimpleMath::Vector3::Transform(HandLocation, modelTransform), EdgeIndex);
	}

	float Outside_GetDistanceFromMasterHandToEdge(Animation* Sender, SimpleMath::Vector3 DeltaTranslation, unsigned int chanel, unsigned int EdgeIndex)
	{
		float AnimationComputeLocalTime(Animation * anim, float time);

		auto Animation = ((ShimmyAnimation *)Sender);

		auto modelTransform = GetEveModelMatrix();

		SimpleMath::Vector3 HandLocation;
		if (chanel == 1) // move right
		{
			auto StartLeftHand = SampleMeta(Animation->Corner_Outside_Hanging, 0.f, 1);
			auto FinishLeftHand = SampleMeta(Animation->Corner_Outside_Hanging, 8.5f / 24.f, 1) + (SampleMeta(Animation->Corner_Outside_Hanging, 8.5f / 24.f, 0) - SampleMeta(Animation->Corner_Outside_Hanging, 0.f, 0));
			auto OffsetToHoldLeftHand = StartLeftHand - FinishLeftHand;
			HandLocation = OffsetToHoldLeftHand + SampleMeta(Animation->Corner_Outside_Hanging, 8.5f / 24.f, chanel + 1) + (SampleMeta(Animation->Corner_Outside_Hanging, 8.5f / 24.f, 0) - SampleMeta(Animation->Corner_Outside_Hanging, 0.f, 0));
		}
		else
		{
			auto StartRightHand = SampleMeta(Animation->Corner_Outside_Hanging, 1.f, 2);
			auto FinishRightHand = SampleMeta(Animation->Corner_Outside_Hanging, 1.f - 8.5f / 24.f, 2) + (SampleMeta(Animation->Corner_Outside_Hanging, 1.f - 8.5f / 24.f, 0) - SampleMeta(Animation->Corner_Outside_Hanging, 1.f, 0));
			auto OffsetToHoldRightHand = StartRightHand - FinishRightHand;
			HandLocation = OffsetToHoldRightHand + SampleMeta(Animation->Corner_Outside_Hanging, 1.f - 8.5f / 24.f, chanel + 1) + (SampleMeta(Animation->Corner_Outside_Hanging, 1.f - 8.5f / 24.f, 0) - SampleMeta(Animation->Corner_Outside_Hanging, 1.f, 0));
		}

		return -1.f*GetDistanceToEdgePlane(DeltaTranslation + SimpleMath::Vector3::Transform(HandLocation, modelTransform), EdgeIndex);
	}
};

IClimbingPathHelper* MakeClimbingPathHelper()
{
	auto Ret = new FClimbingPathHelper();
	return Ret;
}

AnimationBase* ShimmyAnimationGetDebugAnimation(AnimationBase *animation)
{
	auto Shimmy = ((ShimmyAnimation*)animation);

	Shimmy->Corner_Inside_Hanging->setLooping(true);
	return Shimmy->Corner_Inside_Hanging;
}

SimpleMath::Vector3 ShimmyAnimationToSegmentBasis(AnimationBase *animation, const SimpleMath::Vector3& Location)
{
	auto Shimmy = ((ShimmyAnimation*)animation);

	return ClimbingPathHelper->ToSegmentBasis(Location, ClimbingPathHelper->GetCurrentSegmentIndex());
}

int GetShimmyAnimationKind(AnimationBase *animation)
{
	auto Shimmy = ((ShimmyAnimation*)animation);

	return Shimmy->ShimmyKind;
}