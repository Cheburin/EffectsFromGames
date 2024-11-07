struct ASample
{
	double time;
	DirectX::XMFLOAT4 payload;
	ASample(double t, float x, float y, float z, float w)
	{
		time = t;
		payload.x = x;
		payload.y = y;
		payload.z = z;
		payload.w = w;
	}
};

struct JointSamples
{
	std::vector<ASample> X;
	DirectX::XMFLOAT4 Y;
	int Z; //currentSampleIndex

	JointSamples()
	{
		resetSampleIndex(0);
	}

	void resetSampleIndex(double rate)
	{
		if (rate < .0){
			Z = X.size()-1;
		}
		else{
			Z = 0;
		}
	}

	void assign(const SimpleMath::Vector4 & value)
	{
		Y.x = value.x;
		Y.y = value.y;
		Y.z = value.z;
		Y.w = value.w;
	}

	void append(double time, float x, float y, float z, float w)
	{
		X.push_back(ASample(time, x, y, z, w));
	}

protected:
	struct AKeyPair {
		DirectX::XMFLOAT4 first;
		DirectX::XMFLOAT4 second;
		double t;
	};
	AKeyPair _getSample(double time)
	{
		auto & key = Z;
		auto & x = X;
		bool forward = x[key].time < time;
		if (forward){
			for (; key < x.size() && x[key].time < time; key++);
		}
		else{
			for (; -1 < key && time < x[key].time; key--);
		}
		if (key == x.size() || key == -1)
		{
			throw "framePayload out of duration";
		}
		else if (time != x[key].time)
		{
			if (forward)
			{
				key--;
				//interpolation between two frames
				auto & firstKey = x[key];
				auto & secondKey = x[key + 1];
				AKeyPair keys = { firstKey.payload, secondKey.payload, (time - firstKey.time) / (secondKey.time - firstKey.time) };
				return keys;
			}
			else{
				key++;
				//interpolation between two frames
				auto & firstKey = x[key - 1];
				auto & secondKey = x[key];
				AKeyPair keys = { firstKey.payload, secondKey.payload, (time - firstKey.time) / (secondKey.time - firstKey.time) };
				return keys;
			}
		}
		else if (time == x[key].time)
		{
			auto & firstKey = x[key];
			AKeyPair keys = { firstKey.payload, DirectX::XMFLOAT4(), 0.0 };
			return keys;
		}
	}

	bool IsEmpty()
	{
		return X.size()==0;
	}

public:
	void getSample(DirectX::XMFLOAT4 & sample)
	{
		sample = Y;
	}

	void getSample(double time, DirectX::XMFLOAT4 & sample)
	{
		if (IsEmpty())
		{
			sample = Y;
		}
		else
		{
			AKeyPair keys0 = _getSample(time);
			sample = SimpleMath::Vector4::Lerp(keys0.first, keys0.second, keys0.t);
		}
	}
};

struct JointQuaternionSamples : public JointSamples
{
public:
	void getSample(DirectX::XMFLOAT4 & sample)
	{
		sample = Y;
	}

	void getSample(double time, DirectX::XMFLOAT4 & sample)
	{
		if (IsEmpty())
		{
			sample = Y;
		}
		else
		{
			AKeyPair keys0 = _getSample(time);
			sample = SimpleMath::Quaternion::Lerp(keys0.first, keys0.second, keys0.t);
		}
	}
};

struct AnimationRep
{
	double local_time;

	double prev_local_time;

	double local_duration;

	double global_time;

	double Rate;

	double global_duration;

	bool looped;

	bool playing;

	int frameNo;

	int prev_frameNo;

	AnimationRep()
	{
		looped = true;

		playing = false;

		global_time = 0.0;

		local_time = 0.0;

		prev_local_time = 0.0;

		frameNo = 0;

		prev_frameNo = 0;
	}
};
struct AnimationRep2 : AnimationRep
{
public:

	std::vector< JointSamples > JointsScalingSamples;
	std::vector< JointQuaternionSamples > JointsRotationSamples;
	std::vector< JointSamples > JointsTranslationSamples;

	std::vector<JointSamples> MetaSamplesChannels;

public:

	int loop_counter;

	SimpleMath::Vector3 prev_translation;

	SimpleMath::Vector3 begin_translation;

	SimpleMath::Vector3 end_translation;

	SimpleMath::Vector3 a_translation;

	SimpleMath::Vector3 b_translation;

	SimpleMath::Vector3 ab_translation;

	int jointsCount;

	SimpleMath::Vector3 offset;

	std::function<void __cdecl(bool state)> onPlayingChanged;

	void TransformMetaSamples(int channelId, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f)
	{
		auto& MetaSamples = MetaSamplesChannels[channelId];
		for (int sampleIndex = 0; sampleIndex < MetaSamples.X.size(); sampleIndex++)
		{
			auto p = SimpleMath::Vector4(MetaSamples.X[sampleIndex].payload);
			auto ret = f(p);
			MetaSamples.X[sampleIndex].payload.x = ret.x;
			MetaSamples.X[sampleIndex].payload.y = ret.y;
			MetaSamples.X[sampleIndex].payload.z = ret.z;
		}
	}

	void TransformJointTranslationSamples(int jointId, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f)
	{
		auto& JointSamples = JointsTranslationSamples[jointId];
		for (int sampleIndex = 0; sampleIndex < JointSamples.X.size(); sampleIndex++)
		{
			auto p = SimpleMath::Vector4(JointSamples.X[sampleIndex].payload);
			auto ret = f(p);
			JointSamples.X[sampleIndex].payload.x = ret.x;
			JointSamples.X[sampleIndex].payload.y = ret.y;
			JointSamples.X[sampleIndex].payload.z = ret.z;
			JointSamples.X[sampleIndex].payload.w = ret.w;
		}
	}

	void TransformJointRotationSamples(int jointId, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f)
	{
		auto& JointSamples = JointsRotationSamples[jointId];
		for (int sampleIndex = 0; sampleIndex < JointSamples.X.size(); sampleIndex++)
		{
			auto p = SimpleMath::Vector4(JointSamples.X[sampleIndex].payload);
			auto ret = f(p);
			JointSamples.X[sampleIndex].payload.x = ret.x;
			JointSamples.X[sampleIndex].payload.y = ret.y;
			JointSamples.X[sampleIndex].payload.z = ret.z;
			JointSamples.X[sampleIndex].payload.w = ret.w;
		}
	}

	void setJoint(int JointNo, SimpleMath::Matrix matrix)
	{
		DirectX::XMVECTOR S, Q, T;
		DirectX::FXMMATRIX M = matrix;
		if (!XMMatrixDecompose(&S, &Q, &T, M))
		{
			throw "JointSamples set failed (XMMatrixDecompose return false)";
		}
		JointsScalingSamples[JointNo].assign(SimpleMath::Vector4(S));
		JointsRotationSamples[JointNo].assign(SimpleMath::Vector4(Q));
		JointsTranslationSamples[JointNo].assign(SimpleMath::Vector4(T));
	}

	void appendScaling(int JointNo, double t, float x, float y, float z)
	{
		JointsScalingSamples[JointNo].append(t, x, y, z, .0);
	}
	
	void appendRotation(int JointNo, double t, float x, float y, float z, float w)
	{
		JointsRotationSamples[JointNo].append(t, x, y, z, w);
	}

	void appendTranslation(int JointNo, double t, float x, float y, float z)
	{
		JointsTranslationSamples[JointNo].append(t, x, y, z, .0);
	}

	void getJointSQT(int JointNo, double t, JointSQT & joint)
	{
		JointsScalingSamples[JointNo].getSample(t, joint[0]);
		JointsRotationSamples[JointNo].getSample(t, joint[1]);
		JointsTranslationSamples[JointNo].getSample(t, joint[2]);
	}

	void getJointSQT(int JointNo, JointSQT & joint)
	{
		JointsScalingSamples[JointNo].getSample(joint[0]);
		JointsRotationSamples[JointNo].getSample(joint[1]);
		JointsTranslationSamples[JointNo].getSample(joint[2]);
	}

	void getMetaSample(int channelId, double t, SimpleMath::Vector3 & meta)
	{
		DirectX::XMFLOAT4 payload;
		MetaSamplesChannels[channelId].getSample(t, payload);
		meta.x = payload.x;
		meta.y = payload.y;
		meta.z = payload.z;
	}

	void resetSampleIndex(int JointNo, double rate)
	{
		JointsScalingSamples[JointNo].resetSampleIndex(rate);
		JointsRotationSamples[JointNo].resetSampleIndex(rate);
		JointsTranslationSamples[JointNo].resetSampleIndex(rate);
	}

	void resetMetaSampleIndex(int ChannelId, double rate)
	{
		MetaSamplesChannels[ChannelId].resetSampleIndex(rate);
	}

	void setMetaChannelsCount(int Count)
	{
		MetaSamplesChannels.resize(Count);
	}

	void setJointsCount(int Count)
	{
		jointsCount = Count;
		JointsScalingSamples.resize(Count);
		JointsRotationSamples.resize(Count);
		JointsTranslationSamples.resize(Count);
	}

	void extractHandsToMeta1AndMeta2(int RootJointIndex, int Hand0JointIndex, int Hand1JointIndex, std::function<SimpleMath::Matrix * __cdecl(unsigned int index)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTransformations)
	{
		JointSQT joint;

		JointSamples& RootJointSamples = JointsTranslationSamples[RootJointIndex];

		for (int jointIndex = 0; jointIndex < jointsCount; jointIndex++)
		{
			resetSampleIndex(jointIndex, 1);
		}

		for (int sampleIndex = 0; sampleIndex < RootJointSamples.X.size(); sampleIndex++)
		{
			auto time = RootJointSamples.X[sampleIndex].time;

			//calculate pose into loc space at given time
			for (int jointIndex = 0; jointIndex < jointsCount; jointIndex++)
			{
				getJointSQT(jointIndex, time, joint);
				*getSkeletMatrix(jointIndex) = joint.matrix();
			}

			//pose to model space
			calculateFramesTransformations();

			auto Hand0Translation = getSkeletMatrix(Hand0JointIndex)->Translation();

			auto Hand1Translation = getSkeletMatrix(Hand1JointIndex)->Translation();

			//save x y z of root in model space to meta channel
			MetaSamplesChannels[1].append(time, Hand0Translation.x, Hand0Translation.y, Hand0Translation.z, .0);

			//save x y z of root in model space to meta channel
			MetaSamplesChannels[2].append(time, Hand1Translation.x, Hand1Translation.y, Hand1Translation.z, .0);
		}

		for (int jointIndex = 0; jointIndex < jointsCount; jointIndex++)
		{
			resetSampleIndex(jointIndex, 1);
		}
	}

	void SetMeta0ToZero()
	{
		MetaSamplesChannels[0].append(0.0, 0., 0., 0., 0.);
		MetaSamplesChannels[0].append(1.0, 0., 0., 0., 0.);
	}

	void extractRootMotionToMeta0(int RootJointIndex, int Toe0JointIndex, int Toe1JointIndex, int ModelJointIndex, bool extractHeight, std::function<SimpleMath::Matrix * __cdecl(unsigned int index)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTransformations)
	{
		JointSQT joint;

		JointSamples& RootJointSamples = JointsTranslationSamples[RootJointIndex];

		for (int jointIndex = 0; jointIndex < jointsCount; jointIndex++)
		{
			resetSampleIndex(jointIndex, 1);
		}

		for (int sampleIndex = 0; sampleIndex < RootJointSamples.X.size(); sampleIndex++)
		{
			auto time = RootJointSamples.X[sampleIndex].time;

			//calculate pose into loc space at given time
			for (int jointIndex = 0; jointIndex < jointsCount; jointIndex++)
			{
				getJointSQT(jointIndex, time, joint);
				*getSkeletMatrix(jointIndex) = joint.matrix();
			}
			
			//pose to model space
			calculateFramesTransformations();

			auto ModelRootTranslation = getSkeletMatrix(RootJointIndex)->Translation();

			auto ModelToe0Translation = getSkeletMatrix(Toe0JointIndex)->Translation();

			auto ModelToe1Translation = getSkeletMatrix(Toe1JointIndex)->Translation();

			SimpleMath::Vector3 ModelToeTranslation = ModelToe1Translation;

			if (ModelToe0Translation.y < ModelToe1Translation.y)
			{
				ModelToeTranslation = ModelToe0Translation;
			}

			auto fromModelToRootTransform = getSkeletMatrix(ModelJointIndex)->Invert();

			auto ModelToeYTranslation = SimpleMath::Vector4(0, ModelToeTranslation.y, 0, 0);

			//set x and z of root to zero at given time, offset y from toy
			RootJointSamples.X[sampleIndex].payload.x = RootJointSamples.X[sampleIndex].payload.z = 0.0f;

			if (extractHeight)
				RootJointSamples.X[sampleIndex].payload = SimpleMath::Vector4(RootJointSamples.X[sampleIndex].payload) - SimpleMath::Vector4::Transform(ModelToeYTranslation, fromModelToRootTransform);
			
			//save x y z of root in model space to meta channel
			MetaSamplesChannels[0].append(time, ModelRootTranslation.x, extractHeight?ModelToeTranslation.y:0.0f, ModelRootTranslation.z, .0);
		}

		for (int jointIndex = 0; jointIndex < jointsCount; jointIndex++)
		{
			resetSampleIndex(jointIndex, 1);
		}

		auto t0 = MetaSamplesChannels[0].X[0].payload;

		auto t1 = MetaSamplesChannels[0].X[MetaSamplesChannels[0].X.size() - 1].payload;

		begin_translation = SimpleMath::Vector3(t0.x, t0.y, t0.z);

		end_translation = SimpleMath::Vector3(t1.x, t1.y, t1.z);

		prev_translation = begin_translation;
	}

	AnimationRep2()
	{
		loop_counter = 0;

		offset = SimpleMath::Vector3();

		onPlayingChanged = nullptr;
	}
};
struct AnimationRep3 : AnimationRep
{
	AnimationBase* animation1;

	AnimationBase* animation2;

	std::function<double __cdecl(double, double)> BlendFunction;

	std::function<SimpleMath::Vector3 __cdecl(double, double, AnimationBase*, AnimationBase*)> AdvanseFunction;
};