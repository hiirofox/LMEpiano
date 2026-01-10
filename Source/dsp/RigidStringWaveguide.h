#pragma once

#include "DelayLine.h"

class Disperser
{
private:
	constexpr static int MaxStages = 32;
	float sampleRate = 48000;

	float zs[MaxStages];
	float a = 0;
	int stages = 5;
public:
	void SetA(float a)
	{
		this->a = a;
	}
	void SetStages(int stages)
	{
		this->stages = stages;
	}
	inline float ProcessSample(float x)
	{
		for (int i = 0; i < stages; ++i) {
			float out = -a * x + zs[i];
			zs[i] = x + a * out;
			x = out;
		}
		return x;
	}
	float GetGroupDelay(float freq)
	{
		float omega = 2.0f * (float)M_PI * freq / sampleRate;
		float a2 = a * a;
		float cos_w = cosf(omega);
		float numerator = 1.0f - a2;
		float denominator = 1.0f + a2 - 2.0f * a * cos_w;
		if (fabs(denominator) < 1e-29f) {
			return 0.0f;
		}
		float singleStageDelay = numerator / denominator;
		return singleStageDelay * (float)stages;
	}
};

class Damper
{
private:
	float z = 0;
	float dampBase = 1.0, dampHigh = 1.0;
public:
	void SetDampBase(float dampBase) { this->dampBase = dampBase; }
	void SetDampHigh(float dampHigh) { this->dampHigh = dampHigh; }
	inline float ProcessSample(float x)
	{
		float y = (x + z * dampHigh) / (1.0f + dampHigh) * dampBase;
		z = x;
		return y;
	}
};

class RigidStringWaveguide
{
private:
	float sampleRate = 48000;
	DelayLine<48000> delay;
	Disperser disperser;
	Damper damper;
	float fb = 0;
public:
	RigidStringWaveguide(float sampleRate = 48000.0)
		: sampleRate(sampleRate)
	{
	}
	void SetParams(float freq, float disp, float nonlinearV, float damp_base, float damp_high, float peakin, float peakout)
	{
		freq *= sampleRate / 4;//0->12000Hz
		float t = sampleRate / freq;
		disperser.SetA(disp);
		disperser.SetStages(5);
		t -= disperser.GetGroupDelay(freq);
		delay.SetDelayTime(t);

		damper.SetDampBase(damp_base);
		damper.SetDampHigh(damp_high);

	}
	inline float ProcessSample(float excitation)
	{
		float in = excitation + fb;
		delay.WriteSample(excitation);
		float out = damper.ProcessSample(disperser.ProcessSample(delay.ReadSample()));
		fb = out;
		return fb;
	}
};