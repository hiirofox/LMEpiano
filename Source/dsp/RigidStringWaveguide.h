#pragma once

#include <complex>
#include "DelayLine.h"

class Disperser
{
private:
	constexpr static int MaxStages = 32;
	float sampleRate = 48000;

	float zs[MaxStages] = { 0 };
	float a = 0;
	int stages = 5;
public:
	Disperser(float sampleRate = 48000) :sampleRate(sampleRate)
	{
		for (int i = 0; i < MaxStages; ++i) zs[i] = 0;
	}
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
	float GetPhaseDelay(float freq)
	{
		if (freq <= 0.0f) return (1.0f - a) / (1.0f + a) * (float)stages;

		float omega = 2.0f * (float)M_PI * freq / sampleRate;

		std::complex<float> z_inv(cosf(-omega), sinf(-omega));

		std::complex<float> numerator = -a + z_inv;
		std::complex<float> denominator = 1.0f - a * z_inv;
		std::complex<float> H = numerator / denominator;

		float phase = std::arg(H);
		float singleStageDelay = -phase / omega;

		return singleStageDelay * (float)stages;
	}
	void Reset()
	{
		for (int i = 0; i < MaxStages; ++i)
		{
			zs[i] = 0;
		}
	}
};

class Damper
{
private:
	float sampleRate = 48000;
	float z = 0;
	float dampBase = 1.0, dampHigh = 1.0;
public:
	Damper(float sampleRate = 48000) :sampleRate(sampleRate)
	{
	}
	void SetDampBase(float dampBase) { this->dampBase = 1.0 - dampBase; }
	void SetDampHigh(float dampHigh) { this->dampHigh = dampHigh; }
	inline float ProcessSample(float x)
	{
		float y = (x + z * dampHigh) / (1.0f + dampHigh) * dampBase;
		z = x;
		return y;
	}
	float GetPhaseDelay(float freq)
	{
		float omega = 2.0f * (float)M_PI * freq / sampleRate;
		std::complex<float> z_inv(cosf(-omega), sinf(-omega));
		std::complex<float> H_part = 1.0f + dampHigh * z_inv;
		float phase = std::arg(H_part);
		return -phase / omega;
	}
	void Reset()
	{
		z = 0;
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
		disp = 1.0 - expf(-disp * 5.0f);
		disperser.SetA(disp);
		disperser.SetStages(25);

		damp_base = expf((damp_base - 1.0f) * 8.0f) - expf(-8.0f);
		damp_high = expf((damp_high - 1.0f) * 8.0f) - expf(-8.0f);
		damper.SetDampBase(damp_base);
		damper.SetDampHigh(damp_high);

		float totalPeriod = sampleRate / freq;
		float dispDelay = disperser.GetPhaseDelay(freq);
		float dampDelay = damper.GetPhaseDelay(freq);
		float t = totalPeriod - dispDelay - dampDelay;
		if (t < 2.0f) t = 2.0f;
		delay.SetDelayTime(t);
	}
	inline float ProcessSample(float excitation)
	{
		float in = excitation + fb;
		delay.WriteSample(in);
		float out = damper.ProcessSample(disperser.ProcessSample(delay.ReadSample()));
		fb = out;
		return fb;
	}
	void Reset()
	{
		delay.Reset();
		disperser.Reset();
		damper.Reset();
		fb = 0;
	}
};