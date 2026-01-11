#pragma once

#include "RigidStringFDTD.h"
#include "RigidStringWaveguide.h"
#include "Excitation.h"

class LMEpiano
{
private:
	RigidStringWaveguide str1{ 48000 };
	RigidStringWaveguide str2{ 48000 };
	RigidStringWaveguide str3{ 48000 };
	float v1 = 0, v2 = 0, v3 = 0;
	ExcitationPiano exciter;
public:
	LMEpiano(float sampleRate = 48000.0f)
	{
	}
	void SetStringParams(float freq, float disp, float nlv, float damp_base, float damp_high, float peakin, float peakout)
	{
		str1.SetParams(freq, disp, nlv, damp_base, damp_high, peakin, peakout);
		str2.SetParams(freq * 1.0025f, disp, nlv, damp_base, damp_high, peakin, peakout);
		str3.SetParams(freq / 1.0025f, disp, nlv, damp_base, damp_high, peakin, peakout);

	}
	void NoteOn(float velocity)
	{
		exciter.NoteOn(velocity);
	}
	void NoteOff()
	{
		exciter.NoteOff();
	}
	inline float ProcessSample()
	{
		float excitation = exciter.ProcessSample();
		v1 = str1.ProcessSample(excitation);
		v2 = str2.ProcessSample(excitation);
		v3 = str3.ProcessSample(excitation);

		return 0;
	}
	void ProcessBlock(float* outl, float* outr, int numSamples)
	{
		for (int n = 0; n < numSamples; ++n)
		{
			ProcessSample();
			outl[n] = (v1 + v2 + v3 * 0.25) * 0.5;
			outr[n] = (v2 + v3 + v1 * 0.25) * 0.5;
		}
	}
	void Reset()
	{
		str1.Reset();
		str2.Reset();
		str3.Reset();
	}
};