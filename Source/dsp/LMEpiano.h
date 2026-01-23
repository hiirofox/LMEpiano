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
	float bridge_stiffness = 0.35f;
public:
	LMEpiano(float sampleRate = 48000.0f)
	{
	}
	void SetStringParams(float freq, float disp, float nlv, float cross, float unison, float damp_base, float damp_high)
	{
		str1.SetParams(freq, disp, nlv, damp_base, damp_high);
		float freqK = (1.0 - unison) + unison * (1.03);
		str2.SetParams(freq * freqK, disp, nlv, damp_base, damp_high);
		str3.SetParams(freq / freqK, disp, nlv, damp_base, damp_high);
		bridge_stiffness = cross * 2.0 / 3.0;
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
		float exc = exciter.ProcessSample();

		float v_bridge = (v1 + v2 + v3) * bridge_stiffness;
		float in1 = -v_bridge + v1 - exc * 0.25;
		float in2 = -v_bridge + v2 + exc;
		float in3 = -v_bridge + v3 - exc * 0.25;
		v1 = str1.ProcessSample(in1);
		v2 = str2.ProcessSample(in2);
		v3 = str3.ProcessSample(in3);
		return 0;
	}
	void ProcessBlock(float* outl, float* outr, int numSamples)
	{
		for (int n = 0; n < numSamples; ++n)
		{
			ProcessSample();
			outl[n] = (v1 + v3) * 0.5;
			outr[n] = (v1 + v3) * 0.5;
		}
	}
	void Reset()
	{
		str1.Reset();
		str2.Reset();
		str3.Reset();
	}
};

#define MaxNumPolys 16
class LMEpianoPoly
{
private:
	LMEpiano polys[MaxNumPolys];
	int notes[MaxNumPolys] = { 0 };

	float tmpl[2048];
	float tmpr[2048];

	int pos = 0;
	float pitch, disp, nlv, cross, unison, damp_base, damp_high;

public:
	void SetStringParams(float pitch, float disp, float nlv, float cross, float unison, float damp_base, float damp_high)
	{
		this->pitch = pitch;
		this->disp = disp;
		this->nlv = nlv;
		this->cross = cross;
		this->unison = unison;
		this->damp_base = damp_base;
		this->damp_high = damp_high;
	}
	void NoteOn(int note, float velo)
	{
		for (int i = 0; i < MaxNumPolys; ++i)
		{
			if (notes[i] == note)
			{
				float freq = 440.0 * powf(2.0f, (float)(note - 69) / 12.0f);
				polys[i].SetStringParams(freq * pitch, disp, nlv, cross, unison, damp_base, damp_high);
				polys[i].NoteOn(velo);
				return;
			}
		}
		polys[pos].Reset();
		float freq = 440.0 * powf(2.0f, (float)(note - 69) / 12.0f);
		polys[pos].SetStringParams(freq * pitch, disp, nlv, cross, unison, damp_base, damp_high);
		polys[pos].NoteOn(velo);
		notes[pos] = note;
		pos++;
		if (pos >= MaxNumPolys)pos = 0;
	}
	void NoteOff(int note)
	{
		for (int i = 0; i < MaxNumPolys; ++i)
		{
			if (notes[i] == note)
			{
				float freq = 440.0 * powf(2.0f, (float)(note - 69) / 12.0f);
				float damp_release = damp_base * 5.0;
				if (damp_release > 1.0)damp_release = 1.0;
				polys[i].SetStringParams(freq * pitch, disp, nlv, cross, unison, damp_release, damp_high);
				polys[i].NoteOff();
				notes[pos] = -1;
				return;
			}
		}
	}
	void ProcessBlock(float* outl, float* outr, int numSamples)
	{
		for (int i = 0; i < numSamples; ++i)
		{
			outl[i] = 0;
			outr[i] = 0;
		}
		for (int j = 0; j < MaxNumPolys; ++j)
		{
			polys[j].ProcessBlock(tmpl, tmpr, numSamples);
			for (int i = 0; i < numSamples; ++i)
			{
				outl[i] += tmpl[i];
				outr[i] += tmpr[i];
			}
		}
	}
};