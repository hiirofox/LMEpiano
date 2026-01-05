#pragma once

#include <math.h>
#include <stdlib.h>

class ExcitationPiano
{
private:
	float v = 0;
	float randf()
	{
		return (float)(rand() % 10000) / 10000.0f * (rand() % 2 ? 1 : -1);
	}
public:
	void NoteOn(float velocity)
	{
		v = velocity;
	}
	void NoteOff()
	{
		v = 0;
	}
	inline float ProcessSample()
	{
		v *= 0.999f; // Ë¥¼õËÙ¶È	
		return (v > 0.5 ? 1 : 0) * randf();
	}
};