#pragma once

#include <JuceHeader.h>
#include <math.h>
#include <stdlib.h>

class ExcitationPianoNoise
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

class ExcitationPiano
{
private:
	static std::vector<float> pcmData;
	static bool isLoaded;
	int readPosition = 0;
	float currentVelocity = 0.0f;

	void loadResource()
	{
		if (isLoaded == 1)return;
		isLoaded = 1;
		juce::AudioFormatManager formatManager;
		formatManager.registerBasicFormats();

		std::unique_ptr<juce::InputStream> stream(new juce::MemoryInputStream(BinaryData::Piano_IR_wav,
			BinaryData::Piano_IR_wavSize,
			false));

		std::unique_ptr<juce::AudioFormatReader> reader(formatManager.createReaderFor(std::move(stream)));


		if (reader != nullptr)
		{
			pcmData.resize((size_t)reader->lengthInSamples);

			juce::AudioBuffer<float> tempBuffer(1, (int)reader->lengthInSamples);
			reader->read(&tempBuffer, 0, (int)reader->lengthInSamples, 0, true, true);

			auto* rawData = tempBuffer.getReadPointer(0);
			for (int i = 0; i < tempBuffer.getNumSamples(); ++i)
			{
				pcmData[i] = rawData[i];
			}

		}
	}

public:
	ExcitationPiano()
	{
		loadResource();
	}

	void NoteOn(float velocity)
	{
		currentVelocity = velocity;
		readPosition = 0;
	}

	void NoteOff()
	{
	}

	inline float ProcessSample()
	{
		if (readPosition >= pcmData.size())
			return 0.0f;

		float output = pcmData[readPosition] * currentVelocity;

		readPosition++;

		return output;
	}
};