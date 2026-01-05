/*
  ==============================================================================

	This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
LModelAudioProcessorEditor::LModelAudioProcessorEditor(LModelAudioProcessor& p)
	: AudioProcessorEditor(&p), audioProcessor(p)
{
	// Make sure that before the constructor has finished, you've set the
	// editor's size to whatever you need it to be.
	setResizable(true, true); // 允许窗口调整大小

	setOpaque(false);  // 允许在边框外面绘制

	//setResizeLimits(64 * 11, 64 * 5, 10000, 10000); // 设置最小宽高为300x200，最大宽高为800x600
	setSize(64 * 7, 64 * 2);
	setResizeLimits(64 * 7, 64 * 2, 64 * 13, 64 * 2);

	//constrainer.setFixedAspectRatio(11.0 / 4.0);  // 设置为16:9比例
	//setConstrainer(&constrainer);  // 绑定窗口的宽高限制

	K_Pitch.setText("pitch", "");
	K_Pitch.ParamLink(audioProcessor.GetParams(), "pitch");
	addAndMakeVisible(K_Pitch);
	K_Disp.setText("disp", "");
	K_Disp.ParamLink(audioProcessor.GetParams(), "disp");
	addAndMakeVisible(K_Disp);
	K_DampBase.setText("damp_base", "");
	K_DampBase.ParamLink(audioProcessor.GetParams(), "damp_base");
	addAndMakeVisible(K_DampBase);
	K_DampHigh.setText("damp_high", "");
	K_DampHigh.ParamLink(audioProcessor.GetParams(), "damp_high");
	addAndMakeVisible(K_DampHigh);
	K_PeakIn.setText("peakin", "");
	K_PeakIn.ParamLink(audioProcessor.GetParams(), "peakin");
	addAndMakeVisible(K_PeakIn);
	K_PeakOut.setText("peakout", "");
	K_PeakOut.ParamLink(audioProcessor.GetParams(), "peakout");
	addAndMakeVisible(K_PeakOut);


	startTimerHz(30);

}

LModelAudioProcessorEditor::~LModelAudioProcessorEditor()
{
}

//==============================================================================
void LModelAudioProcessorEditor::paint(juce::Graphics& g)
{
	g.fillAll(juce::Colour(0x00, 0x00, 0x00));

	g.fillAll();
	g.setFont(juce::Font("FIXEDSYS", 17.0, 1));
	g.setColour(juce::Colour(0xff00ff00));;

	int w = getBounds().getWidth(), h = getBounds().getHeight();

	//g.drawText("L-MODEL Magnitudelay", juce::Rectangle<float>(32, 16, w, 16), 1);
}

void LModelAudioProcessorEditor::resized()
{
	juce::Rectangle<int> bound = getBounds();
	int x = bound.getX(), y = bound.getY(), w = bound.getWidth(), h = bound.getHeight();
	auto convXY = juce::Rectangle<int>::leftTopRightBottom;

	K_Pitch.setBounds(32 + 64 * 0, 32, 64, 64);
	K_Disp.setBounds(32 + 64 * 1, 32, 64, 64);
	K_DampBase.setBounds(32 + 64 * 2, 32, 64, 64);
	K_DampHigh.setBounds(32 + 64 * 3, 32, 64, 64);
	K_PeakIn.setBounds(32 + 64 * 4, 32, 64, 64);
	K_PeakOut.setBounds(32 + 64 * 5, 32, 64, 64);

}

void LModelAudioProcessorEditor::timerCallback()
{
	repaint();
}
