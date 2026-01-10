#pragma once

#include <vector>
#include <algorithm>
#include <math.h>
#include <string.h>

class RigidStringFDTD {
private:
	float fs;
	float dt;
	int N = 0;

	// 物理参数
	float S, K, R_base, R_high;
	float nonlinear_coeff = 0; // 新增：形变系数
	int strike_pos, pickup_pos;

	// 状态缓冲区
	float y0[2000] = { 0 };
	float y1[2000] = { 0 };
	float y2[2000] = { 0 };
	float* y_prev = y0;
	float* y = y1;
	float* y_next = y2;

	// 边界耦合
	float leftV = 0, rightV = 0;
	float leftN = 0, rightN = 0;

public:
	RigidStringFDTD(float sampleRate = 48000.0)
		: fs(sampleRate), dt(1.0f / sampleRate)
	{
		Reset();
	}

	// 设置参数并重新计算系统稳定性
	void SetParams(float freq, float disp, float nonlinearV, float damp_base, float damp_high, float peakin, float peakout)
	{
		float safeFreq = freq;

		// --- 1. 记录形变系数 ---
		// 这里的系数可以根据实际听感进行缩放，通常取 10.0~1000.0 左右会有明显效果
		nonlinear_coeff = nonlinearV * 1.0f;

		// --- 2. 自动收敛检查与 N 校准 ---
		int targetN = 1500;
		bool stable = false;
		float h, currentS, currentK;

		while (!stable && targetN >= 10) {
			h = 1.0f / (float)targetN;
			currentS = powf((safeFreq * 250.0f) * dt / h, 2);
			currentK = powf((disp * safeFreq * safeFreq * 5.0f) * dt / powf(h, 2), 2);

			// 稳定性判据
			// 注意：非线性会增加 S，所以留出 0.75 的余量是必要的，防止大振幅时崩溃
			if (currentS + 4.0f * currentK <= 0.75f) {
				stable = true;
			}
			else {
				targetN--;
			}
		}

		if (!stable) {
			targetN = 10;
			h = 1.0f / (float)targetN;
			currentS = powf((safeFreq * 250.0f) * dt / h, 2);
			currentK = powf((disp * safeFreq * safeFreq * 5.0f) * dt / powf(h, 2), 2);
			while (currentS + 4.0f * currentK > 0.75f) {
				currentS *= 0.999f;
				currentK *= 0.99f;
			}
		}

		N = targetN;
		S = currentS;
		K = currentK;

		R_base = damp_base * freq * 0.00001f;
		R_high = damp_high * 0.0005f;

		strike_pos = std::max(2, std::min(N - 2, (int)(peakin * N)));
		pickup_pos = std::max(1, std::min(N - 1, (int)(peakout * N)));
	}

	void SetBoundary(float left, float right)
	{
		leftN = left;
		rightN = right;
	}

	float GetLeftBoundary() const { return leftV; }
	float GetRightBoundary() const { return rightV; }

	// 按采样处理
	inline float ProcessSample(float excitation)
	{
		if (N <= 0) return 0.0f;
		// 1. 激励注入（建议注入一个稍宽的区域，模拟榔头宽度）
		float amp = excitation * 0.05f;
		y[strike_pos] += amp;
		y[strike_pos - 1] += amp * 0.5f;
		y[strike_pos + 1] += amp * 0.5f;

		// 2. 核心计算循环
		for (int i = 2; i <= N - 2; ++i)
		{
			float curr = y[i];
			float prev = y_prev[i];

			// --- 核心修改：局部张力调制 ---
			// 计算局部应变（斜率）。使用 (y[i+1]-y[i-1]) / 2h 的近似
			float slope = (y[i + 1] - y[i - 1]);
			float local_strain = slope * slope;

			// 形变系数作用：
			// 增加一个非线性增益，但使用 fast_exp 或简单的限制防止爆炸
			// nonlinear_coeff 越大，音头越“炸”，高频噪声越丰富
			float nonlin_gain = 1.0f + (nonlinear_coeff * local_strain);

			// 安全限幅：防止局部张力超过稳定性极限
			// 如果 S * nonlin_gain + 4K > 0.9，则强制限幅
			if (S * nonlin_gain + 4.0f * K > 0.95f) {
				nonlin_gain = (0.95f - 4.0f * K) / S;
			}

			float S_local = S * nonlin_gain;

			// --- 物理算子 ---
			float term_tension = S_local * (y[i + 1] - 2.0f * curr + y[i - 1]);
			float term_stiff = -K * (y[i + 2] - 4.0f * y[i + 1] + 6.0f * curr - 4.0f * y[i - 1] + y[i - 2]);

			// 阻尼逻辑保持不变
			float laplacian_curr = (y[i + 1] - 2.0f * curr + y[i - 1]);
			float laplacian_prev = (y_prev[i + 1] - 2.0f * prev + y_prev[i - 1]);
			float term_damp_high = R_high * (laplacian_curr - laplacian_prev);

			float physics = term_tension + term_stiff + term_damp_high;
			y_next[i] = (2.0f * curr - prev * (1.0f - R_base) + physics) / (1.0f + R_base);
		}

		// 激励注入
		//y_next[strike_pos] += excitation * 0.02f;

		// --- C. 边界处理 ---
		// 边界处同样需要使用动态的 S_curr
		y_next[1] = (2.0f * y[1] - y_prev[1] + S * (y[2] - 2.0f * y[1] + y[0])) / (1.0f + R_base);
		y_next[N - 1] = (2.0f * y[N - 1] - y_prev[N - 1] + S * (y[N] - 2.0f * y[N - 1] + y[N - 2])) / (1.0f + R_base);

		// 固定边界条件
		leftV = y_next[1];
		rightV = y_next[N - 1];
		y_next[0] = leftN;
		y_next[N] = rightN;

		float out = y_next[pickup_pos];

		std::swap(y_prev, y);
		std::swap(y, y_next);

		return out;
	}

	void ProcessBlock(const float* excitation, float* output, int numSamples)
	{
		for (int n = 0; n < numSamples; ++n) {
			output[n] = ProcessSample(excitation[n]);
		}
	}

	void Reset()
	{
		memset(y0, 0, sizeof(y0));
		memset(y1, 0, sizeof(y1));
		memset(y2, 0, sizeof(y2));
	}
};