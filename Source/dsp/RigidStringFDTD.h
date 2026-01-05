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

		// --- A. 计算非线性张力调制 ---
		// 我们根据当前弦的位移梯度平方和来估算弦的伸长量
		float stretch = 0.0f;
		for (int i = 0; i < N; ++i) {
			float diff = y[i + 1] - y[i];
			stretch += diff * diff;
		}

		// 动态计算当前的 S 参数
		// 振幅越大，S_curr 越大，频率瞬时升高
		float S_curr = S * (1.0f + nonlinear_coeff * stretch);

		// --- B. 核心循环 ---
		for (int i = 2; i <= N - 2; ++i)
		{
			/*
			float curr = y[i];
			float prev = y_prev[i];

			// 物理算子：使用动态的 S_curr 代替原有的 S
			float term_tension = S_curr * (y[i + 1] - 2.0f * curr + y[i - 1]);
			float term_stiff = -K * (y[i + 2] - 4.0f * y[i + 1] + 6.0f * curr - 4.0f * y[i - 1] + y[i - 2]);
			*/
			float curr = y[i];
			float prev = y_prev[i];
			// 计算局部拉伸（根据邻近节点距离差）
			float local_diff = y[i + 1] - y[i - 1]; // 使用中心差分估算斜率
			float S_local = S * (1.0f + nonlinear_coeff * local_diff * local_diff);
			// 应用局部张力
			float term_tension = S_local * (y[i + 1] - 2.0f * curr + y[i - 1]);
			// K 依然保持全局稳定，或者也可以按比例微调
			float term_stiff = -K * (y[i + 2] - 4.0f * y[i + 1] + 6.0f * curr - 4.0f * y[i - 1] + y[i - 2]);

			float laplacian_curr = (y[i + 1] - 2.0f * curr + y[i - 1]);
			float laplacian_prev = (y_prev[i + 1] - 2.0f * prev + y_prev[i - 1]);
			float term_damp_high = R_high * (laplacian_curr - laplacian_prev);

			float physics = term_tension + term_stiff + term_damp_high;
			y_next[i] = (2.0f * curr - prev * (1.0f - R_base) + physics) / (1.0f + R_base);
		}

		// 激励注入
		y_next[strike_pos] += excitation * 0.02f;

		// --- C. 边界处理 ---
		// 边界处同样需要使用动态的 S_curr
		y_next[1] = (2.0f * y[1] - y_prev[1] + S_curr * (y[2] - 2.0f * y[1] + y[0])) / (1.0f + R_base);
		y_next[N - 1] = (2.0f * y[N - 1] - y_prev[N - 1] + S_curr * (y[N] - 2.0f * y[N - 1] + y[N - 2])) / (1.0f + R_base);

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