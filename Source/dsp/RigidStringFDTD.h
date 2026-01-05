#pragma once

#include <vector>
#include <algorithm>
#include <math.h>

class RigidStringFDTD {
private:
	float fs;
	float dt;
	int N = 0;

	// 物理参数
	float S, K, R_base, R_high;
	int strike_pos, pickup_pos;

	// 状态缓冲区
	float y0[2000] = { 0 };
	float y1[2000] = { 0 };
	float y2[2000] = { 0 };
	float* y_prev = y0;
	float* y = y1;
	float* y_next = y2;

	//边界耦合
	float leftV = 0, rightV = 0;
	float leftN = 0, rightN = 0;
public:
	RigidStringFDTD(float sampleRate = 48000.0)
		: fs(sampleRate), dt(1.0f / sampleRate)
	{
		Reset();
	}

	// 设置参数并重新计算系统稳定性
	void SetParams(float freq, float disp, float damp_base, float damp_high, float peakin, float peakout)
	{
		//float safeFreq = std::min(freq, fs * 0.25f);
		float safeFreq = freq;

		// --- 2. 自动收敛检查与 N 校准 ---
		int targetN = 1500; // 初始最大分辨率
		bool stable = false;
		float h, currentS, currentK;

		// 刚性弦稳定性条件补充：N 必须至少为 10 以保证五点算子有意义
		while (!stable && targetN >= 10) {
			h = 1.0f / (float)targetN;

			// 你的自定义物理映射
			currentS = powf((safeFreq * 250.0f) * dt / h, 2);
			// 注意：disp * freq * freq 增长极快，这是导致高频崩溃的主因
			currentK = powf((disp * safeFreq * safeFreq * 5.0f) * dt / powf(h, 2), 2);

			// 稳定性判据：S + 4K < 1.0
			// 留出 0.8 的余量是明智的，但我们还要考虑 R_high 的影响
			// 简单的经验法则：R_high 也会占用稳定性带宽
			if (currentS + 4.0f * currentK <= 0.75f) {
				stable = true;
			}
			else {
				targetN--;
			}
		}

		// --- 3. 最终防线：如果 N 减小到 10 依然不稳定，则强制削减 K 和 S ---
		if (!stable) {
			targetN = 10;
			h = 1.0f / (float)targetN;
			currentS = powf((safeFreq * 250.0f) * dt / h, 2);
			currentK = powf((disp * safeFreq * safeFreq * 5.0f) * dt / powf(h, 2), 2);
			// 强制削减 K 和 S，直到满足稳定性条件
			while (currentS + 4.0f * currentK > 0.75f) {
				currentS *= 0.999f;
				currentK *= 0.99f;
			}
		}

		N = targetN;
		S = currentS;
		K = currentK;

		// 2. 阻尼映射
		R_base = damp_base * freq * 0.00001f; // 缩减系数，使调节更细腻
		R_high = damp_high * 0.0005f;

		// 3. 映射输入输出位置
		strike_pos = std::max(2, std::min(N - 2, (int)(peakin * N)));
		pickup_pos = std::max(1, std::min(N - 1, (int)(peakout * N)));

		// 4. 重置缓冲区大小
		//y_prev.assign(N + 1, 0.0f);
		//y.assign(N + 1, 0.0f);
		//y_next.assign(N + 1, 0.0f);
	}
	void SetBoundary(float left, float right)//设置边界值实现多琴弦耦合
	{
		leftN = left;
		rightN = right;
	}
	float GetLeftBoundary() const
	{
		return leftV;
	}
	float GetRightBoundary() const
	{
		return rightV;
	}
	// 按采样处理 (适合实时非线性交互)
	inline float ProcessSample(float excitation)
	{
		if (N <= 0) return 0.0f;

		// 核心循环：计算内部节点 (2 到 N-2)
		// 使用 y[i-2], y[i-1], y[i], y[i+1], y[i+2] 计算四阶导数 (刚性)
		for (int i = 2; i <= N - 2; ++i)
		{
			float curr = y[i];
			float prev = y_prev[i];

			// 物理算子
			float term_tension = S * (y[i + 1] - 2.0f * curr + y[i - 1]);
			float term_stiff = -K * (y[i + 2] - 4.0f * y[i + 1] + 6.0f * curr - 4.0f * y[i - 1] + y[i - 2]);

			// 高频阻尼 (基于拉普拉斯算子的速度项)
			float laplacian_curr = (y[i + 1] - 2.0f * curr + y[i - 1]);
			float laplacian_prev = (y_prev[i + 1] - 2.0f * prev + y_prev[i - 1]);
			float term_damp_high = R_high * (laplacian_curr - laplacian_prev);

			// 时间步迭代
			//y_next[i] = (2.0f * curr - prev + term_tension + term_stiff + term_damp_high) / (1.0f + R_base);
			//y_next[i] = 2.0f * curr - prev + term_tension + term_stiff + term_damp_high;//有可能Rbase在中间加会让琴弦变软
			float physics = term_tension + term_stiff + term_damp_high;
			y_next[i] = (2.0f * curr - prev * (1.0f - R_base) + physics) / (1.0f + R_base);
			//float term_damp_base = R_base * (curr - prev);//ok!直接衰减速度
			//y_next[i] = 2.0f * curr - prev + term_tension + term_stiff + term_damp_high - term_damp_base;
		}

		// 激励注入 (在指定位置注入能量)
		// 对应 Python 中的 y_next[strike_pos] += ...
		//y_next[strike_pos] += excitation * (dt * dt);
		y_next[strike_pos] += excitation * 0.02;

		// 边界处理 (Simple supported / Hinged)
		// y_next[1]
		y_next[1] = (2.0f * y[1] - y_prev[1] + S * (y[2] - 2.0f * y[1] + y[0])) / (1.0f + R_base);
		// y_next[N-1]
		y_next[N - 1] = (2.0f * y[N - 1] - y_prev[N - 1] + S * (y[N] - 2.0f * y[N - 1] + y[N - 2])) / (1.0f + R_base);

		// 固定边界条件
		leftV = y_next[1];
		rightV = y_next[N - 1];
		y_next[0] = leftN;//设置边界值，以实现多琴弦耦合
		y_next[N] = rightN;

		// 获取输出点
		float out = y_next[pickup_pos];

		// 轮转缓冲区 (Pointer Swapping)
		std::swap(y_prev, y);
		std::swap(y, y_next);

		return out;
	}

	// 按块处理
	void ProcessBlock(const float* excitation, float* output, int numSamples)
	{
		for (int n = 0; n < numSamples; ++n)
		{
			output[n] = ProcessSample(excitation[n]);
		}
	}

	// 重置状态
	void Reset()
	{
		memset(y0, 0, sizeof(y0));
		memset(y1, 0, sizeof(y1));
		memset(y2, 0, sizeof(y2));
	}

};