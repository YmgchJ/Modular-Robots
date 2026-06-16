#ifndef CALCULATE_TRIMMED_MEAN_H
#define CALCULATE_TRIMMED_MEAN_H

#include <vector>
#include <tuple>

// 下位10%と上位10%の値、およびその平均を計算する関数
std::tuple<float, float, float> calculateTrimmedMean(std::vector<int>& data, int windowSize);

#endif // CALCULATE_TRIMMED_MEAN_H