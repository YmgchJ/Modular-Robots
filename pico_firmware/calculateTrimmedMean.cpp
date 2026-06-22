#include "calculateTrimmedMean.h"
#include <algorithm>

std::tuple<float, float, float> calculateTrimmedMean(std::vector<int>& data, int windowSize) {
  // データサイズが不足している場合は現在のデータサイズを使用
  int currentSize = data.size();
  int effectiveWindowSize = std::min(currentSize, windowSize);

  // データをソート
  std::vector<int> sortedData = data;
  std::sort(sortedData.begin(), sortedData.end());

  // 下位10%と上位10%のインデックスを計算
  int lowerIndex = effectiveWindowSize * 0.1;
  int upperIndex = effectiveWindowSize * 0.9;

  // 下位10%と上位10%の値を取得
  float lowerValue = sortedData[lowerIndex] / 4095.0f;
  float upperValue = sortedData[upperIndex] / 4095.0f;

  // upper と lower が同じ場合はデフォルト値を返す
  if (lowerValue == upperValue) {
    return {0.10f, 0.16f, 0.22f}; // デフォルト値
  }

  // 平均を計算
  float meanValue = (lowerValue + upperValue) / 2.0f;

  return {lowerValue, upperValue, meanValue};
}