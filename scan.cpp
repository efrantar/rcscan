#include "scan.h"

void extract_means(
  const cv::Mat& image, const std::vector<std::vector<cv::Rect>>& rects, std::vector<cv::Scalar>& res
) {
  res.resize(rects.size());
  for (int i = 0; i < rects.size(); i++) {
    res[i] = 0;
    for (int j = 0; j < rects[i].size(); j++) // can't foreach here for some reason
       res[i] += cv::mean(image(rects[i][j]));
    res[i] /= int(rects[i].size());
  }
}

