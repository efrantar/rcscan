/**
 * Utilities for extracting facelet color values.
 */

#ifndef __SCAN__
#define __SCAN__

#include <opencv2/opencv.hpp>

void extract_means(
  const cv::Mat& image, const std::vector<std::vector<cv::Rect>>& rects, std::vector<cv::Scalar>& res
);

#endif
