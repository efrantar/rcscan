#include <getopt.h>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

#include "match.h"
#include "scan.h"

const std::string RECTFILE = "scan.rects";

int main(int argc, char *argv[]) {
  bool fix_centers = false;
  
  int opt;
  while ((opt = getopt(argc, argv, "f")) != -1) {
    switch (opt) {
      case 'f':
        fix_centers = true;
        break;
      default:
        std::cout << "Usage: ./scan [-f]" << std::endl;
        return 1;
    }
  }

  std::cout << "This is rcscan v0.9; copyright Elias Frantar 2020." << std::endl << std::endl;

  if (!init_match()) {
    std::cout << "Error loading `scan.tbl`." << std::endl;
    return 0;
  }
  std::vector<std::vector<cv::Rect>> rects(N_FACELETS);

  std::ifstream f(RECTFILE);
  std::string l;
  for (int i = 0; i < N_FACELETS; i++) {
    if (!std::getline(f, l))
      break;
    std::istringstream line(l);
    while (line) {
      cv::Rect r;
      line >> r.x >> r.y >> r.width >> r.height;
      rects[i].push_back(r);
    }
  }
  for (std::vector<cv::Rect>& rs : rects) {
    if (rs.empty()) {
      std::cout << "Invalid `scan.rects`." << std::endl;
      return 0;
    }
  }

  std::string cmd;
  cv::Mat frame;
  int bgrs[N_FACELETS][3];
  
  std::cout << "Enter >>load FILE<< to load a frame and then >>scan<< to figure out the cube state." << std::endl << std::endl;

  while (std::cin) {
    std::cout << "Ready!" << std::endl;
    std::cin >> cmd;

    if (cmd == "load") {
      std::string file;
      std::cin >> file;
      frame = cv::imread(file, cv::IMREAD_COLOR);
    } else if (cmd == "scan" && !frame.empty()) {
      auto tick = std::chrono::high_resolution_clock::now();

      std::vector<cv::Scalar> means;
      extract_means(frame, rects, means);
      for (int i = 0; i < N_FACELETS; i++) {
        for (int j = 0; j < 3; j++)
          bgrs[i][j] = means[i][j];
      }
      std::string facecube = match_colors(bgrs, fix_centers);
      std::cout << ((facecube == "") ? "Scan Error." : facecube) << std::endl;

      std::cout << std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - tick
      ).count() / 1000. << "ms" << std::endl;
    } else
      std::cout << "Error." << std::endl;
  }

  return 0;
}

