/**
 * Implementation of matching the extracted BGR values to cube colors.
 */

#ifndef __MATCH__
#define __MATCH__

#include <string>

const int N_FACELETS = 54;

bool init_match(const std::string& file);
std::string match_colors(const int bgrs[N_FACELETS][3], bool fix_centers = false);

#endif
