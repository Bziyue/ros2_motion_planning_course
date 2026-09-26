#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>
#include "motion2d/mapping/esdf.hpp"

using namespace motion2d;

/** @brief Rasterized circle vs its analytic signed distance, with measured build times.
 * @details Explicit supplied geometry, not SLAM. Occupied centres inside radius R;
 * conservative bounds are to the resulting SQUARE cells, not to the original circle.
 */
int main(int argc, char ** argv)
{
  const std::filesystem::path out = argc > 1 ? argv[1] : "tmp/ch12_esdf";
  std::filesystem::create_directories(out);
  std::ofstream summary(out/"summary.csv");
  summary << "resolution,cells,mae_m,max_error_m,median_us,p95_us\n";
  for (const double resolution : {.2, .1, .05}) {
    GridConfig g; g.resolution = resolution; g.width = g.height = int(std::round(8/resolution)); g.origin = {-4, -4};
    const OccupancyGrid2D geometry(g);
    std::vector<std::int8_t> raw(g.width*g.height, 0);
    for (int i = 0; i < int(raw.size()); ++i) {if (geometry.cellCenter(i).norm() <= 1.) {raw[i] = 100;}}
    std::vector<double> times;
    for (int run = 0; run < 105; ++run) {
      const auto start = std::chrono::steady_clock::now(); Esdf2D esdf(g, raw);
      const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now()-start).count();
      if (run >= 5) {times.push_back(us);}
    }
    std::sort(times.begin(), times.end());
    Esdf2D field(g, raw);
    std::ofstream samples(out/("field_"+std::to_string(int(std::round(resolution*100)))+".csv"));
    samples << "x,y,distance,analytic,gx,gy,clearance_bound\n";
    double sum = 0, maximum = 0; int count = 0;
    // Same query lattice at all resolutions; annulus keeps exterior from being nearest.
    for (int y = -48; y <= 48; ++y) {
      for (int x = -48; x <= 48; ++x) {
        const Eigen::Vector2d p(x*.03+.007, y*.03+.011);
        const auto s = field.sample(p); if (!s) {continue;}
        const double analytic = p.norm()-1.;
        if (p.norm() >= .3 && p.norm() <= 1.6) {
          const double error = std::abs(s->distance-analytic);
          sum += error; maximum = std::max(maximum, error); ++count;
        }
        samples << p.x() << ',' << p.y() << ',' << s->distance << ',' << analytic << ','
                << s->gradient.x() << ',' << s->gradient.y() << ',' << s->clearance_lower_bound << '\n';
      }
    }
    summary << resolution << ',' << raw.size() << ',' << sum/count << ',' << maximum << ','
            << times[times.size()/2] << ',' << times[94] << '\n';
    std::cout << "resolution=" << resolution << " mean error=" << sum/count << " m, median="
              << times[times.size()/2] << " us\n";
  }
}
