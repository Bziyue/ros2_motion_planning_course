#include "motion2d/mapping/distance_transform.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motion2d
{
namespace
{
// Finite parabolas only: infinity-infinity is never used in an intersection.
std::vector<double> envelope(const std::vector<double> & f)
{
  const int n = static_cast<int>(f.size());
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> result(n, inf), begin(n);
  std::vector<int> sites(n);
  int last = -1;
  for (int q = 0; q < n; ++q) {
    if (!std::isfinite(f[q])) {continue;}
    double crossing = -inf;
    // edt_envelope_begin
    while (last >= 0) {
      const int v = sites[last];
      crossing = ((f[q]+double(q)*q)-(f[v]+double(v)*v))/(2.0*(q-v));
      if (crossing > begin[last]) {break;}
      --last;
    }
    ++last; sites[last] = q; begin[last] = last == 0 ? -inf : crossing;
    // edt_envelope_end
  }
  if (last < 0) {return result;}
  int active = 0;
  for (int x = 0; x < n; ++x) {
    while (active < last && begin[active+1] < x) {++active;}
    const double delta = x-sites[active];
    result[x] = delta*delta+f[sites[active]];
  }
  return result;
}
}  // namespace

std::vector<double> squaredDistanceTransform(int width, int height,
  const std::vector<std::uint8_t> & seeds)
{
  if (width <= 0 || height <= 0 || std::int64_t(width)*height > 16000000 ||
    seeds.size() != static_cast<std::size_t>(width)*height) {
    throw std::invalid_argument("EDT requires matching positive dimensions, at most 16M cells");
  }
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> temporary(seeds.size()), result(seeds.size()), line(height);
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {line[y] = seeds[y*width+x] ? 0 : inf;}
    const auto column = envelope(line);
    for (int y = 0; y < height; ++y) {temporary[y*width+x] = column[y];}
  }
  line.resize(width);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {line[x] = temporary[y*width+x];}
    const auto row = envelope(line);
    for (int x = 0; x < width; ++x) {result[y*width+x] = row[x];}
  }
  return result;
}

std::vector<std::uint8_t> obstacleMask(const std::vector<std::int8_t> & occupancy,
  int free_threshold, bool unknown_blocked)
{
  if (free_threshold < 0 || free_threshold > 100) {throw std::invalid_argument("invalid free threshold");}
  std::vector<std::uint8_t> result; result.reserve(occupancy.size());
  for (const auto v : occupancy) {
    if (v < -1 || v > 100) {throw std::invalid_argument("occupancy must be -1 or 0..100");}
    result.push_back(v < 0 ? unknown_blocked : v > free_threshold);
  }
  return result;
}
}  // namespace motion2d
