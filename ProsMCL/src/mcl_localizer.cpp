#include "api.h"
#include "mcl_localizer.h"
#include "mcl_config.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace {
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

struct Segment { double x0; double y0; double x1; double y1; };
static const Segment MAP_SEGMENTS[] = {
  {72, 72, 72, -72},
  {72, -72, -72, -72},
  {-72, -72, -72, 72},
  {-72, 72, 72, 72},
  {-24.4, 46.25, 24.4, 46.25},
  {24.4, 46.25, 24.4, 49.75},
  {24.4, 49.75, -24.4, 49.75},
  {-24.4, 49.75, -24.4, 46.25},
  {20.9, 46.25, 24.4, 46.25},
  {24.4, 46.25, 24.4, 49.75},
  {24.4, 49.75, 20.9, 49.75},
  {20.9, 49.75, 20.9, 46.25},
  {-24.4, 46.25, -20.9, 46.25},
  {-20.9, 46.25, -20.9, 49.75},
  {-20.9, 49.75, -24.4, 49.75},
  {-24.4, 49.75, -24.4, 46.25},
  {-24.4, -49.75, 24.4, -49.75},
  {24.4, -49.75, 24.4, -46.25},
  {24.4, -46.25, -24.4, -46.25},
  {-24.4, -46.25, -24.4, -49.75},
  {20.9, -49.75, 24.4, -49.75},
  {24.4, -49.75, 24.4, -46.25},
  {24.4, -46.25, 20.9, -46.25},
  {20.9, -46.25, 20.9, -49.75},
  {-24.4, -49.75, -20.9, -49.75},
  {-20.9, -49.75, -20.9, -46.25},
  {-20.9, -46.25, -24.4, -46.25},
  {-24.4, -46.25, -24.4, -49.75},
  {-9.22774, 6.75287, 6.75287, -9.22774},
  {6.75287, -9.22774, 9.22774, -6.75287},
  {9.22774, -6.75287, -6.75287, 9.22774},
  {-6.75287, 9.22774, -9.22774, 6.75287},
  {6.75287, 9.22774, -9.22774, -6.75287},
  {-9.22774, -6.75287, -6.75287, -9.22774},
  {-6.75287, -9.22774, 9.22774, 6.75287},
  {9.22774, 6.75287, 6.75287, 9.22774},
  {-72, 45.5, -67, 45.5},
  {-67, 45.5, -67, 50.5},
  {-67, 50.5, -72, 50.5},
  {-72, 50.5, -72, 45.5},
  {67, 45.5, 72, 45.5},
  {72, 45.5, 72, 50.5},
  {72, 50.5, 67, 50.5},
  {67, 50.5, 67, 45.5},
  {-72, -50.5, -67, -50.5},
  {-67, -50.5, -67, -45.5},
  {-67, -45.5, -72, -45.5},
  {-72, -45.5, -72, -50.5},
  {67, -50.5, 72, -50.5},
  {72, -50.5, 72, -45.5},
  {72, -45.5, 67, -45.5},
  {67, -45.5, 67, -50.5},
};
static const int MAP_SEGMENT_COUNT = 52;
static const Segment PERIM_SEGMENTS[] = {
  {-72, -72, 72, -72},
  {72, -72, 72, 72},
  {72, 72, -72, 72},
  {-72, 72, -72, -72},
};
static const int PERIM_SEGMENTS_COUNT = 4;
static const Segment OBJECT_SEGMENTS[] = {
  {-24.4, 46.25, 24.4, 46.25},
  {24.4, 46.25, 24.4, 49.75},
  {24.4, 49.75, -24.4, 49.75},
  {-24.4, 49.75, -24.4, 46.25},
  {20.9, 46.25, 24.4, 46.25},
  {24.4, 46.25, 24.4, 49.75},
  {24.4, 49.75, 20.9, 49.75},
  {20.9, 49.75, 20.9, 46.25},
  {-24.4, 46.25, -20.9, 46.25},
  {-20.9, 46.25, -20.9, 49.75},
  {-20.9, 49.75, -24.4, 49.75},
  {-24.4, 49.75, -24.4, 46.25},
  {-24.4, -49.75, 24.4, -49.75},
  {24.4, -49.75, 24.4, -46.25},
  {24.4, -46.25, -24.4, -46.25},
  {-24.4, -46.25, -24.4, -49.75},
  {20.9, -49.75, 24.4, -49.75},
  {24.4, -49.75, 24.4, -46.25},
  {24.4, -46.25, 20.9, -46.25},
  {20.9, -46.25, 20.9, -49.75},
  {-24.4, -49.75, -20.9, -49.75},
  {-20.9, -49.75, -20.9, -46.25},
  {-20.9, -46.25, -24.4, -46.25},
  {-24.4, -46.25, -24.4, -49.75},
  {-9.22774, 6.75287, 6.75287, -9.22774},
  {6.75287, -9.22774, 9.22774, -6.75287},
  {9.22774, -6.75287, -6.75287, 9.22774},
  {-6.75287, 9.22774, -9.22774, 6.75287},
  {6.75287, 9.22774, -9.22774, -6.75287},
  {-9.22774, -6.75287, -6.75287, -9.22774},
  {-6.75287, -9.22774, 9.22774, 6.75287},
  {9.22774, 6.75287, 6.75287, 9.22774},
  {-72, 45.5, -67, 45.5},
  {-67, 45.5, -67, 50.5},
  {-67, 50.5, -72, 50.5},
  {-72, 50.5, -72, 45.5},
  {67, 45.5, 72, 45.5},
  {72, 45.5, 72, 50.5},
  {72, 50.5, 67, 50.5},
  {67, 50.5, 67, 45.5},
  {-72, -50.5, -67, -50.5},
  {-67, -50.5, -67, -45.5},
  {-67, -45.5, -72, -45.5},
  {-72, -45.5, -72, -50.5},
  {67, -50.5, 72, -50.5},
  {72, -50.5, 72, -45.5},
  {72, -45.5, 67, -45.5},
  {67, -45.5, 67, -50.5},
};
static const int OBJECT_SEGMENTS_COUNT = 48;
struct PolyIndex { int offset; int count; };
static const double OBJECT_POINTS[][2] = {
  {-24.4, 46.25},
  {24.4, 46.25},
  {24.4, 49.75},
  {-24.4, 49.75},
  {20.9, 46.25},
  {24.4, 46.25},
  {24.4, 49.75},
  {20.9, 49.75},
  {-24.4, 46.25},
  {-20.9, 46.25},
  {-20.9, 49.75},
  {-24.4, 49.75},
  {-24.4, -49.75},
  {24.4, -49.75},
  {24.4, -46.25},
  {-24.4, -46.25},
  {20.9, -49.75},
  {24.4, -49.75},
  {24.4, -46.25},
  {20.9, -46.25},
  {-24.4, -49.75},
  {-20.9, -49.75},
  {-20.9, -46.25},
  {-24.4, -46.25},
  {-9.22774, 6.75287},
  {6.75287, -9.22774},
  {9.22774, -6.75287},
  {-6.75287, 9.22774},
  {6.75287, 9.22774},
  {-9.22774, -6.75287},
  {-6.75287, -9.22774},
  {9.22774, 6.75287},
  {-72, 45.5},
  {-67, 45.5},
  {-67, 50.5},
  {-72, 50.5},
  {67, 45.5},
  {72, 45.5},
  {72, 50.5},
  {67, 50.5},
  {-72, -50.5},
  {-67, -50.5},
  {-67, -45.5},
  {-72, -45.5},
  {67, -50.5},
  {72, -50.5},
  {72, -45.5},
  {67, -45.5},
};
static const PolyIndex OBJECT_POLYS[] = {
  {0, 4},
  {4, 4},
  {8, 4},
  {12, 4},
  {16, 4},
  {20, 4},
  {24, 4},
  {28, 4},
  {32, 4},
  {36, 4},
  {40, 4},
  {44, 4},
};
static const int OBJECT_POLY_COUNT = 12;
static const int MAP_DIST_FIELD_W = 145;
static const int MAP_DIST_FIELD_H = 145;
static const double MAP_DIST_FIELD_RES_IN = 1;
static const double MAP_DIST_FIELD_ORIGIN_X = -72;
static const double MAP_DIST_FIELD_ORIGIN_Y = -72;
// Grid data declared in mcl_map_data.h; defined in mcl_map_data.cpp
extern const uint16_t MAP_DIST_FIELD[];
static const int MAP_DIST_FIELD_PERIM_W = 145;
static const int MAP_DIST_FIELD_PERIM_H = 145;
static const double MAP_DIST_FIELD_PERIM_RES_IN = 1;
static const double MAP_DIST_FIELD_PERIM_ORIGIN_X = -72;
static const double MAP_DIST_FIELD_PERIM_ORIGIN_Y = -72;
// Grid data declared in mcl_map_data.h; defined in mcl_map_data.cpp
extern const uint16_t MAP_DIST_FIELD_PERIM[];
static const int MAP_DIST_FIELD_OBJ_W = 145;
static const int MAP_DIST_FIELD_OBJ_H = 145;
static const double MAP_DIST_FIELD_OBJ_RES_IN = 1;
static const double MAP_DIST_FIELD_OBJ_ORIGIN_X = -72;
static const double MAP_DIST_FIELD_OBJ_ORIGIN_Y = -72;
// Grid data declared in mcl_map_data.h; defined in mcl_map_data.cpp
extern const uint16_t MAP_DIST_FIELD_OBJ[];
static const int MAP_RAY_BUCKET_NX = 12;
static const int MAP_RAY_BUCKET_NY = 12;
static const double MAP_RAY_BUCKET_CELL_IN = 12;
static const double MAP_RAY_BUCKET_ORIGIN_X = -72;
static const double MAP_RAY_BUCKET_ORIGIN_Y = -72;
static const uint32_t MAP_RAY_BUCKET_OFFSETS[] = {
  0u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 10u, 11u, 12u, 14u, 18u, 18u, 18u, 22u, 25u, 26u, 27u, 30u, 34u, 34u, 34u, 
  38u, 42u, 42u, 42u, 46u, 49u, 50u, 51u, 54u, 58u, 58u, 58u, 62u, 63u, 63u, 63u, 63u, 63u, 63u, 63u, 63u, 63u, 
  63u, 63u, 64u, 65u, 65u, 65u, 65u, 65u, 65u, 65u, 65u, 65u, 65u, 65u, 66u, 67u, 67u, 67u, 67u, 67u, 72u, 77u, 
  77u, 77u, 77u, 77u, 78u, 79u, 79u, 79u, 79u, 79u, 84u, 89u, 89u, 89u, 89u, 89u, 90u, 91u, 91u, 91u, 91u, 91u, 
  91u, 91u, 91u, 91u, 91u, 91u, 92u, 93u, 93u, 93u, 93u, 93u, 93u, 93u, 93u, 93u, 93u, 93u, 94u, 98u, 98u, 98u, 
  102u, 105u, 106u, 107u, 110u, 114u, 114u, 114u, 118u, 122u, 122u, 122u, 126u, 129u, 130u, 131u, 134u, 138u, 138u, 
  138u, 142u, 144u, 145u, 146u, 147u, 148u, 149u, 150u, 151u, 152u, 153u, 154u, 156u
};
static const uint16_t MAP_RAY_BUCKET_INDICES[] = {
  1u, 2u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 0u, 1u, 2u, 44u, 45u, 47u, 16u, 19u, 24u, 27u, 16u, 24u, 25u, 
  16u, 16u, 16u, 20u, 23u, 16u, 17u, 20u, 21u, 0u, 48u, 49u, 51u, 2u, 45u, 46u, 47u, 18u, 19u, 26u, 27u, 18u, 25u, 
  26u, 18u, 18u, 18u, 22u, 23u, 17u, 18u, 21u, 22u, 0u, 49u, 50u, 51u, 2u, 0u, 2u, 0u, 2u, 28u, 30u, 32u, 33u, 
  34u, 28u, 29u, 30u, 32u, 34u, 0u, 2u, 28u, 30u, 31u, 32u, 34u, 28u, 30u, 32u, 34u, 35u, 0u, 2u, 0u, 2u, 0u, 2u, 
  36u, 37u, 39u, 4u, 7u, 12u, 15u, 4u, 12u, 13u, 4u, 4u, 4u, 8u, 11u, 4u, 5u, 8u, 9u, 0u, 40u, 41u, 43u, 2u, 37u, 
  38u, 39u, 6u, 7u, 14u, 15u, 6u, 13u, 14u, 6u, 6u, 6u, 10u, 11u, 5u, 6u, 9u, 10u, 0u, 41u, 42u, 43u, 2u, 3u, 3u, 
  3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 0u, 3u
};
static const int PERIM_RAY_BUCKET_NX = 12;
static const int PERIM_RAY_BUCKET_NY = 12;
static const double PERIM_RAY_BUCKET_CELL_IN = 12;
static const double PERIM_RAY_BUCKET_ORIGIN_X = -72;
static const double PERIM_RAY_BUCKET_ORIGIN_Y = -72;
static const uint32_t PERIM_RAY_BUCKET_OFFSETS[] = {
  0u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 10u, 11u, 12u, 14u, 15u, 15u, 15u, 15u, 15u, 15u, 15u, 15u, 15u, 15u, 15u, 
  16u, 17u, 17u, 17u, 17u, 17u, 17u, 17u, 17u, 17u, 17u, 17u, 18u, 19u, 19u, 19u, 19u, 19u, 19u, 19u, 19u, 19u, 
  19u, 19u, 20u, 21u, 21u, 21u, 21u, 21u, 21u, 21u, 21u, 21u, 21u, 21u, 22u, 23u, 23u, 23u, 23u, 23u, 23u, 23u, 
  23u, 23u, 23u, 23u, 24u, 25u, 25u, 25u, 25u, 25u, 25u, 25u, 25u, 25u, 25u, 25u, 26u, 27u, 27u, 27u, 27u, 27u, 
  27u, 27u, 27u, 27u, 27u, 27u, 28u, 29u, 29u, 29u, 29u, 29u, 29u, 29u, 29u, 29u, 29u, 29u, 30u, 31u, 31u, 31u, 
  31u, 31u, 31u, 31u, 31u, 31u, 31u, 31u, 32u, 33u, 33u, 33u, 33u, 33u, 33u, 33u, 33u, 33u, 33u, 33u, 34u, 36u, 
  37u, 38u, 39u, 40u, 41u, 42u, 43u, 44u, 45u, 46u, 48u
};
static const uint16_t PERIM_RAY_BUCKET_INDICES[] = {
  0u, 3u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 1u, 3u, 1u, 3u, 1u, 3u, 1u, 3u, 1u, 3u, 1u, 3u, 1u, 3u, 1u, 
  3u, 1u, 3u, 1u, 3u, 1u, 2u, 3u, 2u, 2u, 2u, 2u, 2u, 2u, 2u, 2u, 2u, 2u, 1u, 2u
};
static const int OBJECT_RAY_BUCKET_NX = 12;
static const int OBJECT_RAY_BUCKET_NY = 12;
static const double OBJECT_RAY_BUCKET_CELL_IN = 12;
static const double OBJECT_RAY_BUCKET_ORIGIN_X = -72;
static const double OBJECT_RAY_BUCKET_ORIGIN_Y = -72;
static const uint32_t OBJECT_RAY_BUCKET_OFFSETS[] = {
  0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 3u, 3u, 3u, 7u, 10u, 11u, 12u, 15u, 19u, 19u, 19u, 22u, 25u, 
  25u, 25u, 29u, 32u, 33u, 34u, 37u, 41u, 41u, 41u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 
  44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 44u, 49u, 54u, 54u, 54u, 
  54u, 54u, 54u, 54u, 54u, 54u, 54u, 54u, 59u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 
  64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 64u, 67u, 67u, 67u, 71u, 74u, 
  75u, 76u, 79u, 83u, 83u, 83u, 86u, 89u, 89u, 89u, 93u, 96u, 97u, 98u, 101u, 105u, 105u, 105u, 108u, 108u, 108u, 
  108u, 108u, 108u, 108u, 108u, 108u, 108u, 108u, 108u, 108u
};
static const uint16_t OBJECT_RAY_BUCKET_INDICES[] = {
  40u, 41u, 43u, 12u, 15u, 20u, 23u, 12u, 20u, 21u, 12u, 12u, 12u, 16u, 19u, 12u, 13u, 16u, 17u, 44u, 45u, 47u, 
  41u, 42u, 43u, 14u, 15u, 22u, 23u, 14u, 21u, 22u, 14u, 14u, 14u, 18u, 19u, 13u, 14u, 17u, 18u, 45u, 46u, 47u, 
  24u, 26u, 28u, 29u, 30u, 24u, 25u, 26u, 28u, 30u, 24u, 26u, 27u, 28u, 30u, 24u, 26u, 28u, 30u, 31u, 32u, 33u, 
  35u, 0u, 3u, 8u, 11u, 0u, 8u, 9u, 0u, 0u, 0u, 4u, 7u, 0u, 1u, 4u, 5u, 36u, 37u, 39u, 33u, 34u, 35u, 2u, 3u, 10u, 
  11u, 2u, 9u, 10u, 2u, 2u, 2u, 6u, 7u, 1u, 2u, 5u, 6u, 37u, 38u, 39u
};

static std::mt19937& rng_engine() {
  static std::mt19937 eng(0u);
  return eng;
}

static double rand_uniform() {
  return std::generate_canonical<double, 53>(rng_engine());
}

static double rand_gaussian(double mean, double stddev) {
  if (stddev <= 0.0) return mean;
  std::normal_distribution<double> dist(mean, stddev);
  return dist(rng_engine());
}

static double wrap_deg(double deg) {
  while (deg >= 360.0) deg -= 360.0;
  while (deg < 0.0) deg += 360.0;
  return deg;
}

static double angle_diff_deg(double a, double b) {
  double d = std::fmod(a - b + 180.0, 360.0);
  if (d < 0.0) d += 360.0;
  return d - 180.0;
}

static double gaussian(double x, double mu, double sigma) {
  if (sigma <= 1e-9) return (std::fabs(x - mu) <= 1e-9) ? 1.0 : 0.0;
  double z = (x - mu) / sigma;
  return std::exp(-0.5 * z * z);
}

static void heading_unit(double heading_deg, double& dx, double& dy) {
  double th = heading_deg * DEG_TO_RAD;
  dx = std::sin(th);
  dy = std::cos(th);
}

static int fov_ray_count() {
  int k = MCL_DIST_RAYS_PER_SENSOR;
  if (k < 1) k = 1;
  if (k > 9) k = 9;
  return k;
}

static double fov_half_deg(double meas_mm) {
  if (!MCL_DIST_FOV_MULTI_RAY) return 0.0;
  return (meas_mm < MCL_DIST_FOV_SWITCH_MM) ? MCL_DIST_FOV_HALF_DEG_NEAR : MCL_DIST_FOV_HALF_DEG_FAR;
}

static double logmeanexp(const double* vals, int n) {
  if (!vals || n <= 0) return -1e300;
  double m = vals[0];
  for (int i = 1; i < n; ++i) if (vals[i] > m) m = vals[i];
  double s = 0.0;
  for (int i = 0; i < n; ++i) s += std::exp(vals[i] - m);
  if (s <= 1e-300) return -1e300;
  return m + std::log(s) - std::log(static_cast<double>(n));
}

static void rotate_local_to_world(double lx, double ly, double heading_deg, double& wx, double& wy) {
  double th = heading_deg * DEG_TO_RAD;
  double s = std::sin(th);
  double c = std::cos(th);
  wx = lx * s - ly * c;
  wy = lx * c + ly * s;
}

static double dist_point_seg_sq(double px, double py, double x0, double y0, double x1, double y1) {
  double dx = x1 - x0;
  double dy = y1 - y0;
  if (std::fabs(dx) < 1e-9 && std::fabs(dy) < 1e-9) {
    double ex = px - x0;
    double ey = py - y0;
    return ex * ex + ey * ey;
  }
  double t = ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy);
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  double cx = x0 + t * dx;
  double cy = y0 + t * dy;
  double ex = px - cx;
  double ey = py - cy;
  return ex * ex + ey * ey;
}

static double distance_field_lookup(const uint16_t* field, int w, int h, double res,
                                   double ox, double oy, double x, double y) {
  if (!field || w <= 1 || h <= 1 || res <= 1e-9) return -1.0;
  double fx = (x - ox) / res;
  double fy = (y - oy) / res;
  int ix = static_cast<int>(std::floor(fx));
  int iy = static_cast<int>(std::floor(fy));
  if (ix < 0) ix = 0;
  if (iy < 0) iy = 0;
  if (ix > w - 2) ix = w - 2;
  if (iy > h - 2) iy = h - 2;
  double tx = fx - ix;
  double ty = fy - iy;
  int idx = iy * w + ix;
  double d00 = static_cast<double>(field[idx]) / MCL_MM_PER_IN;
  double d10 = static_cast<double>(field[idx + 1]) / MCL_MM_PER_IN;
  double d01 = static_cast<double>(field[idx + w]) / MCL_MM_PER_IN;
  double d11 = static_cast<double>(field[idx + w + 1]) / MCL_MM_PER_IN;
  double dx0 = d00 + (d10 - d00) * tx;
  double dx1 = d01 + (d11 - d01) * tx;
  return dx0 + (dx1 - dx0) * ty;
}

static bool sensor_allows_object(int sensor_idx, int obj_idx) {
  if (obj_idx < 0 || obj_idx >= OBJECT_POLY_COUNT) return false;
  if (sensor_idx < 0 || sensor_idx >= MCL_DISTANCE_SENSOR_COUNT) return true;
  if (obj_idx >= 64) return true;
  uint64_t mask = MCL_DISTANCE_SENSORS[sensor_idx].object_mask;
  return ((mask >> obj_idx) & 1ull) != 0ull;
}

static bool sensor_has_full_object_mask(int sensor_idx) {
  if (OBJECT_POLY_COUNT <= 0) return true;
  if (sensor_idx < 0 || sensor_idx >= MCL_DISTANCE_SENSOR_COUNT) return true;
  if (OBJECT_POLY_COUNT >= 64) return true;
  uint64_t need = (1ull << OBJECT_POLY_COUNT) - 1ull;
  uint64_t have = MCL_DISTANCE_SENSORS[sensor_idx].object_mask;
  return (have & need) == need;
}

static double distance_to_objects_mask(double x, double y, int sensor_idx) {
  if (OBJECT_POLY_COUNT <= 0) return -1.0;
  bool have = false;
  double best = 1e18;
  for (int i = 0; i < OBJECT_POLY_COUNT; ++i) {
    if (!sensor_allows_object(sensor_idx, i)) continue;
    const PolyIndex& p = OBJECT_POLYS[i];
    if (p.count < 2) continue;
    have = true;
    for (int j = 0; j < p.count; ++j) {
      int k = (j + 1) % p.count;
      double x0 = OBJECT_POINTS[p.offset + j][0];
      double y0 = OBJECT_POINTS[p.offset + j][1];
      double x1 = OBJECT_POINTS[p.offset + k][0];
      double y1 = OBJECT_POINTS[p.offset + k][1];
      double d2 = dist_point_seg_sq(x, y, x0, y0, x1, y1);
      if (d2 < best) best = d2;
    }
  }
  if (!have || best >= 1e17) return -1.0;
  return std::sqrt(best);
}

static double distance_to_map_mode(double x, double y, int mode, int sensor_idx) {
  bool full_object_mask = sensor_has_full_object_mask(sensor_idx);
  if (mode == MCL_MAP_MODE_PERIMETER) {
    double d = distance_field_lookup(MAP_DIST_FIELD_PERIM, MAP_DIST_FIELD_PERIM_W, MAP_DIST_FIELD_PERIM_H,
                                     MAP_DIST_FIELD_PERIM_RES_IN, MAP_DIST_FIELD_PERIM_ORIGIN_X, MAP_DIST_FIELD_PERIM_ORIGIN_Y, x, y);
    if (d >= 0.0) return d;
    const Segment* segs = PERIM_SEGMENTS;
    int seg_count = PERIM_SEGMENTS_COUNT;
    if (seg_count <= 0) return 0.0;
    double best = 1e18;
    for (int i = 0; i < seg_count; ++i) {
      const Segment& s = segs[i];
      double d2 = dist_point_seg_sq(x, y, s.x0, s.y0, s.x1, s.y1);
      if (d2 < best) best = d2;
    }
    return std::sqrt(best);
  } else if (mode == MCL_MAP_MODE_OBJECTS) {
    if (full_object_mask) {
      double d = distance_field_lookup(MAP_DIST_FIELD_OBJ, MAP_DIST_FIELD_OBJ_W, MAP_DIST_FIELD_OBJ_H,
                                       MAP_DIST_FIELD_OBJ_RES_IN, MAP_DIST_FIELD_OBJ_ORIGIN_X, MAP_DIST_FIELD_OBJ_ORIGIN_Y, x, y);
      if (d >= 0.0) return d;
    }
    double od = distance_to_objects_mask(x, y, sensor_idx);
    return (od >= 0.0) ? od : 0.0;
  }
  if (full_object_mask) {
    double d = distance_field_lookup(MAP_DIST_FIELD, MAP_DIST_FIELD_W, MAP_DIST_FIELD_H,
                                     MAP_DIST_FIELD_RES_IN, MAP_DIST_FIELD_ORIGIN_X, MAP_DIST_FIELD_ORIGIN_Y, x, y);
    if (d >= 0.0) return d;
  }
  double best = 1e18;
  bool have = false;
  if (PERIM_SEGMENTS_COUNT > 0) {
    have = true;
    for (int i = 0; i < PERIM_SEGMENTS_COUNT; ++i) {
      const Segment& s = PERIM_SEGMENTS[i];
      double d2 = dist_point_seg_sq(x, y, s.x0, s.y0, s.x1, s.y1);
      if (d2 < best) best = d2;
    }
  }
  double od = distance_to_objects_mask(x, y, sensor_idx);
  if (od >= 0.0) {
    have = true;
    double od2 = od * od;
    if (od2 < best) best = od2;
  }
  if (!have || best >= 1e17) return 0.0;
  return std::sqrt(best);
}

static bool ray_segment_intersect(double ox, double oy, double dx, double dy,
                                  double x0, double y0, double x1, double y1, double& t_out) {
  double rx = dx, ry = dy;
  double sx = x1 - x0, sy = y1 - y0;
  double rxs = rx * sy - ry * sx;
  if (std::fabs(rxs) < 1e-9) return false;
  double qpx = x0 - ox, qpy = y0 - oy;
  double t = (qpx * sy - qpy * sx) / rxs;
  double u = (qpx * ry - qpy * rx) / rxs;
  if (t >= 0.0 && u >= 0.0 && u <= 1.0) {
    t_out = t;
    return true;
  }
  return false;
}

static double raycast_distance_segments(const Segment* segs, int count, double ox, double oy,
                                        double dx, double dy, double max_dist_in) {
  double best = -1.0;
  for (int i = 0; i < count; ++i) {
    double t = 0.0;
    if (!ray_segment_intersect(ox, oy, dx, dy, segs[i].x0, segs[i].y0, segs[i].x1, segs[i].y1, t)) {
      continue;
    }
    if (t <= max_dist_in && (best < 0.0 || t < best)) best = t;
  }
  return best;
}

struct RayBucketView {
  const uint32_t* offsets;
  const uint16_t* indices;
  int nx;
  int ny;
  double cell_in;
  double origin_x;
  double origin_y;
};

static double raycast_distance_segments_bucketed(const Segment* segs, int count, const RayBucketView& view,
                                                 double ox, double oy, double dx, double dy, double max_dist_in) {
  if (!segs || count <= 0) return -1.0;
  if (!view.offsets || !view.indices || view.nx <= 0 || view.ny <= 0 || view.cell_in <= 1e-6) {
    return raycast_distance_segments(segs, count, ox, oy, dx, dy, max_dist_in);
  }
  double best = -1.0;
  double step = std::max(0.25, view.cell_in * 0.5);
  int steps = static_cast<int>(std::ceil(max_dist_in / step));
  if (steps < 1) steps = 1;
  int last_cell = -1;
  for (int st = 0; st <= steps; ++st) {
    double d = std::min(max_dist_in, step * static_cast<double>(st));
    if (best >= 0.0 && d > best + view.cell_in) break;
    double x = ox + dx * d;
    double y = oy + dy * d;
    int ix = static_cast<int>(std::floor((x - view.origin_x) / view.cell_in));
    int iy = static_cast<int>(std::floor((y - view.origin_y) / view.cell_in));
    if (ix < 0 || iy < 0 || ix >= view.nx || iy >= view.ny) continue;
    int cell = iy * view.nx + ix;
    if (cell == last_cell) continue;
    last_cell = cell;
    uint32_t begin = view.offsets[cell];
    uint32_t end = view.offsets[cell + 1];
    for (uint32_t k = begin; k < end; ++k) {
      int si = static_cast<int>(view.indices[k]);
      if (si < 0 || si >= count) continue;
      double t = 0.0;
      if (!ray_segment_intersect(ox, oy, dx, dy, segs[si].x0, segs[si].y0, segs[si].x1, segs[si].y1, t)) continue;
      if (t <= max_dist_in && (best < 0.0 || t < best)) best = t;
    }
  }
  if (best >= 0.0) return best;
  return raycast_distance_segments(segs, count, ox, oy, dx, dy, max_dist_in);
}

static double raycast_distance_objects_mask(double ox, double oy, double dx, double dy, double max_dist_in, int sensor_idx) {
  if (OBJECT_POLY_COUNT <= 0) return -1.0;
  double best = -1.0;
  for (int i = 0; i < OBJECT_POLY_COUNT; ++i) {
    if (!sensor_allows_object(sensor_idx, i)) continue;
    const PolyIndex& p = OBJECT_POLYS[i];
    if (p.count < 2) continue;
    for (int j = 0; j < p.count; ++j) {
      int k = (j + 1) % p.count;
      double x0 = OBJECT_POINTS[p.offset + j][0];
      double y0 = OBJECT_POINTS[p.offset + j][1];
      double x1 = OBJECT_POINTS[p.offset + k][0];
      double y1 = OBJECT_POINTS[p.offset + k][1];
      double t = 0.0;
      if (!ray_segment_intersect(ox, oy, dx, dy, x0, y0, x1, y1, t)) continue;
      if (t <= max_dist_in && (best < 0.0 || t < best)) best = t;
    }
  }
  return best;
}

static double raycast_distance_mode(double ox, double oy, double heading_deg, double max_dist_in, int mode, int sensor_idx) {
  double dx = 0.0, dy = 0.0;
  heading_unit(heading_deg, dx, dy);
  bool full_object_mask = sensor_has_full_object_mask(sensor_idx);
  const RayBucketView perim_view = {PERIM_RAY_BUCKET_OFFSETS, PERIM_RAY_BUCKET_INDICES, PERIM_RAY_BUCKET_NX, PERIM_RAY_BUCKET_NY, PERIM_RAY_BUCKET_CELL_IN, PERIM_RAY_BUCKET_ORIGIN_X, PERIM_RAY_BUCKET_ORIGIN_Y};
  const RayBucketView obj_view = {OBJECT_RAY_BUCKET_OFFSETS, OBJECT_RAY_BUCKET_INDICES, OBJECT_RAY_BUCKET_NX, OBJECT_RAY_BUCKET_NY, OBJECT_RAY_BUCKET_CELL_IN, OBJECT_RAY_BUCKET_ORIGIN_X, OBJECT_RAY_BUCKET_ORIGIN_Y};
  double best = -1.0;
  if (mode == MCL_MAP_MODE_PERIMETER) {
    best = raycast_distance_segments_bucketed(PERIM_SEGMENTS, PERIM_SEGMENTS_COUNT, perim_view, ox, oy, dx, dy, max_dist_in);
  } else if (mode == MCL_MAP_MODE_OBJECTS) {
    if (full_object_mask) {
      best = raycast_distance_segments_bucketed(OBJECT_SEGMENTS, OBJECT_SEGMENTS_COUNT, obj_view, ox, oy, dx, dy, max_dist_in);
    } else {
      best = raycast_distance_objects_mask(ox, oy, dx, dy, max_dist_in, sensor_idx);
    }
  } else {
    best = raycast_distance_segments_bucketed(PERIM_SEGMENTS, PERIM_SEGMENTS_COUNT, perim_view, ox, oy, dx, dy, max_dist_in);
    double o = full_object_mask ?
      raycast_distance_segments_bucketed(OBJECT_SEGMENTS, OBJECT_SEGMENTS_COUNT, obj_view, ox, oy, dx, dy, max_dist_in) :
      raycast_distance_objects_mask(ox, oy, dx, dy, max_dist_in, sensor_idx);
    if (o >= 0.0 && (best < 0.0 || o < best)) best = o;
  }
  return best;
}

static bool point_in_poly(double x, double y, int offset, int count) {
  bool c = false;
  for (int i = 0, j = count - 1; i < count; j = i++) {
    double xi = OBJECT_POINTS[offset + i][0];
    double yi = OBJECT_POINTS[offset + i][1];
    double xj = OBJECT_POINTS[offset + j][0];
    double yj = OBJECT_POINTS[offset + j][1];
    bool intersect = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi);
    if (intersect) c = !c;
  }
  return c;
}

static bool in_object(double x, double y) {
  for (int i = 0; i < OBJECT_POLY_COUNT; ++i) {
    const PolyIndex& p = OBJECT_POLYS[i];
    if (p.count <= 0) continue;
    if (point_in_poly(x, y, p.offset, p.count)) return true;
  }
  return false;
}

static double dist_sq_point_segment(double px, double py, double x0, double y0, double x1, double y1) {
  double dx = x1 - x0;
  double dy = y1 - y0;
  double denom = dx * dx + dy * dy;
  if (denom <= 1e-12) {
    double ox = px - x0;
    double oy = py - y0;
    return ox * ox + oy * oy;
  }
  double t = ((px - x0) * dx + (py - y0) * dy) / denom;
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  double cx = x0 + t * dx;
  double cy = y0 + t * dy;
  double ox = px - cx;
  double oy = py - cy;
  return ox * ox + oy * oy;
}

static double poly_edge_dist(double x, double y, int offset, int count) {
  if (count < 2) return 0.0;
  double best = 1e18;
  for (int i = 0; i < count; ++i) {
    int j = (i + 1) % count;
    double xi = OBJECT_POINTS[offset + i][0];
    double yi = OBJECT_POINTS[offset + i][1];
    double xj = OBJECT_POINTS[offset + j][0];
    double yj = OBJECT_POINTS[offset + j][1];
    double d2 = dist_sq_point_segment(x, y, xi, yi, xj, yj);
    if (d2 < best) best = d2;
  }
  return (best <= 0.0) ? 0.0 : std::sqrt(best);
}

static void robot_corners(double x, double y, double heading_deg, double out[4][2]) {
  double hw = MCL_BOT_WIDTH_IN * 0.5;
  double hl = MCL_BOT_LENGTH_IN * 0.5;
  double off_x = 0.0, off_y = 0.0;
  rotate_local_to_world(MCL_BOT_OFFSET_X_IN, MCL_BOT_OFFSET_Y_IN, heading_deg, off_x, off_y);
  const double local[4][2] = {{-hl, -hw}, {hl, -hw}, {hl, hw}, {-hl, hw}};
  for (int i = 0; i < 4; ++i) {
    double rx = 0.0, ry = 0.0;
    rotate_local_to_world(local[i][0], local[i][1], heading_deg, rx, ry);
    out[i][0] = x + off_x + rx;
    out[i][1] = y + off_y + ry;
  }
}

static bool point_in_quad(double x, double y, const double q[4][2]) {
  bool c = false;
  for (int i = 0, j = 3; i < 4; j = i++) {
    double xi = q[i][0], yi = q[i][1];
    double xj = q[j][0], yj = q[j][1];
    bool intersect = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi);
    if (intersect) c = !c;
  }
  return c;
}

static double orient2d(double ax, double ay, double bx, double by, double cx, double cy) {
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

static bool on_segment(double ax, double ay, double bx, double by, double px, double py) {
  return px >= std::min(ax, bx) - 1e-9 && px <= std::max(ax, bx) + 1e-9 &&
         py >= std::min(ay, by) - 1e-9 && py <= std::max(ay, by) + 1e-9;
}

static bool segments_intersect(double ax, double ay, double bx, double by,
                               double cx, double cy, double dx, double dy) {
  double o1 = orient2d(ax, ay, bx, by, cx, cy);
  double o2 = orient2d(ax, ay, bx, by, dx, dy);
  double o3 = orient2d(cx, cy, dx, dy, ax, ay);
  double o4 = orient2d(cx, cy, dx, dy, bx, by);
  if ((o1 > 0.0) != (o2 > 0.0) && (o3 > 0.0) != (o4 > 0.0)) return true;
  if (std::fabs(o1) <= 1e-9 && on_segment(ax, ay, bx, by, cx, cy)) return true;
  if (std::fabs(o2) <= 1e-9 && on_segment(ax, ay, bx, by, dx, dy)) return true;
  if (std::fabs(o3) <= 1e-9 && on_segment(cx, cy, dx, dy, ax, ay)) return true;
  if (std::fabs(o4) <= 1e-9 && on_segment(cx, cy, dx, dy, bx, by)) return true;
  return false;
}

static double object_clip_overlap(double x, double y, double heading_deg) {
  if (OBJECT_POLY_COUNT <= 0) return 0.0;
  double corners[4][2];
  robot_corners(x, y, heading_deg, corners);
  double max_pen = 0.0;
  for (int i = 0; i < OBJECT_POLY_COUNT; ++i) {
    const PolyIndex& p = OBJECT_POLYS[i];
    if (p.count <= 0) continue;
    bool overlap = false;
    double poly_pen = 0.0;
    for (int c = 0; c < 4; ++c) {
      double cx = corners[c][0];
      double cy = corners[c][1];
      if (point_in_poly(cx, cy, p.offset, p.count)) {
        overlap = true;
        double dist = poly_edge_dist(cx, cy, p.offset, p.count);
        if (dist > poly_pen) poly_pen = dist;
      }
    }
    if (!overlap) {
      for (int v = 0; v < p.count; ++v) {
        double vx = OBJECT_POINTS[p.offset + v][0];
        double vy = OBJECT_POINTS[p.offset + v][1];
        if (point_in_quad(vx, vy, corners)) {
          overlap = true;
          break;
        }
      }
    }
    if (!overlap) {
      for (int e = 0; e < p.count && !overlap; ++e) {
        int en = (e + 1) % p.count;
        double ex0 = OBJECT_POINTS[p.offset + e][0];
        double ey0 = OBJECT_POINTS[p.offset + e][1];
        double ex1 = OBJECT_POINTS[p.offset + en][0];
        double ey1 = OBJECT_POINTS[p.offset + en][1];
        for (int r = 0; r < 4; ++r) {
          int rn = (r + 1) % 4;
          double rx0 = corners[r][0];
          double ry0 = corners[r][1];
          double rx1 = corners[rn][0];
          double ry1 = corners[rn][1];
          if (segments_intersect(ex0, ey0, ex1, ey1, rx0, ry0, rx1, ry1)) {
            overlap = true;
            break;
          }
        }
      }
    }
    if (overlap && poly_pen < 1e-3) poly_pen = 0.01;
    if (poly_pen > max_pen) max_pen = poly_pen;
  }
  return max_pen;
}

static double object_clip_weight(double x, double y, double heading_deg) {
  if (MCL_OBJECT_MODE <= 0) return 1.0;
  double overlap = object_clip_overlap(x, y, heading_deg);
  if (overlap <= 0.0) return 1.0;
  if (MCL_OBJECT_MODE >= 2) return 0.0;
  double free_in = MCL_OBJECT_CLIP_FREE_IN;
  double max_in = MCL_OBJECT_CLIP_MAX_IN;
  double sigma_in = MCL_OBJECT_CLIP_SIGMA_IN;
  if (overlap <= free_in) return 1.0;
  if (max_in <= free_in || sigma_in <= 1e-6) return 0.0;
  if (overlap >= max_in) return 0.0;
  double z = (overlap - free_in) / sigma_in;
  return std::exp(-0.5 * z * z);
}

static bool pose_outside_perimeter(double x, double y, double heading_deg) {
  double corners[4][2];
  robot_corners(x, y, heading_deg, corners);
  for (int i = 0; i < 4; ++i) {
    double cx = corners[i][0];
    double cy = corners[i][1];
    if (cx < -MCL_FIELD_HALF_HEIGHT_IN || cx > MCL_FIELD_HALF_HEIGHT_IN ||
        cy < -MCL_FIELD_HALF_WIDTH_IN || cy > MCL_FIELD_HALF_WIDTH_IN) {
      return true;
    }
  }
  return false;
}

static double polyline_dist_sq(double x, double y, const std::vector<MCLPose>& pts) {
  if (pts.size() < 2) {
    if (pts.empty()) return 1e18;
    double dx = x - pts[0].x;
    double dy = y - pts[0].y;
    return dx * dx + dy * dy;
  }
  double best = 1e18;
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    double d2 = dist_sq_point_segment(x, y, pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y);
    if (d2 < best) best = d2;
  }
  return best;
}

struct PoseSample { double x; double y; double theta; };

static PoseSample sample_random_pose() {
  double x_min = MCL_REGION_X_MIN_IN;
  double y_min = MCL_REGION_Y_MIN_IN;
  double x_max = MCL_REGION_X_MAX_IN;
  double y_max = MCL_REGION_Y_MAX_IN;
  if (x_max <= x_min) { x_min = -MCL_FIELD_HALF_HEIGHT_IN; x_max = MCL_FIELD_HALF_HEIGHT_IN; }
  if (y_max <= y_min) { y_min = -MCL_FIELD_HALF_WIDTH_IN; y_max = MCL_FIELD_HALF_WIDTH_IN; }
  int attempts = std::max(1, MCL_REGION_SAMPLE_ATTEMPTS);
  for (int i = 0; i < attempts; ++i) {
    double x = x_min + rand_uniform() * (x_max - x_min);
    double y = y_min + rand_uniform() * (y_max - y_min);
    double th = rand_uniform() * 360.0;
    if (MCL_REGION_PERIMETER_GATE && pose_outside_perimeter(x, y, th)) continue;
    return {x, y, th};
  }
  for (int i = 0; i < attempts; ++i) {
    double x = -MCL_FIELD_HALF_HEIGHT_IN + rand_uniform() * (MCL_FIELD_HALF_HEIGHT_IN * 2.0);
    double y = -MCL_FIELD_HALF_WIDTH_IN + rand_uniform() * (MCL_FIELD_HALF_WIDTH_IN * 2.0);
    double th = rand_uniform() * 360.0;
    if (MCL_REGION_PERIMETER_GATE && pose_outside_perimeter(x, y, th)) continue;
    return {x, y, th};
  }
  return {0.0, 0.0, rand_uniform() * 360.0};
}


static double inv_norm_cdf(double p) {
  if (p <= 0.0) return -1e9;
  if (p >= 1.0) return 1e9;
  static const double a[] = {
    -3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02,
    1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00
  };
  static const double b[] = {
    -5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,
    6.680131188771972e+01, -1.328068155288572e+01
  };
  static const double c[] = {
    -7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
    -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00
  };
  static const double d[] = {
    7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
    3.754408661907416e+00
  };
  double plow = 0.02425;
  double phigh = 1.0 - plow;
  if (p < plow) {
    double q = std::sqrt(-2.0 * std::log(p));
    return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
      ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
  }
  if (p > phigh) {
    double q = std::sqrt(-2.0 * std::log(1.0 - p));
    return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
      ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
  }
  double q = p - 0.5;
  double r = q * q;
  return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q /
    (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
}

static int kld_required_particles(int k, double epsilon, double delta) {
  if (k <= 1) return 1;
  double p = (delta >= 0.5) ? delta : (1.0 - delta);
  double z = inv_norm_cdf(p);
  double km = k - 1.0;
  double frac = 1.0 - 2.0 / (9.0 * km) + z * std::sqrt(2.0 / (9.0 * km));
  double n = (km / (2.0 * epsilon)) * (frac * frac * frac);
  return static_cast<int>(std::ceil(n));
}

static uint64_t mix_u64(uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}

static uint64_t bin_index(double x, double y, double theta) {
  double bx = std::max(1e-6, MCL_KLD_BIN_XY_IN);
  double by = bx;
  double bt = std::max(1e-6, MCL_KLD_BIN_THETA_DEG);
  int64_t ix = static_cast<int64_t>(std::floor(x / bx));
  int64_t iy = static_cast<int64_t>(std::floor(y / by));
  int64_t it = static_cast<int64_t>(std::floor(wrap_deg(theta) / bt));
  uint64_t h = 0;
  h ^= mix_u64(static_cast<uint64_t>(ix));
  h ^= mix_u64(static_cast<uint64_t>(iy) + 0x9e3779b97f4a7c15ULL);
  h ^= mix_u64(static_cast<uint64_t>(it) + 0xbf58476d1ce4e5b9ULL);
  return h;
}
}  // namespace

double MCLLocalizer::regionWeight(double x, double y, double heading_deg) const {
  double weight = 1.0;
  if (MCL_REGION_PERIMETER_GATE) {
    if (pose_outside_perimeter(x, y, heading_deg)) return 0.0;
  }
  double obj_w = object_clip_weight(x, y, heading_deg);
  if (obj_w <= 0.0) return 0.0;
  weight *= obj_w;
  if (!MCL_REGION_ENABLED) return weight;
  if (segment_band_active_ && segment_band_radius_ > 0.0 && segment_band_pts_.size() >= 2) {
    double d2 = polyline_dist_sq(x, y, segment_band_pts_);
    if (d2 > segment_band_radius_ * segment_band_radius_) {
      if (MCL_REGION_MODE == MCL_REGION_MODE_HARD) return 0.0;
      weight *= MCL_REGION_PENALTY;
    }
    return weight;
  }
  if (x < MCL_REGION_X_MIN_IN || x > MCL_REGION_X_MAX_IN ||
      y < MCL_REGION_Y_MIN_IN || y > MCL_REGION_Y_MAX_IN) {
    if (MCL_REGION_MODE == MCL_REGION_MODE_HARD) return 0.0;
    weight *= MCL_REGION_PENALTY;
  }
  return weight;
}

MCLLocalizer::MCLLocalizer()
  : count_(MCL_PARTICLE_COUNT), w_slow_(0.0), w_fast_(0.0), forced_injection_fraction_(0.0), confidence_(0.0), ess_ratio_(1.0),
    estimate_valid_(false), have_estimate_ever_(false), last_dist_used_mask_(0), segment_band_radius_(0.0),
    segment_band_active_(false) {
  if (MCL_KLD_ENABLED) count_ = MCL_N_MIN;
  for (int i = 0; i < count_; ++i) {
    particles_[i] = {0.0, 0.0, 0.0, 1.0 / std::max(1, count_)};
  }
  for (int i = 0; i < MCL_DISTANCE_SENSOR_COUNT_SAFE; ++i) {
    last_dist_measured_mm_[i] = -1.0;
    last_dist_expected_mm_[i] = -1.0;
    last_dist_errno_[i] = 0;
  }
}

void MCLLocalizer::seed(unsigned int seed) {
  rng_engine().seed(seed);
}

void MCLLocalizer::init(double start_x, double start_y, double start_theta) {
  count_ = MCL_KLD_ENABLED ? MCL_N_MIN : MCL_PARTICLE_COUNT;
  double base_x = start_x;
  double base_y = start_y;
  double base_th = start_theta;
  int attempts = std::max(1, MCL_REGION_SAMPLE_ATTEMPTS);
  estimate_valid_ = false;
  have_estimate_ever_ = false;
  confidence_ = 0.0;
  ess_ratio_ = 1.0;
  for (int i = 0; i < count_; ++i) {
    bool placed = false;
    int tries = MCL_REGION_PERIMETER_GATE ? attempts : 1;
    for (int a = 0; a < tries; ++a) {
      double x = base_x + rand_gaussian(0.0, MCL_SETPOSE_SIGMA_XY_IN);
      double y = base_y + rand_gaussian(0.0, MCL_SETPOSE_SIGMA_XY_IN);
      double th = wrap_deg(base_th + rand_gaussian(0.0, MCL_SETPOSE_SIGMA_THETA_DEG));
      if (MCL_REGION_PERIMETER_GATE && pose_outside_perimeter(x, y, th)) continue;
      particles_[i].x = x;
      particles_[i].y = y;
      particles_[i].theta = th;
      placed = true;
      break;
    }
    if (!placed) {
      PoseSample p = sample_random_pose();
      particles_[i].x = p.x;
      particles_[i].y = p.y;
      particles_[i].theta = p.theta;
    }
    particles_[i].w = 1.0 / std::max(1, count_);
  }
}

void MCLLocalizer::initGlobal() {
  count_ = MCL_KLD_ENABLED ? MCL_N_MIN : MCL_PARTICLE_COUNT;
  estimate_valid_ = false;
  have_estimate_ever_ = false;
  confidence_ = 0.0;
  ess_ratio_ = 1.0;
  for (int i = 0; i < count_; ++i) {
    PoseSample p = sample_random_pose();
    particles_[i].x = p.x;
    particles_[i].y = p.y;
    particles_[i].theta = p.theta;
    particles_[i].w = 1.0 / std::max(1, count_);
  }
}

void MCLLocalizer::setSegmentBand(const MCLPose* pts, int n, double radius_in) {
  segment_band_pts_.clear();
  segment_band_radius_ = 0.0;
  segment_band_active_ = false;
  if (!pts || n <= 0 || radius_in <= 0.0) return;
  segment_band_pts_.assign(pts, pts + n);
  segment_band_radius_ = std::max(0.0, radius_in);
  segment_band_active_ = (segment_band_pts_.size() >= 2 && segment_band_radius_ > 0.0);
}

void MCLLocalizer::clearSegmentBand() {
  segment_band_pts_.clear();
  segment_band_radius_ = 0.0;
  segment_band_active_ = false;
}

void MCLLocalizer::predict(double dx_in, double dy_in, double dtheta_deg, double noise_scale) {
  if (!MCL_USE_MOTION) return;
  if (std::fabs(dx_in) + std::fabs(dy_in) + std::fabs(dtheta_deg) <= 1e-9) return;
  estimate_valid_ = false;
  double ns = std::isfinite(noise_scale) ? noise_scale : 1.0;
  if (ns < 1.0) ns = 1.0;
  double dist = std::sqrt(dx_in * dx_in + dy_in * dy_in);
  for (int i = 0; i < count_; ++i) {
    double base_th = particles_[i].theta;
    double ndx = dx_in;
    double ndy = dy_in;
    double nth = dtheta_deg;
    if (MCL_USE_ALPHA_MODEL) {
      double sigma_trans = (MCL_ALPHA1 * dist + MCL_ALPHA2 * std::fabs(dtheta_deg)) * ns;
      double sigma_rot = (MCL_ALPHA3 * dist + MCL_ALPHA4 * std::fabs(dtheta_deg)) * ns;
      ndx += rand_gaussian(0.0, sigma_trans);
      ndy += rand_gaussian(0.0, sigma_trans);
      nth += rand_gaussian(0.0, sigma_rot);
    } else {
      ndx += rand_gaussian(0.0, MCL_MOTION_SIGMA_X_IN * ns);
      ndy += rand_gaussian(0.0, MCL_MOTION_SIGMA_Y_IN * ns);
      nth += rand_gaussian(0.0, MCL_MOTION_SIGMA_THETA_DEG * ns);
    }
    double wx = 0.0, wy = 0.0;
    rotate_local_to_world(ndx, ndy, base_th, wx, wy);
    double new_x = particles_[i].x + wx;
    double new_y = particles_[i].y + wy;
    double new_th = wrap_deg(base_th + nth);
    if (MCL_REGION_PERIMETER_GATE && pose_outside_perimeter(new_x, new_y, new_th)) {
      particles_[i].x = new_x;
      particles_[i].y = new_y;
      particles_[i].theta = new_th;
      particles_[i].w = 0.0;
      continue;
    }
    particles_[i].x = new_x;
    particles_[i].y = new_y;
    particles_[i].theta = new_th;
  }
}

void MCLLocalizer::updateDistance(const double* dist_mm, const double* dist_conf, const int* conf_meaningful, const double* dist_obj_size, const int* obj_size_valid, const int* dist_errno, int count) {
  if (!MCL_USE_DISTANCE || !dist_mm || count <= 0) return;
  (void)dist_obj_size;
  (void)obj_size_valid;
  bool have_est = have_estimate_ever_;
  MCLPose est = estimate_;
  if (have_est) est = estimate();
  estimate_valid_ = false;
  int used = std::min(count, MCL_DISTANCE_SENSOR_COUNT);
  last_dist_used_mask_ = 0;
  for (int i = 0; i < MCL_DISTANCE_SENSOR_COUNT_SAFE; ++i) {
    last_dist_measured_mm_[i] = -1.0;
    last_dist_expected_mm_[i] = -1.0;
    last_dist_errno_[i] = 0;
  }
  for (int i = 0; i < used; ++i) {
    if (dist_mm[i] >= 0.0 && dist_mm[i] < 9000.0) last_dist_measured_mm_[i] = dist_mm[i];
    if (dist_errno) last_dist_errno_[i] = dist_errno[i];
  }
  for (int i = 0; i < used; ++i) skip_sensor_[i] = 0;
  double neff = effectiveN();
  double localized = (count_ > 0)
    ? (1.0 - (neff / std::max(1.0, static_cast<double>(count_))))
    : 0.0;
  double gate_conf = MCL_CONFIDENCE_THRESHOLD;
  bool allow_skip = (MCL_DISTANCE_MODEL == MCL_DISTANCE_MODEL_BEAM) &&
    (MCL_DIST_GATE_MM > 0.0) && (MCL_DIST_GATE_REJECT_RATIO > 0.0) &&
    (count_ > 0) && (gate_conf > 0.0) && (localized >= gate_conf);
  bool allow_innovation_skip = have_est && (MCL_DIST_INNOVATION_GATE_MM > 0.0);
  if (allow_innovation_skip && MCL_DIST_INNOVATION_GATE_MIN_CONF > 0.0) {
    allow_innovation_skip = (confidence_ >= MCL_DIST_INNOVATION_GATE_MIN_CONF);
  }
  if (allow_innovation_skip) {
    for (int s = 0; s < used; ++s) {
      if (skip_sensor_[s]) continue;
      const MCLDistanceSensorConfig& cfg = MCL_DISTANCE_SENSORS[s];
      double meas_raw = dist_mm[s];
      if (!std::isfinite(meas_raw) || meas_raw < 0.0 || meas_raw >= 9000.0) {
        skip_sensor_[s] = 1;
        continue;
      }
      double meas = meas_raw - cfg.bias_mm;
      double s_min = (cfg.min_range_mm > 0.0) ? cfg.min_range_mm : MCL_DIST_MIN_RANGE_MM;
      double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;
      if (s_max <= 0.0) s_max = MCL_DIST_MAX_RANGE_MM;
      if (s_min <= 0.0) s_min = 20.0;
      if (s_min > 0.0 && meas < s_min) {
        skip_sensor_[s] = 1;
        continue;
      }
      if (meas > s_max) {
        skip_sensor_[s] = 1;
        continue;
      }
      if (MCL_LF_IGNORE_MAX && MCL_DISTANCE_MODEL == MCL_DISTANCE_MODEL_LIKELIHOOD_FIELD &&
          meas >= s_max) {
        skip_sensor_[s] = 1;
        continue;
      }
      double off_x = 0.0, off_y = 0.0;
      rotate_local_to_world(cfg.x_in, cfg.y_in, est.theta, off_x, off_y);
      double ox = est.x + off_x;
      double oy = est.y + off_y;
      double heading = wrap_deg(est.theta + cfg.angle_deg + cfg.angle_offset_deg);
      double expected_in = raycast_distance_mode(ox, oy, heading, s_max / MCL_MM_PER_IN, cfg.map_mode, s);
      if (expected_in < 0.0) expected_in = s_max / MCL_MM_PER_IN;
      double expected_mm = expected_in * MCL_MM_PER_IN;
      double gate_mm = (cfg.innovation_gate_mm > 0.0) ? cfg.innovation_gate_mm : MCL_DIST_INNOVATION_GATE_MM;
      if (gate_mm > 0.0 && std::fabs(meas - expected_mm) > gate_mm) {
        skip_sensor_[s] = 1;
      }
    }
  }
  if (allow_skip) {
    for (int s = 0; s < used; ++s) {
      const MCLDistanceSensorConfig& cfg = MCL_DISTANCE_SENSORS[s];
      double meas_raw = dist_mm[s];
      if (!std::isfinite(meas_raw) || meas_raw < 0.0 || meas_raw >= 9000.0) {
        skip_sensor_[s] = 1;
        continue;
      }
      double meas = meas_raw - cfg.bias_mm;
      double s_min = (cfg.min_range_mm > 0.0) ? cfg.min_range_mm : MCL_DIST_MIN_RANGE_MM;
      double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;
      if (s_max <= 0.0) s_max = MCL_DIST_MAX_RANGE_MM;
      if (s_min <= 0.0) s_min = 20.0;
      if (s_min > 0.0 && meas < s_min) {
        skip_sensor_[s] = 1;
        continue;
      }
      if (meas > s_max) {
        skip_sensor_[s] = 1;
        continue;
      }
      int gated = 0;
      for (int i = 0; i < count_; ++i) {
        double off_x = 0.0, off_y = 0.0;
        rotate_local_to_world(cfg.x_in, cfg.y_in, particles_[i].theta, off_x, off_y);
        double ox = particles_[i].x + off_x;
        double oy = particles_[i].y + off_y;
        double heading = wrap_deg(particles_[i].theta + cfg.angle_deg + cfg.angle_offset_deg);
        double expected_in = raycast_distance_mode(ox, oy, heading, s_max / MCL_MM_PER_IN, cfg.map_mode, s);
        if (expected_in < 0.0) expected_in = s_max / MCL_MM_PER_IN;
        double expected_mm = expected_in * MCL_MM_PER_IN;
        if (std::fabs(meas - expected_mm) > MCL_DIST_GATE_MM) {
          gated++;
        }
      }
      if (gated >= static_cast<int>(MCL_DIST_GATE_REJECT_RATIO * count_)) {
        skip_sensor_[s] = 1;
      }
    }
  }
  double max_log_w = -1e300;
  for (int i = 0; i < count_; ++i) {
    double w = particles_[i].w;
    if (w < 1e-300) w = 1e-300;
    double log_w = std::log(w);
    for (int s = 0; s < used; ++s) {
      if (skip_sensor_[s]) continue;
      const MCLDistanceSensorConfig& cfg = MCL_DISTANCE_SENSORS[s];
      double meas_raw = dist_mm[s];
      if (!std::isfinite(meas_raw) || meas_raw < 0.0 || meas_raw >= 9000.0) {
        continue;
      }
      double meas = meas_raw - cfg.bias_mm;
      double s_min = (cfg.min_range_mm > 0.0) ? cfg.min_range_mm : MCL_DIST_MIN_RANGE_MM;
      double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;
      if (s_max <= 0.0) s_max = MCL_DIST_MAX_RANGE_MM;
      if (s_min <= 0.0) s_min = 20.0;
      if (s_min > 0.0 && meas < s_min) {
        continue;
      }
      if (MCL_LF_IGNORE_MAX && MCL_DISTANCE_MODEL == MCL_DISTANCE_MODEL_LIKELIHOOD_FIELD &&
          meas >= s_max) {
        continue;
      }
      if (meas > s_max) {
        continue;
      }
      double off_x = 0.0, off_y = 0.0;
      rotate_local_to_world(cfg.x_in, cfg.y_in, particles_[i].theta, off_x, off_y);
      double ox = particles_[i].x + off_x;
      double oy = particles_[i].y + off_y;
      double heading = wrap_deg(particles_[i].theta + cfg.angle_deg + cfg.angle_offset_deg);
      double sigma_eff = (meas <= 200.0) ? MCL_DIST_SIGMA_HIT_MM : (MCL_DIST_SIGMA_FAR_SCALE * meas);
      if (sigma_eff < MCL_DIST_SIGMA_MIN_MM) sigma_eff = MCL_DIST_SIGMA_MIN_MM;
      if (sigma_eff > MCL_DIST_SIGMA_MAX_MM) sigma_eff = MCL_DIST_SIGMA_MAX_MM;
      if (dist_conf && conf_meaningful && conf_meaningful[s] == 1) {
        double conf_min_map = MCL_DIST_CONFIDENCE_MIN;
        if (conf_min_map >= 63.0) conf_min_map = 62.0;
        if (conf_min_map < 0.0) conf_min_map = 0.0;
        double denom = 63.0 - conf_min_map;
        if (denom > 1e-9) {
          double q = (dist_conf[s] - conf_min_map) / denom;
          if (q < 0.0) q = 0.0;
          if (q > 1.0) q = 1.0;
          sigma_eff *= (1.0 + MCL_DIST_CONF_SIGMA_SCALE * (1.0 - q));
        }
      }
      int ray_count = (MCL_DIST_FOV_MULTI_RAY ? fov_ray_count() : 1);
      double half_deg = fov_half_deg(meas);
      double logp_rays[9];
      int ray_used = 0;
      for (int r = 0; r < ray_count; ++r) {
        double delta = 0.0;
        if (ray_count > 1) {
          delta = -half_deg + (2.0 * half_deg * static_cast<double>(r) / static_cast<double>(ray_count - 1));
        }
        double ray_heading = wrap_deg(heading + delta);
        double w_ray = 1.0;
        if (MCL_DISTANCE_MODEL == MCL_DISTANCE_MODEL_LIKELIHOOD_FIELD) {
          double meas_in = meas / MCL_MM_PER_IN;
          double dir_x = 0.0, dir_y = 0.0;
          heading_unit(ray_heading, dir_x, dir_y);
          double end_x = ox + dir_x * meas_in;
          double end_y = oy + dir_y * meas_in;
          double dist_mm = distance_to_map_mode(end_x, end_y, cfg.map_mode, s) * MCL_MM_PER_IN;
          double p_hit = gaussian(dist_mm, 0.0, sigma_eff);
          double p_rand = (meas >= 0.0 && meas <= s_max) ? (1.0 / s_max) : 0.0;
          double p_max = (MCL_DIST_W_MAX > 0.0 && meas >= s_max) ? 1.0 : 0.0;
          w_ray = MCL_DIST_W_HIT * p_hit + MCL_DIST_W_RAND * p_rand + MCL_DIST_W_MAX * p_max;
          if (MCL_DIST_GATE_MM > 0.0 && dist_mm > MCL_DIST_GATE_MM) {
            if (MCL_DIST_GATE_MODE == MCL_GATE_MODE_SOFT) {
              w_ray *= MCL_DIST_GATE_PENALTY;
            } else {
              w_ray = 0.0;
            }
          }
        } else {
          double expected_in = raycast_distance_mode(ox, oy, ray_heading, s_max / MCL_MM_PER_IN, cfg.map_mode, s);
          if (expected_in < 0.0) expected_in = s_max / MCL_MM_PER_IN;
          double expected_mm = expected_in * MCL_MM_PER_IN;
          bool gated = (MCL_DIST_GATE_MM > 0.0 && std::fabs(meas - expected_mm) > MCL_DIST_GATE_MM);
          double error = meas - expected_mm;
          double p_hit = gaussian(error, 0.0, sigma_eff);
          double p_rand = (meas >= 0.0 && meas <= s_max) ? (1.0 / s_max) : 0.0;
          double p_short = 0.0;
          if (MCL_DIST_W_SHORT > 0.0 && meas >= 0.0 && meas <= expected_mm) {
            p_short = MCL_DIST_LAMBDA_SHORT * std::exp(-MCL_DIST_LAMBDA_SHORT * meas);
          }
          double p_max = (MCL_DIST_W_MAX > 0.0 && meas >= s_max) ? 1.0 : 0.0;
          w_ray = MCL_DIST_W_HIT * p_hit + MCL_DIST_W_RAND * p_rand + MCL_DIST_W_SHORT * p_short + MCL_DIST_W_MAX * p_max;
          if (gated) {
            if (MCL_DIST_GATE_MODE == MCL_GATE_MODE_SOFT) {
              w_ray *= MCL_DIST_GATE_PENALTY;
            } else {
              w_ray = 0.0;
            }
          }
        }
        if (MCL_DIST_MIN_SENSOR_WEIGHT > 0.0 && w_ray < MCL_DIST_MIN_SENSOR_WEIGHT) w_ray = MCL_DIST_MIN_SENSOR_WEIGHT;
        if (w_ray < 1e-300) w_ray = 1e-300;
        if (ray_used < 9) logp_rays[ray_used++] = std::log(w_ray);
      }
      if (ray_used > 0) {
        log_w += logmeanexp(logp_rays, ray_used);
      }
    }
    tmp_weights_[i] = log_w;
    if (log_w > max_log_w) max_log_w = log_w;
  }
  if (!std::isfinite(max_log_w)) return;
  double total = 0.0;
  for (int i = 0; i < count_; ++i) {
    double w = std::exp(tmp_weights_[i] - max_log_w);
    tmp_weights_[i] = w;
    total += w;
  }
  if (total <= 1e-12) return;
  for (int i = 0; i < count_; ++i) {
    particles_[i].w = tmp_weights_[i];
  }
  // Debug expected distance is computed from the prior estimate (pre-normalize).
  MCLPose dbg_pose = est;
  uint32_t mask = 0;
  for (int s = 0; s < used; ++s) {
    const MCLDistanceSensorConfig& cfg = MCL_DISTANCE_SENSORS[s];
    double meas = last_dist_measured_mm_[s];
    if (meas < 0.0) continue;
    double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;
    if (s_max <= 0.0) s_max = MCL_DIST_MAX_RANGE_MM;
    double off_x = 0.0, off_y = 0.0;
    rotate_local_to_world(cfg.x_in, cfg.y_in, dbg_pose.theta, off_x, off_y);
    double ox = dbg_pose.x + off_x;
    double oy = dbg_pose.y + off_y;
    double heading = wrap_deg(dbg_pose.theta + cfg.angle_deg + cfg.angle_offset_deg);
    double expected_in = raycast_distance_mode(ox, oy, heading, s_max / MCL_MM_PER_IN, cfg.map_mode, s);
    if (expected_in < 0.0) expected_in = s_max / MCL_MM_PER_IN;
    last_dist_expected_mm_[s] = expected_in * MCL_MM_PER_IN;
    if (!skip_sensor_[s]) mask |= (1u << s);
  }
  last_dist_used_mask_ = mask;
}

void MCLLocalizer::updateIMU(double heading_deg) {
  if (!MCL_USE_IMU) return;
  estimate_valid_ = false;
  for (int i = 0; i < count_; ++i) {
    double diff = angle_diff_deg(particles_[i].theta, heading_deg);
    particles_[i].w *= gaussian(diff, 0.0, MCL_IMU_SIGMA_DEG);
  }
}

void MCLLocalizer::updateVision(double x_in, double y_in, double theta_deg, double confidence) {
  if (!MCL_USE_VISION) return;
  if (confidence < MCL_VISION_CONFIDENCE_MIN) return;
  estimate_valid_ = false;
  for (int i = 0; i < count_; ++i) {
    double dx = particles_[i].x - x_in;
    double dy = particles_[i].y - y_in;
    double dist = std::sqrt(dx * dx + dy * dy);
    particles_[i].w *= gaussian(dist, 0.0, MCL_VISION_SIGMA_XY_IN);
    double diff = angle_diff_deg(particles_[i].theta, theta_deg);
    particles_[i].w *= gaussian(diff, 0.0, MCL_VISION_SIGMA_THETA_DEG);
  }
}

void MCLLocalizer::normalize() {
  double total = 0.0;
  double total_sensor = 0.0;
  double sumsq_sensor = 0.0;
  for (int i = 0; i < count_; ++i) {
    double ws = particles_[i].w;
    total_sensor += ws;
    sumsq_sensor += ws * ws;
    double w = particles_[i].w * regionWeight(particles_[i].x, particles_[i].y, particles_[i].theta);
    particles_[i].w = w;
    total += w;
  }
  double w_avg = total / std::max(1, count_);
  if (MCL_AUGMENTED_ENABLED) {
    if (w_slow_ <= 0.0) w_slow_ = w_avg;
    else w_slow_ += MCL_ALPHA_SLOW * (w_avg - w_slow_);
    if (w_fast_ <= 0.0) w_fast_ = w_avg;
    else w_fast_ += MCL_ALPHA_FAST * (w_avg - w_fast_);
  }
  if (total <= 1e-12) {
    for (int i = 0; i < count_; ++i) particles_[i].w = 1.0 / std::max(1, count_);
    confidence_ = 0.0;
    ess_ratio_ = 1.0;
    estimate_valid_ = false;
    return;
  }
  for (int i = 0; i < count_; ++i) particles_[i].w /= total;
  double neff = 0.0;
  if (sumsq_sensor > 1e-18 && total_sensor > 1e-12) {
    neff = (total_sensor * total_sensor) / sumsq_sensor;
  } else {
    neff = effectiveN();
  }
  double frac = neff / std::max(1.0, static_cast<double>(count_));
  ess_ratio_ = std::max(0.0, std::min(1.0, frac));
  confidence_ = std::max(0.0, std::min(1.0, 1.0 - ess_ratio_));
  if (MCL_CONFIDENCE_AUTO_REINIT && MCL_CONFIDENCE_THRESHOLD > 0.0 &&
      confidence_ < MCL_CONFIDENCE_THRESHOLD) {
    if (MCL_REINIT_MODE == MCL_REINIT_MODE_ESTIMATE) {
      MCLPose est = estimate();
      init(est.x, est.y, est.theta);
    } else {
      initGlobal();
    }
    estimate();
    return;
  }
  double x = 0.0, y = 0.0;
  double sin_sum = 0.0, cos_sum = 0.0;
  for (int i = 0; i < count_; ++i) {
    x += particles_[i].x * particles_[i].w;
    y += particles_[i].y * particles_[i].w;
    double th = particles_[i].theta * DEG_TO_RAD;
    sin_sum += std::sin(th) * particles_[i].w;
    cos_sum += std::cos(th) * particles_[i].w;
  }
  estimate_.x = x;
  estimate_.y = y;
  estimate_.theta = wrap_deg(std::atan2(sin_sum, cos_sum) * RAD_TO_DEG);
  if (MCL_MODE_SPLIT_ENABLED && confidence_ <= MCL_MODE_SPLIT_CONF_MAX && count_ >= 4) {
    int seed_a = 0;
    int seed_b = 0;
    double best_w = particles_[0].w;
    for (int i = 1; i < count_; ++i) {
      if (particles_[i].w > best_w) {
        best_w = particles_[i].w;
        seed_a = i;
      }
    }
    double far_d2 = -1.0;
    for (int i = 0; i < count_; ++i) {
      double dx = particles_[i].x - particles_[seed_a].x;
      double dy = particles_[i].y - particles_[seed_a].y;
      double d2 = dx * dx + dy * dy;
      if (d2 > far_d2) {
        far_d2 = d2;
        seed_b = i;
      }
    }
    double cx[2] = {particles_[seed_a].x, particles_[seed_b].x};
    double cy[2] = {particles_[seed_a].y, particles_[seed_b].y};
    double mass[2] = {0.0, 0.0};
    double sx[2] = {0.0, 0.0};
    double sy[2] = {0.0, 0.0};
    double ss[2] = {0.0, 0.0};
    double cc[2] = {0.0, 0.0};
    for (int iter = 0; iter < 2; ++iter) {
      mass[0] = mass[1] = 0.0;
      sx[0] = sx[1] = 0.0;
      sy[0] = sy[1] = 0.0;
      ss[0] = ss[1] = 0.0;
      cc[0] = cc[1] = 0.0;
      for (int i = 0; i < count_; ++i) {
        double dx0 = particles_[i].x - cx[0];
        double dy0 = particles_[i].y - cy[0];
        double dx1 = particles_[i].x - cx[1];
        double dy1 = particles_[i].y - cy[1];
        int k = ((dx1 * dx1 + dy1 * dy1) < (dx0 * dx0 + dy0 * dy0)) ? 1 : 0;
        double w = particles_[i].w;
        mass[k] += w;
        sx[k] += particles_[i].x * w;
        sy[k] += particles_[i].y * w;
        double th = particles_[i].theta * DEG_TO_RAD;
        ss[k] += std::sin(th) * w;
        cc[k] += std::cos(th) * w;
      }
      for (int k = 0; k < 2; ++k) {
        if (mass[k] > 1e-12) {
          cx[k] = sx[k] / mass[k];
          cy[k] = sy[k] / mass[k];
        }
      }
    }
    double min_mass = MCL_MODE_SPLIT_MIN_MASS;
    if (min_mass < 0.0) min_mass = 0.0;
    if (min_mass > 0.49) min_mass = 0.49;
    double sep_min2 = MCL_MODE_SPLIT_MIN_SEPARATION_IN * MCL_MODE_SPLIT_MIN_SEPARATION_IN;
    double cdx = cx[0] - cx[1];
    double cdy = cy[0] - cy[1];
    double sep2 = cdx * cdx + cdy * cdy;
    if (mass[0] >= min_mass && mass[1] >= min_mass && sep2 >= sep_min2) {
      int keep = (mass[1] > mass[0]) ? 1 : 0;
      if (mass[keep] > 1e-12) {
        estimate_.x = sx[keep] / mass[keep];
        estimate_.y = sy[keep] / mass[keep];
        estimate_.theta = wrap_deg(std::atan2(ss[keep], cc[keep]) * RAD_TO_DEG);
      }
    }
  }
  if (MCL_CGR_LITE_ENABLED) cgrLiteRefineEstimate();
  estimate_valid_ = true;
  have_estimate_ever_ = true;
}

void MCLLocalizer::cgrLiteRefineEstimate() {
  if (!MCL_CGR_LITE_ENABLED || count_ <= 0) return;
  int top_k = std::max(1, std::min(MCL_CGR_LITE_TOP_K, count_));
  int max_iters = std::max(1, MCL_CGR_LITE_MAX_ITERS);
  double budget_ms = MCL_CGR_LITE_BUDGET_MS;
  if (!std::isfinite(budget_ms) || budget_ms < 0.0) budget_ms = 0.0;
  std::uint32_t t0 = pros::millis();
  double cx = estimate_.x;
  double cy = estimate_.y;
  double ct = estimate_.theta;
  const double sigma_xy = std::max(2.0, MCL_MODE_SPLIT_MIN_SEPARATION_IN * 0.5);
  const double sigma_th = 20.0;
  for (int iter = 0; iter < max_iters; ++iter) {
    if (budget_ms > 0.0) {
      std::uint32_t elapsed = pros::millis() - t0;
      if (static_cast<double>(elapsed) >= budget_ms) break;
    }
    for (int j = 0; j < top_k; ++j) {
      tmp_weights_[j] = -1.0;
      cdf_buf_[j] = 0.0;
    }
    for (int i = 0; i < count_; ++i) {
      double dx = particles_[i].x - cx;
      double dy = particles_[i].y - cy;
      double dth = angle_diff_deg(particles_[i].theta, ct);
      double score = particles_[i].w * std::exp(-0.5 * ((dx * dx + dy * dy) / (sigma_xy * sigma_xy) + (dth * dth) / (sigma_th * sigma_th)));
      int worst = 0;
      for (int j = 1; j < top_k; ++j) {
        if (tmp_weights_[j] < tmp_weights_[worst]) worst = j;
      }
      if (score > tmp_weights_[worst]) {
        tmp_weights_[worst] = score;
        cdf_buf_[worst] = static_cast<double>(i);
      }
    }
    double sw = 0.0, sx = 0.0, sy = 0.0, ss = 0.0, cc = 0.0;
    for (int j = 0; j < top_k; ++j) {
      double w = tmp_weights_[j];
      if (!(w > 0.0)) continue;
      int idx = static_cast<int>(cdf_buf_[j]);
      if (idx < 0 || idx >= count_) continue;
      sw += w;
      sx += particles_[idx].x * w;
      sy += particles_[idx].y * w;
      double th = particles_[idx].theta * DEG_TO_RAD;
      ss += std::sin(th) * w;
      cc += std::cos(th) * w;
    }
    if (sw <= 1e-12) break;
    cx = sx / sw;
    cy = sy / sw;
    ct = wrap_deg(std::atan2(ss, cc) * RAD_TO_DEG);
  }
  estimate_.x = cx;
  estimate_.y = cy;
  estimate_.theta = ct;
}

double MCLLocalizer::effectiveN() const {
  double denom = 0.0;
  for (int i = 0; i < count_; ++i) denom += particles_[i].w * particles_[i].w;
  return (denom <= 1e-12) ? 0.0 : (1.0 / denom);
}

void MCLLocalizer::getLastDistanceDebug(double* measured_mm, double* expected_mm, int* errno_codes, int capacity, uint32_t* used_mask) const {
  int n = std::min(capacity, MCL_DISTANCE_SENSOR_COUNT_SAFE);
  for (int i = 0; i < n; ++i) {
    if (measured_mm) measured_mm[i] = last_dist_measured_mm_[i];
    if (expected_mm) expected_mm[i] = last_dist_expected_mm_[i];
    if (errno_codes) errno_codes[i] = last_dist_errno_[i];
  }
  if (used_mask) *used_mask = last_dist_used_mask_;
}

void MCLLocalizer::copyWeights(double* out_weights, int capacity) const {
  if (!out_weights || capacity <= 0) return;
  int n = std::min(capacity, count_);
  for (int i = 0; i < n; ++i) out_weights[i] = particles_[i].w;
}

void MCLLocalizer::setWeights(const double* in_weights, int capacity) {
  if (!in_weights || capacity <= 0) return;
  int n = std::min(capacity, count_);
  double total = 0.0;
  for (int i = 0; i < n; ++i) {
    double w = in_weights[i];
    if (!std::isfinite(w) || w < 0.0) w = 0.0;
    particles_[i].w = w;
    total += w;
  }
  for (int i = n; i < count_; ++i) particles_[i].w = 0.0;
  if (total <= 1e-12) {
    for (int i = 0; i < count_; ++i) particles_[i].w = 1.0 / std::max(1, count_);
  } else {
    for (int i = 0; i < count_; ++i) particles_[i].w /= total;
  }
  estimate_valid_ = false;
}

void MCLLocalizer::setForcedInjectionFraction(double frac) {
  if (!std::isfinite(frac)) frac = 0.0;
  if (frac < 0.0) frac = 0.0;
  if (frac > 1.0) frac = 1.0;
  forced_injection_fraction_ = frac;
}

void MCLLocalizer::estimateCovariance(MCLPose* pose_out, double cov_out[3][3]) {
  MCLPose est = estimate();
  if (pose_out) *pose_out = est;
  if (!cov_out) return;
  for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) cov_out[r][c] = 0.0;
  for (int i = 0; i < count_; ++i) {
    double w = particles_[i].w;
    double dx = particles_[i].x - est.x;
    double dy = particles_[i].y - est.y;
    double dth = angle_diff_deg(particles_[i].theta, est.theta);
    cov_out[0][0] += w * dx * dx;
    cov_out[0][1] += w * dx * dy;
    cov_out[0][2] += w * dx * dth;
    cov_out[1][0] += w * dy * dx;
    cov_out[1][1] += w * dy * dy;
    cov_out[1][2] += w * dy * dth;
    cov_out[2][0] += w * dth * dx;
    cov_out[2][1] += w * dth * dy;
    cov_out[2][2] += w * dth * dth;
  }
}

void MCLLocalizer::resample() {
  double neff = effectiveN();
  if (!MCL_RESAMPLE_ALWAYS && neff >= MCL_RESAMPLE_THRESHOLD * std::max(1, count_)) return;
  int target = count_;
  if (MCL_KLD_ENABLED) {
    for (int i = 0; i < count_; ++i) {
      bin_buf_[i] = bin_index(particles_[i].x, particles_[i].y, particles_[i].theta);
    }
    std::sort(bin_buf_, bin_buf_ + count_);
    int k = 1;
    for (int i = 1; i < count_; ++i) {
      if (bin_buf_[i] != bin_buf_[i - 1]) k++;
    }
    int n_req = kld_required_particles(k, MCL_KLD_EPSILON, MCL_KLD_DELTA);
    target = std::max(MCL_N_MIN, std::min(MCL_N_MAX, n_req));
  }
  target = std::max(1, std::min(target, MCL_PARTICLE_CAPACITY));
  if (MCL_RESAMPLE_METHOD == MCL_RESAMPLE_STRATIFIED) {
    double c = particles_[0].w;
    int i = 0;
    for (int m = 0; m < target; ++m) {
      double u = (m + rand_uniform()) / target;
      while (u > c && i < count_ - 1) {
        i++;
        c += particles_[i].w;
      }
      Particle p = particles_[i];
      p.w = 1.0 / std::max(1, target);
      resample_buf_[m] = p;
    }
  } else if (MCL_RESAMPLE_METHOD == MCL_RESAMPLE_MULTINOMIAL) {
    double total = 0.0;
    for (int i = 0; i < count_; ++i) {
      total += particles_[i].w;
      cdf_buf_[i] = total;
    }
    for (int m = 0; m < target; ++m) {
      double r = rand_uniform() * total;
      int idx = 0;
      while (idx < count_ - 1 && r > cdf_buf_[idx]) idx++;
      Particle p = particles_[idx];
      p.w = 1.0 / std::max(1, target);
      resample_buf_[m] = p;
    }
  } else {
    double step = 1.0 / target;
    double r = rand_uniform() * step;
    double c = particles_[0].w;
    int i = 0;
    for (int m = 0; m < target; ++m) {
      double u = r + m * step;
      while (u > c && i < count_ - 1) {
        i++;
        c += particles_[i].w;
      }
      Particle p = particles_[i];
      p.w = 1.0 / std::max(1, target);
      resample_buf_[m] = p;
    }
  }
  double inj = 0.0;
  if (MCL_AUGMENTED_ENABLED) {
    if (w_slow_ > 1e-9 && w_fast_ > 1e-9) {
      inj = std::max(0.0, 1.0 - w_fast_ / w_slow_);
    }
  } else {
    inj = MCL_RANDOM_INJECTION;
  }
  if (forced_injection_fraction_ > inj) inj = forced_injection_fraction_;
  if (inj > 1.0) inj = 1.0;
  for (int i = 0; i < target; ++i) {
    if (inj > 0.0 && rand_uniform() < inj) {
      PoseSample p = sample_random_pose();
      resample_buf_[i].x = p.x;
      resample_buf_[i].y = p.y;
      resample_buf_[i].theta = p.theta;
      resample_buf_[i].w = 1.0 / std::max(1, target);
    }
  }
  if (MCL_RESAMPLE_ROUGHEN_XY_IN > 0.0 || MCL_RESAMPLE_ROUGHEN_THETA_DEG > 0.0) {
    for (int i = 0; i < target; ++i) {
      if (MCL_RESAMPLE_ROUGHEN_XY_IN > 0.0) {
        resample_buf_[i].x += rand_gaussian(0.0, MCL_RESAMPLE_ROUGHEN_XY_IN);
        resample_buf_[i].y += rand_gaussian(0.0, MCL_RESAMPLE_ROUGHEN_XY_IN);
      }
      if (MCL_RESAMPLE_ROUGHEN_THETA_DEG > 0.0) {
        resample_buf_[i].theta = wrap_deg(resample_buf_[i].theta + rand_gaussian(0.0, MCL_RESAMPLE_ROUGHEN_THETA_DEG));
      }
    }
  }
  count_ = target;
  for (int i = 0; i < count_; ++i) particles_[i] = resample_buf_[i];
}

MCLPose MCLLocalizer::estimate() {
  if (!estimate_valid_) {
    double x = 0.0, y = 0.0;
    double sin_sum = 0.0, cos_sum = 0.0;
    for (int i = 0; i < count_; ++i) {
      x += particles_[i].x * particles_[i].w;
      y += particles_[i].y * particles_[i].w;
      double th = particles_[i].theta * DEG_TO_RAD;
      sin_sum += std::sin(th) * particles_[i].w;
      cos_sum += std::cos(th) * particles_[i].w;
    }
    estimate_.x = x;
    estimate_.y = y;
    estimate_.theta = wrap_deg(std::atan2(sin_sum, cos_sum) * RAD_TO_DEG);
    estimate_valid_ = true;
    have_estimate_ever_ = true;
  }
  return estimate_;
}
