// mcl_map_data.h (generated)
#pragma once
#include <cstdint>

// Likelihood-field distance grids used by distance sensors.
extern const uint16_t MAP_DIST_FIELD[];
extern const uint16_t MAP_DIST_FIELD_PERIM[];
extern const uint16_t MAP_DIST_FIELD_OBJ[];

constexpr uint32_t MCL_MAP_DATA_HASH32 = 0x8d1797d2u;
constexpr uint32_t MCL_MAP_PROJECT_HASH32 = 0x2ecb12bau;
constexpr int MCL_MAP_GRID_W = 145;
constexpr int MCL_MAP_GRID_H = 145;
constexpr int MCL_MAP_DIST_FIELD_LEN = 21025;
constexpr int MCL_MAP_DIST_FIELD_PERIM_LEN = 21025;
constexpr int MCL_MAP_DIST_FIELD_OBJ_LEN = 21025;

