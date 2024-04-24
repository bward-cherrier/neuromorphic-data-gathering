#pragma once

#include <string_view>

/**
 * dv-processing version (MAJOR * 10000 + MINOR * 100 + PATCH).
 */
#define DV_PROCESSING_VERSION_MAJOR 1
#define DV_PROCESSING_VERSION_MINOR 7
#define DV_PROCESSING_VERSION_PATCH 9
#define DV_PROCESSING_VERSION \
	((DV_PROCESSING_VERSION_MAJOR * 10000) + (DV_PROCESSING_VERSION_MINOR * 100) + DV_PROCESSING_VERSION_PATCH)

/**
 * dv-processing name string.
 */
#define DV_PROCESSING_NAME_STRING "dv-processing"

/**
 * dv-processing version string.
 */
#define DV_PROCESSING_VERSION_STRING "1.7.9"

namespace dv {

static constexpr int VERSION_MAJOR{DV_PROCESSING_VERSION_MAJOR};
static constexpr int VERSION_MINOR{DV_PROCESSING_VERSION_MINOR};
static constexpr int VERSION_PATCH{DV_PROCESSING_VERSION_PATCH};
static constexpr int VERSION{DV_PROCESSING_VERSION};

static constexpr std::string_view NAME_STRING{DV_PROCESSING_NAME_STRING};
static constexpr std::string_view VERSION_STRING{DV_PROCESSING_VERSION_STRING};

} // namespace dv
