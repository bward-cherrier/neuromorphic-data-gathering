#pragma once

#include <cmath>
#include <concepts>

namespace dv::cluster::mean_shift::kernel {

template<class T>
concept MeanShiftKernel = requires(T t, float sd, float bw) {
							  { T::getSearchRadius(bw) } -> std::same_as<float>;
							  { T::apply(sd, bw) } -> std::same_as<float>;
						  };

struct Epanechnikov {
	static float getSearchRadius(const float bandwidth) {
		return bandwidth;
	}

	static float apply(const float squaredDistance, const float bandwidth) {
		return squaredDistance / (bandwidth * bandwidth) <= 1.0f ? 1.0f : 0.0f;
	}
};

static_assert(MeanShiftKernel<Epanechnikov>);

struct Gaussian {
	static float getSearchRadius(const float bandwidth) {
		return 3.0f * bandwidth;
	}

	static float apply(const float squaredDistance, const float bandwidth) {
		return std::exp(-0.5f * squaredDistance / (bandwidth * bandwidth));
	}
};

static_assert(MeanShiftKernel<Gaussian>);

} // namespace dv::cluster::mean_shift::kernel
