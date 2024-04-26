#include "../../include/dv-processing/containers/kd_tree.hpp"

#include "boost/ut.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <random>
#include <unordered_set>

using namespace boost::ut;
using namespace dv::containers::kd_tree;

std::unordered_set<uint64_t> generateRandomButUniqueIndices(const uint64_t numValues, const uint64_t maxIndex) {
	std::unordered_set<uint64_t> expectedIndices;

	static constexpr uint32_t SEED = 0;
	std::default_random_engine generator(SEED);
	std::uniform_int_distribution<uint64_t> distribution(0, maxIndex);

	for (uint8_t i = 0; i < numValues; i++) {
		auto randomIndex = 0;
		do {
			randomIndex = distribution(generator);
		}
		while (expectedIndices.find(randomIndex) != expectedIndices.end());
		expectedIndices.insert(randomIndex);
	}

	return expectedIndices;
}

void testMatrix() {
	"1d_dynamic_size_row_vector_knnSearch"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeColMajorXX<int32_t>;

		Tree::Matrix data(1, NUM_SAMPLES);

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(i) = NEIGHBOURS_VALUE;
			}
			else {
				data(i) = i;
			}
		}

		const auto tree = Tree(data);

		const auto centre = Tree::Vector::Constant(1, 1, NEIGHBOURS_VALUE);

		const auto indicesAndDistances = tree.knnSearch(centre, EXPECTED_NUM_NEIGHBOURS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(indexAndDistance.first) == centre(0);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"1d_dynamic_size_column_vector_knnSearch"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeRowMajorXX<int32_t>;

		Tree::Matrix data(NUM_SAMPLES, 1);

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(i) = NEIGHBOURS_VALUE;
			}
			else {
				data(i) = i;
			}
		}

		const auto tree = Tree(data);

		const auto centre = Tree::Vector::Constant(1, 1, NEIGHBOURS_VALUE);

		const auto indicesAndDistances = tree.knnSearch(centre, EXPECTED_NUM_NEIGHBOURS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(indexAndDistance.first) == centre(0);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"1d_static_size_row_vector_knnSearch"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeRowMajorX1<int32_t, NUM_SAMPLES>;

		Tree::Matrix data;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(i) = NEIGHBOURS_VALUE;
			}
			else {
				data(i) = i;
			}
		}

		const auto tree = Tree(data);

		const auto centre = Tree::Vector::Constant(1, 1, NEIGHBOURS_VALUE);

		const auto indicesAndDistances = tree.knnSearch(centre, EXPECTED_NUM_NEIGHBOURS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(indexAndDistance.first) == centre(0);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"1d_static_size_column_vector_knnSearch"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeColMajor1X<int32_t, NUM_SAMPLES>;

		Tree::Matrix data;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(i) = NEIGHBOURS_VALUE;
			}
			else {
				data(i) = i;
			}
		}

		const auto tree = Tree(data);

		const auto centre = Tree::Vector::Constant(1, 1, NEIGHBOURS_VALUE);

		const auto indicesAndDistances = tree.knnSearch(centre, EXPECTED_NUM_NEIGHBOURS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(indexAndDistance.first) == centre(0);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"1d_dynamic_size_row_vector_radiusSearch"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeColMajorXX<int32_t>;

		Tree::Matrix data(1, NUM_SAMPLES);

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(i) = NEIGHBOURS_VALUE;
			}
			else {
				data(i) = i;
			}
		}

		const auto tree = Tree(data);

		static constexpr auto RADIUS = 1;
		const auto centre            = Tree::Vector::Constant(1, 1, NEIGHBOURS_VALUE);

		const auto indicesAndDistances = tree.radiusSearch(centre, RADIUS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(indexAndDistance.first) == centre(0);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"1d_dynamic_size_column_vector_radiusSearch"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeRowMajorXX<int32_t>;

		Tree::Matrix data(NUM_SAMPLES, 1);

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(i) = NEIGHBOURS_VALUE;
			}
			else {
				data(i) = i;
			}
		}

		const auto tree = Tree(data);

		static constexpr auto RADIUS = 1;
		const auto centre            = Tree::Vector::Constant(1, 1, NEIGHBOURS_VALUE);

		const auto indicesAndDistances = tree.radiusSearch(centre, RADIUS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(indexAndDistance.first) == centre(0);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"1d_static_size_row_vector_radiusSearch"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeRowMajorX1<int32_t, NUM_SAMPLES>;

		Tree::Matrix data;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(i) = NEIGHBOURS_VALUE;
			}
			else {
				data(i) = i;
			}
		}

		const auto tree = Tree(data);

		static constexpr auto RADIUS = 1;
		const auto centre            = Tree::Vector::Constant(1, 1, NEIGHBOURS_VALUE);

		const auto indicesAndDistances = tree.radiusSearch(centre, RADIUS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(indexAndDistance.first) == centre(0);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"1d_static_size_column_vector_radiusSearch"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeColMajor1X<int32_t, NUM_SAMPLES>;

		Tree::Matrix data;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(i) = NEIGHBOURS_VALUE;
			}
			else {
				data(i) = i;
			}
		}

		const auto tree = Tree(data);

		static constexpr auto RADIUS = 1;
		const auto centre            = Tree::Vector::Constant(1, 1, NEIGHBOURS_VALUE);

		const auto indicesAndDistances = tree.radiusSearch(centre, RADIUS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(indexAndDistance.first) == centre(0);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"2d_dynamic_size_row_major_knnSearch"_test = [&] {
		static constexpr auto DIMS            = 2;
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeRowMajorXX<int32_t>;

		Tree::Matrix data(NUM_SAMPLES, DIMS);

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(i, 0) = NEIGHBOURS_VALUE;
				data(i, 1) = NEIGHBOURS_VALUE;
			}
			else {
				data(i, 0) = i;
				data(i, 1) = i;
			}
		}

		const auto tree = Tree(data);

		Eigen::Matrix<int32_t, 2, 1> centre;
		centre << NEIGHBOURS_VALUE, NEIGHBOURS_VALUE;

		const auto indicesAndDistances = tree.knnSearch(centre, EXPECTED_NUM_NEIGHBOURS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(indexAndDistance.first, 0) == centre(0) && data(indexAndDistance.first, 1) == centre(1);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"2d_dynamic_size_column_major_knnSearch"_test = [&] {
		static constexpr auto DIMS            = 2;
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeColMajorXX<int32_t>;

		Tree::Matrix data(DIMS, NUM_SAMPLES);

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(0, i) = NEIGHBOURS_VALUE;
				data(1, i) = NEIGHBOURS_VALUE;
			}
			else {
				data(0, i) = i;
				data(1, i) = i;
			}
		}

		const auto tree = Tree(data);

		Eigen::Matrix<int32_t, 2, 1> centre;
		centre << NEIGHBOURS_VALUE, NEIGHBOURS_VALUE;

		const auto indicesAndDistances = tree.knnSearch(centre, EXPECTED_NUM_NEIGHBOURS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(0, indexAndDistance.first) == centre(0) && data(1, indexAndDistance.first) == centre(1);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"2d_static_size_row_major_knnSearch"_test = [&] {
		static constexpr auto DIMS            = 2;
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeRowMajorX2<int32_t, NUM_SAMPLES>;

		Tree::Matrix data;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(i, 0) = NEIGHBOURS_VALUE;
				data(i, 1) = NEIGHBOURS_VALUE;
			}
			else {
				data(i, 0) = i;
				data(i, 1) = i;
			}
		}

		const auto tree = Tree(data);

		Eigen::Matrix<int32_t, 2, 1> centre;
		centre << NEIGHBOURS_VALUE, NEIGHBOURS_VALUE;

		const auto indicesAndDistances = tree.knnSearch(centre, EXPECTED_NUM_NEIGHBOURS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(indexAndDistance.first, 0) == centre(0) && data(indexAndDistance.first, 1) == centre(1);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"2d_static_size_column_major_knnSearch"_test = [&] {
		static constexpr auto DIMS            = 2;
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeColMajor2X<int32_t, NUM_SAMPLES>;

		Tree::Matrix data;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(0, i) = NEIGHBOURS_VALUE;
				data(1, i) = NEIGHBOURS_VALUE;
			}
			else {
				data(0, i) = i;
				data(1, i) = i;
			}
		}

		const auto tree = Tree(data);

		Eigen::Matrix<int32_t, 2, 1> centre;
		centre << NEIGHBOURS_VALUE, NEIGHBOURS_VALUE;

		const auto indicesAndDistances = tree.knnSearch(centre, EXPECTED_NUM_NEIGHBOURS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(0, indexAndDistance.first) == centre(0) && data(1, indexAndDistance.first) == centre(1);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"2d_dynamic_size_row_major_radiusSearch"_test = [&] {
		static constexpr auto DIMS            = 2;
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeRowMajorXX<int32_t>;

		Tree::Matrix data(NUM_SAMPLES, DIMS);

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(i, 0) = NEIGHBOURS_VALUE;
				data(i, 1) = NEIGHBOURS_VALUE;
			}
			else {
				data(i, 0) = i;
				data(i, 1) = i;
			}
		}

		const auto tree = Tree(data);

		Eigen::Matrix<int32_t, 2, 1> centre;
		static constexpr auto RADIUS = 1;
		centre << NEIGHBOURS_VALUE, NEIGHBOURS_VALUE;

		const auto indicesAndDistances = tree.radiusSearch(centre, RADIUS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(indexAndDistance.first, 0) == centre(0) && data(indexAndDistance.first, 1) == centre(1);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"2d_dynamic_size_column_major_radiusSearch"_test = [&] {
		static constexpr auto DIMS = 2;
		const uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeColMajorXX<int32_t>;

		Tree::Matrix data(DIMS, NUM_SAMPLES);

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(0, i) = NEIGHBOURS_VALUE;
				data(1, i) = NEIGHBOURS_VALUE;
			}
			else {
				data(0, i) = i;
				data(1, i) = i;
			}
		}

		const auto tree = Tree(data);

		Eigen::Matrix<int32_t, 2, 1> centre;
		static constexpr auto RADIUS = 1;
		centre << NEIGHBOURS_VALUE, NEIGHBOURS_VALUE;

		const auto indicesAndDistances = tree.radiusSearch(centre, RADIUS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(0, indexAndDistance.first) == centre(0) && data(1, indexAndDistance.first) == centre(1);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"2d_static_size_row_major_radiusSearch"_test = [&] {
		static constexpr auto DIMS            = 2;
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeRowMajorX2<int32_t, NUM_SAMPLES>;

		Tree::Matrix data;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(i, 0) = NEIGHBOURS_VALUE;
				data(i, 1) = NEIGHBOURS_VALUE;
			}
			else {
				data(i, 0) = i;
				data(i, 1) = i;
			}
		}

		const auto tree = Tree(data);

		Eigen::Matrix<int32_t, 2, 1> centre;
		static constexpr auto RADIUS = 1;
		centre << NEIGHBOURS_VALUE, NEIGHBOURS_VALUE;

		const auto indicesAndDistances = tree.radiusSearch(centre, RADIUS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(indexAndDistance.first, 0) == centre(0) && data(indexAndDistance.first, 1) == centre(1);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"2d_static_size_column_major_radiusSearch"_test = [&] {
		static constexpr auto DIMS            = 2;
		static constexpr uint32_t NUM_SAMPLES = 10000;

		using Tree = KDTreeColMajor2X<int32_t, NUM_SAMPLES>;

		Tree::Matrix data;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				data(0, i) = NEIGHBOURS_VALUE;
				data(1, i) = NEIGHBOURS_VALUE;
			}
			else {
				data(0, i) = i;
				data(1, i) = i;
			}
		}

		const auto tree = Tree(data);

		Eigen::Matrix<int32_t, 2, 1> centre;
		static constexpr auto RADIUS = 1;
		centre << NEIGHBOURS_VALUE, NEIGHBOURS_VALUE;

		const auto indicesAndDistances = tree.radiusSearch(centre, RADIUS);

		expect(indicesAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return data(0, indexAndDistance.first) == centre(0) && data(1, indexAndDistance.first) == centre(1);
		}));

		expect(std::all_of(indicesAndDistances.begin(), indicesAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};
}

void testEventStore() {
	"EventStore_radiusSearch_Event_centre"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		dv::EventStore events;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				events << dv::Event(0, NEIGHBOURS_VALUE, NEIGHBOURS_VALUE, 0);
			}
			else {
				events << dv::Event(0, 0, 0, 0);
			}
		}

		const auto tree = KDTreeEventStoreAdaptor(events);

		dv::Event centre(0, NEIGHBOURS_VALUE, NEIGHBOURS_VALUE, 0);
		static constexpr auto RADIUS = 1;

		const auto eventsAndDistances = tree.radiusSearch(centre, RADIUS);

		expect(eventsAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(eventsAndDistances.begin(), eventsAndDistances.end(), [&](const auto &indexAndDistance) {
			return *indexAndDistance.first == centre;
		}));

		expect(std::all_of(eventsAndDistances.begin(), eventsAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"EventStore_radiusSearch_TimedKeyPoint_centre"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		dv::EventStore events;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				events << dv::Event(0, NEIGHBOURS_VALUE, NEIGHBOURS_VALUE, 0);
			}
			else {
				events << dv::Event(0, 0, 0, 0);
			}
		}

		const auto tree = KDTreeEventStoreAdaptor(events);

		dv::TimedKeyPoint centre(dv::Point2f(NEIGHBOURS_VALUE, NEIGHBOURS_VALUE), 0.0, 0.0, 0.0, 0, 0, 0);
		static constexpr auto RADIUS = 1;

		const auto eventsAndDistances = tree.radiusSearch(centre, RADIUS);

		expect(eventsAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(eventsAndDistances.begin(), eventsAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.first->x() == NEIGHBOURS_VALUE && indexAndDistance.first->y() == NEIGHBOURS_VALUE;
		}));

		expect(std::all_of(eventsAndDistances.begin(), eventsAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"EventStore_radiusSearch_cvPoint_centre"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		dv::EventStore events;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				events << dv::Event(0, NEIGHBOURS_VALUE, NEIGHBOURS_VALUE, 0);
			}
			else {
				events << dv::Event(0, 0, 0, 0);
			}
		}

		const auto tree = KDTreeEventStoreAdaptor(events);

		cv::Point centre(NEIGHBOURS_VALUE, NEIGHBOURS_VALUE);
		static constexpr auto RADIUS = 1;

		const auto eventsAndDistances = tree.radiusSearch(centre, RADIUS);

		expect(eventsAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(eventsAndDistances.begin(), eventsAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.first->x() == NEIGHBOURS_VALUE && indexAndDistance.first->y() == NEIGHBOURS_VALUE;
		}));

		expect(std::all_of(eventsAndDistances.begin(), eventsAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"EventStore_knnSearch_Event_centre"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		dv::EventStore events;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				events << dv::Event(0, NEIGHBOURS_VALUE, NEIGHBOURS_VALUE, 0);
			}
			else {
				events << dv::Event(0, 0, 0, 0);
			}
		}

		const auto tree = KDTreeEventStoreAdaptor(events);

		dv::Event centre(0, NEIGHBOURS_VALUE, NEIGHBOURS_VALUE, 0);
		static constexpr auto RADIUS = 1;

		const auto eventsAndDistances = tree.knnSearch(centre, EXPECTED_NUM_NEIGHBOURS);

		expect(eventsAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(eventsAndDistances.begin(), eventsAndDistances.end(), [&](const auto &indexAndDistance) {
			return *indexAndDistance.first == centre;
		}));

		expect(std::all_of(eventsAndDistances.begin(), eventsAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"EventStore_knnSearch_TimedKeyPoint_centre"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		dv::EventStore events;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				events << dv::Event(0, NEIGHBOURS_VALUE, NEIGHBOURS_VALUE, 0);
			}
			else {
				events << dv::Event(0, 0, 0, 0);
			}
		}

		const auto tree = KDTreeEventStoreAdaptor(events);

		dv::TimedKeyPoint centre(dv::Point2f(NEIGHBOURS_VALUE, NEIGHBOURS_VALUE), 0.0, 0.0, 0.0, 0, 0, 0);
		static constexpr auto RADIUS = 1;

		const auto eventsAndDistances = tree.knnSearch(centre, EXPECTED_NUM_NEIGHBOURS);

		expect(eventsAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(eventsAndDistances.begin(), eventsAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.first->x() == NEIGHBOURS_VALUE && indexAndDistance.first->y() == NEIGHBOURS_VALUE;
		}));

		expect(std::all_of(eventsAndDistances.begin(), eventsAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};

	"EventStore_knnSearch_cvPoint_centre"_test = [&] {
		static constexpr uint32_t NUM_SAMPLES = 10000;

		dv::EventStore events;

		static constexpr uint8_t EXPECTED_NUM_NEIGHBOURS = 10;
		static constexpr int32_t NEIGHBOURS_VALUE        = -1000;

		const auto expectedIndices = generateRandomButUniqueIndices(EXPECTED_NUM_NEIGHBOURS, NUM_SAMPLES - 1);

		for (uint64_t i = 0; i < NUM_SAMPLES; i++) {
			if (expectedIndices.find(i) != expectedIndices.end()) {
				events << dv::Event(0, NEIGHBOURS_VALUE, NEIGHBOURS_VALUE, 0);
			}
			else {
				events << dv::Event(0, 0, 0, 0);
			}
		}

		const auto tree = KDTreeEventStoreAdaptor(events);

		cv::Point centre(NEIGHBOURS_VALUE, NEIGHBOURS_VALUE);
		static constexpr auto RADIUS = 1;

		const auto eventsAndDistances = tree.knnSearch(centre, EXPECTED_NUM_NEIGHBOURS);

		expect(eventsAndDistances.size() == EXPECTED_NUM_NEIGHBOURS);

		expect(std::all_of(eventsAndDistances.begin(), eventsAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.first->x() == NEIGHBOURS_VALUE && indexAndDistance.first->y() == NEIGHBOURS_VALUE;
		}));

		expect(std::all_of(eventsAndDistances.begin(), eventsAndDistances.end(), [&](const auto &indexAndDistance) {
			return indexAndDistance.second == 0;
		}));
	};
}

int main() {
	testMatrix();
	testEventStore();

	return EXIT_SUCCESS;
}
