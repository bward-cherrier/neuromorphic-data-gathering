//
// Created by giovanni on 26.04.22.
//

#pragma once

#include <Eigen/Dense>

namespace dv::optimization {

/**
 * Basic functor class inherited by all contrastMaximization functor. This functor is used by Eigen/NumericalDiff class,
 * which handles the non linear optimization underlying contrast maximization algorithm.
 * For more information about contrast maximization please check "contrast_maximization_rotation.hpp" or
 * "contrast_maximization_translation_and_depth.hpp".
 * @tparam _Scalar type of variable to optimize (e.g. int, float..).
 * @tparam NX Number of input variables (note: all variables are stored as Nx1 vector of values)
 * @tparam NY Number of output measurements (note: number of measurements needs to be at least as big as number of
 * 			  input variables - NX - otherwise the optimization problem cannot be solved.)
 */

template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
class OptimizationFunctor {
private:
	int mInputs, mValues;

public:
	typedef _Scalar Scalar;

	enum { // Required by numerical differentiation module
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};

	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;
	/**
	 * Base method for cost function implementation.
	 * @param input parameters to be optimized
	 * @param cost cost value updated at each iteration of the optimization.
	 * @return optimization result (positive if successful)
	 */
	virtual int operator()(const Eigen::VectorXf &input, Eigen::VectorXf &cost) const = 0;

	/**
	 * Constructor for cost optimization parameters
	 * @param inputs number of inputs to be optimized
	 * @param values number of functions evaluation for gradient computation
	 */
	OptimizationFunctor(int inputs, int values) : mInputs(inputs), mValues(values) {
	}

	/**
	 * getter for size of input parameters to be optimized.
	 * @return number of input parameters optimized.
	 */
	int inputs() const {
		return mInputs;
	}

	/**
	 * getter for size of function evaluations performed at each optimization iteration.
	 * @return number of function evaluations at each optimization iteration.
	 */
	int values() const {
		return mValues;
	}
};

} // namespace dv::optimization
