//
// Created by giovanni on 20.04.22.
//
#pragma once
#include "../core/concepts.hpp"
#include "../optimization/optimization_functor.hpp"

#include <memory>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

namespace dv::optimization {

/**
 * Wrapper for all contrast maximization algorithms. For more information about contrast maximization please check
 * "contrast_maximization_rotation.hpp" or "contrast_maximization_translation_and_depth.hpp".
 * This wrapper is mainly meant to set the non linear differentiation parameters (see contructor for more information).
 * In addition, the class expose to user only "optimize" function which returns a struct containing the result of the
 * non-linear optimization (successful or not), number of iteration of the optimization and optimized parameters.
 * @tparam Functor Functor that handles optimization. Cost is computed by overriding operator() method. For an example
 * of a functor please check "contrast_maximization_rotation.hpp" or "contrast_maximization_translation_and_depth.hpp".
 */
template<class Functor>
class ContrastMaximizationWrapper {
	struct optimizationParameters {
		float learningRate = float(1e-1);
		float epsfcn       = 0;
		float ftol         = 0.000345267;
		float gtol         = 0;
		float xtol         = 0.000345267;
		int maxfev         = 400;
	};

	struct optimizationOutput {
		[[maybe_unused]] int optimizationSuccessful;
		int iter;
		Eigen::VectorXf optimizedVariable;
	};

private:
	std::unique_ptr<Functor> mFunctor = nullptr;
	optimizationParameters mParams;

public:
	/**
	 *
	 * @param functor_ functor handling contrast maximization optimization.
	 * the functor should inherit "OptimizationFunctor" and overload the "int operator()" method to compute cost for
	 * contrast maximization and optimize pre-defined parameters.
	 * @param learningRate constant multiplying input value to find new value at which function will be evaluated.
	 * E.g. assuming function is evaluated at x --> f(x), next input sample x' is computed as x' = abs(x) *
	 * learningRate.
	 * @param epsfcn error precision
	 * @param ftol tolerance for the norm of the vector function
	 * @param gtol tolerance for the norm of the gradient of the error vector
	 * @param xtol tolerance for the norm of the solution vector
	 * @param maxfev max number of function evaluations
	 * Note that default parameters are taken from default parameters of LevenbergMarquardt optimizer.
	 */
	ContrastMaximizationWrapper(std::unique_ptr<Functor> functor_, float learningRate, float epsfcn = 0,
		float ftol = 0.000345267, float gtol = 0, float xtol = 0.000345267, int maxfev = 400) :
		mFunctor(std::move(functor_)) {
		mParams.learningRate = learningRate;
		mParams.epsfcn       = epsfcn;
		mParams.ftol         = ftol;
		mParams.gtol         = gtol;
		mParams.xtol         = xtol;
		mParams.maxfev       = maxfev;
	}

	/**
	 * Function optimizing cost defined in mFunctor (inside operator() method).
	 * @param initialValues Initial values of variables to be optimized.
	 * @return optimized variable that minimize cost.
	 */
	[[nodiscard]] optimizationOutput optimize(const Eigen::VectorXf &initialValues) {
		Eigen::VectorXf optimizedVariable = initialValues;
		Eigen::NumericalDiff<Functor> numDiff(*mFunctor, mParams.learningRate);
		Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Functor>, float> lm(numDiff);
		lm.parameters.epsfcn = mParams.epsfcn;
		lm.parameters.ftol   = mParams.ftol;
		lm.parameters.gtol   = mParams.gtol;
		lm.parameters.xtol   = mParams.xtol;
		lm.parameters.maxfev = mParams.maxfev;

		int ret = lm.minimize(optimizedVariable);
		optimizationOutput output;
		output.iter                   = lm.iter;
		output.optimizationSuccessful = ret;
		output.optimizedVariable      = optimizedVariable;

		return output;
	}
};

} // namespace dv::optimization
