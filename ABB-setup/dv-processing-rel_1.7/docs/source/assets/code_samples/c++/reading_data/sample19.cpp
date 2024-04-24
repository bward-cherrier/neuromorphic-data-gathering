#include <dv-processing/io/camera_capture.hpp>

int main() {
	// Open a Davis camera
	dv::io::CameraCapture capture("", dv::io::CameraCapture::CameraType::DAVIS);

	/// Access biases raw value
	// Photoreceptor bias
	uint16_t defaultPrBpInt = capture.deviceConfigGet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PRBP);
	// Source follower bias
	uint16_t defaultPrSfBpInt = capture.deviceConfigGet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PRSFBP);
	// Differential bias
	uint16_t defaultDiffBnInt = capture.deviceConfigGet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_DIFFBN);
	// On threshold bias
	uint16_t defaultOnBnInt = capture.deviceConfigGet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ONBN);
	// Off threshold bias
	uint16_t defaultOffBnInt = capture.deviceConfigGet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_OFFBN);
	// Refractory period bias
	uint16_t defaultRefrBpInt = capture.deviceConfigGet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_REFRBP);

	/// Change biases values
	// Convert bias integer to values
	caer_bias_coarsefine coarseFinePrBp   = caerBiasCoarseFineParse(defaultPrBpInt);
	caer_bias_coarsefine coarseFinePrSfBp = caerBiasCoarseFineParse(defaultPrSfBpInt);
	caer_bias_coarsefine coarseFineDiffBn = caerBiasCoarseFineParse(defaultDiffBnInt);
	caer_bias_coarsefine coarseFineOnBn   = caerBiasCoarseFineParse(defaultOnBnInt);
	caer_bias_coarsefine coarseFineOffBn  = caerBiasCoarseFineParse(defaultOffBnInt);
	caer_bias_coarsefine coarseFineRefrBp = caerBiasCoarseFineParse(defaultRefrBpInt);
	// For example here, add 1 on the log-scale coarse value, i.e. multiply bias value by 10 (approximately)
	coarseFinePrBp.coarseValue   += 1;
	coarseFinePrSfBp.coarseValue += 1;
	coarseFineDiffBn.coarseValue += 1;
	coarseFineOnBn.coarseValue   += 1;
	coarseFineOffBn.coarseValue  += 1;
	coarseFineRefrBp.coarseValue += 1;
	// Convert back
	const uint16_t newPrBp   = caerBiasCoarseFineGenerate(coarseFinePrBp);
	const uint16_t newPrSfBp = caerBiasCoarseFineGenerate(coarseFinePrSfBp);
	const uint16_t newDiffBn = caerBiasCoarseFineGenerate(coarseFineDiffBn);
	const uint16_t newOnBn   = caerBiasCoarseFineGenerate(coarseFineOnBn);
	const uint16_t newOffBn  = caerBiasCoarseFineGenerate(coarseFineOffBn);
	const uint16_t newRefrBp = caerBiasCoarseFineGenerate(coarseFineRefrBp);

	/// Set biases raw value
	// Setting photoreceptor bias
	capture.deviceConfigSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PRBP, newPrBp);
	// Setting source follower bias
	capture.deviceConfigSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PRSFBP, newPrSfBp);
	// Setting differential bias
	capture.deviceConfigSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_DIFFBN, newDiffBn);
	// Setting on threshold bias
	capture.deviceConfigSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ONBN, newOnBn);
	// Setting off threshold bias
	capture.deviceConfigSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_OFFBN, newOffBn);
	// Setting refractory period bias
	capture.deviceConfigSet(DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_REFRBP, newRefrBp);

	return 0;
}
