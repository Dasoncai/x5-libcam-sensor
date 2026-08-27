#include <stdio.h>
#include <stdint.h>

typedef uint32_t (*AGainToIndex_t)(double);
typedef double (*IndexToAGain_t)(uint32_t);
typedef uint32_t (*DGainToIndex_t)(double);
typedef double (*IndexToDGain_t)(uint32_t);

typedef struct {
	AGainToIndex_t againtoindex_callback;
	IndexToAGain_t indextoagain_callback;
	DGainToIndex_t dgaintoindex_callback;
	IndexToDGain_t indextodgain_callback;
} Callbacks;

//again to index
uint32_t againtoindex_function(double input) {

	uint32_t index = 0;

	if (input < 2.0) {
		index = (input - 1.0) / 0.0078125;
	} else if (input < 3.1) {
		index = 128 + (input / 2 - 1.0) / 0.0078125;
	} else if (input < 6.2) {
		index = 199 + (input / 3.1 - 1.0) / 0.0078125;
	} else if (input < 12.4) {
		index = 199 + 128 + (input / 6.2 - 1.0) / 0.0078125;
	} else if (input < 24.8) {
		index = 199 + 128 * 2 + (input / 12.4 - 1.0) / 0.0078125;
	} else if (input < 49.6) {
		index = 199 + 128 * 3 + (input / 24.8 - 1.0) / 0.0078125;
	} else if (input < 99.2) {
		index = 199 + 128 * 4 + (input / 49.6 - 1.0) / 0.0078125;
	} else {
		index = 199 + 128 * 5 + (input / 99.2 - 1.0) / 0.0078125;
	}

	return index;
}

//index to again
double indextoagain_function(uint32_t input) {

	double gain = 1.0;

	if (input < 128) {
		gain = 1.0 * (1.0 + input * 0.0078125);
	} else if (input < 199) {
		gain = 2.0 * (1.0 + (input - 128) * 0.0078125);
	} else if (input < 199 + 128) {
		gain = 3.1 * (1.0 + (input - 199) * 0.0078125);
	} else if (input < 199 + 128 * 2) {
		gain = 6.2 * (1.0 + (input - 199 - 128) * 0.0078125);
	} else if (input < 199 + 128 * 3) {
		gain = 12.4 * (1.0 + (input - 199 - 128 * 2) * 0.0078125);
	} else if (input  < 199 + 128 * 4) {
		gain = 24.8 * (1.0 + (input - 199 - 128 * 3) * 0.0078125);
	} else if (input < 199 + 128 * 5) {
		gain = 49.6 * (1.0 + (input - 199 - 128 * 4) * 0.0078125);
	} else if (input < 199 + 128 * 6) {
		gain = 99.2 * (1.0 + (input - 199 - 128 * 5) * 0.0078125);
	}

	return gain;
}

//dgain to index
uint32_t dgaintoindex_function(double input) {
	return (uint32_t)(input);
}

//index to dgain
double indextodgain_function(uint32_t input) {
	return (double)(input);
}


Callbacks cb = {againtoindex_function, indextoagain_function,
		dgaintoindex_function, indextodgain_function,};

//get_gain_index_callbacks
Callbacks* get_gain_index_callbacks() {
	return &cb;
}

