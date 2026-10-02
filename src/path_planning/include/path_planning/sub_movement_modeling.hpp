#include <cmath>
#include <bitset>
#include <path_planning/BigInt.hpp>

inline constexpr float PI = 3.14159265358979323846f;
inline constexpr unsigned char NUM_INPUTS = 23;
inline constexpr unsigned char NUM_OUTPUTS = 12;

namespace {
	constexpr unsigned char round(float value) {
		int floor_value = (int) value;

		if(value > 0 && value - floor_value > 0.5f) {
			return ++floor_value;
		} else if (value < 0 && value - floor_value < -0.5f) {
			return --floor_value;
		} else {
			return floor_value;
		}
	}

	
	struct function_variable {
		//All ranges are by default centered on 0, and the resulting possible values for the variable will be [-range/2, range/2]
		float range;
		//Possible partitions numbers are [0, partitions]
		unsigned char partitions;
		float min;
		float max;
		float step;

		constexpr function_variable(float range, unsigned char partitions) :
			range(range),
			partitions(partitions),
			min(-range / 2.0f),
			max(-min),
			step(range / partitions)
		{}

		constexpr function_variable(float min, float max,  unsigned char partitions) :
			range(max - min),
			partitions(partitions),
			min(min),
			max(max),
			step(range / partitions)
		{}

		constexpr float calc_value(unsigned char partition_number) const {
			return min + partition_number * step;
		}

		inline float calc_value(BigInt partition_number) const {
			return calc_value(static_cast<unsigned char>(partition_number.to_int()));
		}

		constexpr unsigned char calc_partition_number(float value) const {
			if(value > max) {
				return partitions;
			} else if(value < min) {
				return 0;
			} else {
				return round((value - min) / step);
			}
		}
	};

	constexpr function_variable QUATERNION[4] = {function_variable{2.0f, 127}, function_variable{2.0f, 127}, function_variable{2.0f, 127}, function_variable{2.0f, 127}};
	constexpr function_variable LOCAL_LINEAR_VELOCITY[3] = {function_variable{10.0f, 127}, function_variable{10.0f, 127}, function_variable{10.0f, 127}};
	constexpr function_variable LOCAL_ANGULAR_VELOCITY[3] = {function_variable{4.0f * PI, 127}, function_variable{4.0f * PI, 127}, function_variable{4.0f * PI, 127}};
	constexpr function_variable LOCAL_LINEAR_ACCELERATION[3] = {function_variable{100.0f, 127}, function_variable{100.0f, 127}, function_variable{100.0f, 127}};
	constexpr function_variable LOCAL_ANGULAR_ACCELERATION[3] = {function_variable{40.0f * PI, 127}, function_variable{40.0f * PI, 127}, function_variable{40.0f * PI, 127}};
	constexpr function_variable COMMANDED_LINEAR_VELOCITY[3] = {function_variable{10.0f, 127}, function_variable{10.0f, 127}, function_variable{10.0f, 127}};
	constexpr function_variable COMMANDED_ANGULAR_VELOCITY[3] = {function_variable{4.0f * PI, 127}, function_variable{4.0f * PI, 127}, function_variable{4.0f * PI, 127}};
	constexpr function_variable DELTA_TIME{0.0f, 1.0f, 127};
	constexpr function_variable DELTA_QUATERNION[3] = {function_variable{2.0f, 127}, function_variable{2.0f, 127}, function_variable{2.0f, 127}};
	constexpr function_variable DELTA_LOCAL_LINEAR_POSITION[3] = {function_variable{1.0f, 127}, function_variable{1.0f, 127}, function_variable{1.0f, 127}};

	constexpr function_variable INPUT_ORDER[NUM_INPUTS] = {
		QUATERNION[0], QUATERNION[1], QUATERNION[2], QUATERNION[3],
		LOCAL_LINEAR_VELOCITY[0], LOCAL_LINEAR_VELOCITY[1], LOCAL_LINEAR_VELOCITY[2],
		LOCAL_ANGULAR_VELOCITY[0], LOCAL_ANGULAR_VELOCITY[1], LOCAL_ANGULAR_VELOCITY[2],
		LOCAL_LINEAR_ACCELERATION[0], LOCAL_LINEAR_ACCELERATION[1], LOCAL_LINEAR_ACCELERATION[2],
		LOCAL_ANGULAR_ACCELERATION[0], LOCAL_ANGULAR_ACCELERATION[1], LOCAL_ANGULAR_ACCELERATION[2],
		COMMANDED_LINEAR_VELOCITY[0], COMMANDED_LINEAR_VELOCITY[1], COMMANDED_LINEAR_VELOCITY[2],
		COMMANDED_ANGULAR_VELOCITY[0], COMMANDED_ANGULAR_VELOCITY[1], COMMANDED_ANGULAR_VELOCITY[2],
		DELTA_TIME
	};
	constexpr function_variable OUTPUT_ORDER[NUM_OUTPUTS] = {
		LOCAL_LINEAR_VELOCITY[0], LOCAL_LINEAR_VELOCITY[1], LOCAL_LINEAR_VELOCITY[2],
		LOCAL_ANGULAR_VELOCITY[0], LOCAL_ANGULAR_VELOCITY[1], LOCAL_ANGULAR_VELOCITY[2],
		DELTA_QUATERNION[0], DELTA_QUATERNION[1], DELTA_QUATERNION[2],
		DELTA_LOCAL_LINEAR_POSITION[0], DELTA_LOCAL_LINEAR_POSITION[1], DELTA_LOCAL_LINEAR_POSITION[2]
	};

	constexpr unsigned char calc_bits(const unsigned char num_vars, const function_variable order[]) {
		double log_totals = 0;

		for(int i = 0; i < num_vars; ++i) {
			log_totals += std::log2(order[i].partitions);
		}

		return std::ceil(log_totals);
	}

	//Total number of bits to uniquely represent all input/output variables combinations (i.e. multiply possible number of partitions together and take log_2)
	constexpr unsigned char INPUT_BITS = calc_bits(NUM_INPUTS, INPUT_ORDER);
	constexpr unsigned char OUTPUT_BITS = calc_bits(NUM_OUTPUTS, OUTPUT_ORDER);

	template <typename UInt> constexpr UInt calc_to_bits_helper(const unsigned char num_vars, const function_variable order[], float src[]) {
		UInt partition_total = 0;

		for(int i = 0; i < num_vars; ++i) {
			if(i != 0) {
				partition_total *= order[i - 1].partitions;
			}

			partition_total += order[i].calc_partition_number(src[i]);
		}

		return partition_total;
	}

	template <unsigned char num_bits> constexpr std::bitset<num_bits> calc_to_bits(const unsigned char num_vars, const function_variable order[], float src[]) {
		if constexpr(num_bits <= 8 * sizeof(unsigned int)) {
			return std::bitset<num_bits>(calc_to_bits_helper<unsigned int>(num_vars, order, src));
		} else if constexpr(num_bits <= 8 * sizeof(unsigned long)) {
			return std::bitset<num_bits>(calc_to_bits_helper<unsigned long>(num_vars, order, src));
		} else if constexpr(num_bits <= 8 * sizeof(unsigned long long)) {
			return std::bitset<num_bits>(calc_to_bits_helper<unsigned long long>(num_vars, order, src));
		} else {
			return std::bitset<num_bits>(calc_to_bits_helper<BigInt>(num_vars, order, src).to_string());
		}
	}

	
	template <typename UInt> constexpr void calc_from_bits_helper(const unsigned char num_vars, const function_variable order[], UInt bits_uint, float dest[]) {
		for(int i = num_vars; i >= 0; --i) {
			if(i != 0) {
				dest[i] = order[i].calc_value(bits_uint % order[i - 1].partitions);
				bits_uint -= dest[i];
				bits_uint /= order[i -1].partitions;
			} else {
				dest[i] = order[i].calc_value(bits_uint);
			}
		}
	}

	template <unsigned char num_bits> constexpr void calc_from_bits(const unsigned char num_vars, const function_variable order[], std::bitset<num_bits> bits, float dest[]) {
		if constexpr(num_bits <= 8 * sizeof(unsigned int)) {
			calc_from_bits_helper<unsigned int>(num_vars, order, static_cast<unsigned int>(bits.to_ulong()), dest);
		} else if constexpr(num_bits <= 8 * sizeof(unsigned int)) {
			calc_from_bits_helper<unsigned long>(num_vars, order, bits.to_ulong(), dest);
		} else if constexpr(num_bits <= 8 * sizeof(unsigned long long)) {
			calc_from_bits_helper<unsigned long long>(num_vars, order, bits.to_ullong(), dest);
		} else {
			calc_from_bits_helper<BigInt>(num_vars, order, BigInt(bits.to_string()), dest);
		}
	}
}

std::bitset<INPUT_BITS> calc_inputs_to_bits(float inputs[]) {
	return calc_to_bits<INPUT_BITS>(NUM_INPUTS, INPUT_ORDER, inputs);
}

std::bitset<OUTPUT_BITS> calc_outputs_to_bits(float outputs[]) {
	return calc_to_bits<OUTPUT_BITS>(NUM_OUTPUTS, OUTPUT_ORDER, outputs);
}

void calc_inputs_from_bits(std::bitset<INPUT_BITS> inputs_bits, float inputs_dest[]) {
	calc_from_bits<INPUT_BITS>(NUM_INPUTS, INPUT_ORDER, inputs_bits, inputs_dest);
}

void calc_outputs_from_bits(std::bitset<OUTPUT_BITS> outputs_bits, float outputs_dest[]) {
	calc_from_bits<OUTPUT_BITS>(NUM_OUTPUTS, OUTPUT_ORDER, outputs_bits, outputs_dest);
}
