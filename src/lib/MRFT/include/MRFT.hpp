#ifndef MRFT_HPP
#define MRFT_HPP

#include <cmath>
#include "DataTypes.hpp"
#include "Timer.hpp"

namespace HEAR{

class MRFT_Block{

private:
	Timer _timer;
	bool _enable = false;
	bool first_run = true;
	float last_output;
	float e_max;
	float e_min;
	bool has_reached_min;
	bool has_reached_max;
	float peak_conf_counter;
	float input_bias = 0.0;
	bool record_input_bias = false;
	MRFT_parameters parameters;

public:
	float mrft_anti_false_switching(float err);
    	void update_params(MRFT_parameters* para);

	MRFT_Block(int b_uid);
	~MRFT_Block();
	void process();
	void process(float &command, float error);
	void update(UpdateMsg* u_msg);
	void update(bool enable);
	void reset();
};

}

#endif
