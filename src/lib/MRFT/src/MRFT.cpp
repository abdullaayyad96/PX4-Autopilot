#include "MRFT.hpp"

namespace HEAR{

MRFT_Block::MRFT_Block(int b_uid){
}

MRFT_Block::~MRFT_Block() {
}

void MRFT_Block::process() {
	if(_enable){
		float input = 0, bias =0;
		if(record_input_bias){
			record_input_bias = false;
			input_bias = input;
		}
		float command = this->mrft_anti_false_switching(input-input_bias) + bias;
	}
}

void MRFT_Block::process(float &command, float error) {
	if(_enable){
		command = this->mrft_anti_false_switching(error);
	}
	else {
		command = 0.0f;
	}
}

void MRFT_Block::reset(){
	first_run = true;
	last_output = 0;
	e_max = 0;
	e_min = 0;
	has_reached_min = 0;
	has_reached_max = 0;
	peak_conf_counter = 0;
}

void MRFT_Block::update(UpdateMsg* u_msg){
	if(u_msg->getType() == UPDATE_MSG_TYPE::MRFT_UPDATE){
		auto mrft_update_params = ((MRFT_UpdateMsg*)u_msg)->param;
		if(true){ //mrft_update_params.id == _id){ //TODO: correct
			parameters = mrft_update_params;
		}
	}
	else if(u_msg->getType() == UPDATE_MSG_TYPE::BOOL_MSG){
		_enable = ((BoolMsg*)u_msg)->data;
		if(_enable == true){
			record_input_bias = true;
			this->reset();
		}
	}

}

void MRFT_Block::update(bool enable){
	_enable = enable;
	if(_enable == true){
		record_input_bias = true;
		this->reset();
	}

}

void MRFT_Block::update_params(MRFT_parameters* para){
	parameters.beta = para->beta;
	parameters.relay_amp = para->relay_amp;
	parameters.no_switch_delay_in_ms = para->no_switch_delay_in_ms;
	parameters.num_of_peak_conf_samples = para->num_of_peak_conf_samples;
	parameters.id = para->id;

}

float MRFT_Block::mrft_anti_false_switching(float err){
	// MRFT algorithm with false switching prevention mechanism
	// June 2020
	// Coded By M. Chehadeh, Khalifa University
	// Translated to C++ By Pedro Silva, Khalifa University

	// mode_of_operation=0 take last e_max or e_min
	// mode_of_operation=1 take current e_max or e_min and mirror it
	float h = parameters.relay_amp;
	float beta = parameters.beta;
	int no_switch_delay_in_ms = parameters.no_switch_delay_in_ms;
	int num_of_peak_conf_samples = parameters.num_of_peak_conf_samples;
	int mode_of_operation = 1;
	float output=0;
	float e_max_o=0;
	float e_min_o=0;
	float sw_max_o=0;
	float sw_min_o=0;

	if(first_run){
		first_run = false;
		_timer.tick();
		peak_conf_counter=0;
		e_max=0;
		e_min=0;
		if(err>0){
			has_reached_max=true;
			has_reached_min=false;
			last_output=h;
		}else{
			has_reached_max=false;
			has_reached_min=true;
			last_output=-h;
		}
		output = last_output;
		return output;
	}

	output = last_output;

	if (_timer.tockMilliSeconds() <= no_switch_delay_in_ms){
		e_min_o = e_min;
		e_max_o = e_max;
		return output;
	}

	if (last_output<0){

		if (err<e_min){
			e_min=err;
			peak_conf_counter=0;
		}else{
			peak_conf_counter=peak_conf_counter+1;
			if (peak_conf_counter>=num_of_peak_conf_samples){
				has_reached_min=true;
				peak_conf_counter=0;
			}
			e_max=err;
		}
		float sw_min;
		if (mode_of_operation==0){
			sw_min = ((e_max-e_min)/2)+e_min+beta*((e_max-e_min)/2);
		}else if (mode_of_operation==1){
			float e_max_star=-e_min;
			sw_min = ((e_max_star-e_min)/2)+e_min+beta*((e_max_star-e_min)/2);
		}
		sw_min_o = sw_min;
		if (has_reached_min){
			if (err>sw_min){
				output=h;
				_timer.tick();
				has_reached_min=false;
			}else{
				output=last_output;
			}
		}
	}else{
		if (err>e_max){
			e_max=err;
			peak_conf_counter=0;
		}else{
			peak_conf_counter=peak_conf_counter+1;
			if (peak_conf_counter>=num_of_peak_conf_samples){
				has_reached_max=true;
				peak_conf_counter=0;
			}
			e_min=err;
		}
		float sw_max;
		if (mode_of_operation==0){
			sw_max=e_max-((e_max-e_min)/2)-(beta*((e_max-e_min)/2));
		}else if (mode_of_operation==1){
			float e_min_star=-e_max;
			sw_max=e_max-((e_max-e_min_star)/2)-(beta*((e_max-e_min_star)/2));
		}
		sw_max_o=sw_max;
		if (has_reached_max){
			if (err<sw_max){
				output=-h;
				_timer.tick();
				has_reached_max=false;
			}else{
				output=last_output;
			}
		}
	}
	e_min_o=e_min;
	e_max_o=e_max;
	last_output=output;
	return output;
}

}
