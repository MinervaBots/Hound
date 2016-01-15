#include "encoderlist.h"

EncoderList::EncoderList(Stream *serial){
	this->serial = serial;
}

EncoderList::~EncoderList() {

}

void EncoderList::addEncoder(SimpleEncoder * encoder) {
	this->add(encoder);
}

bool EncoderList::read() {

	//If the list is not empty
	if(_size != 0) {

		String str = "";
		int i = 0; //current encoder index
		bool has_read = false;

		while(serial->available()) {
			has_read = true;
			char incoming_char = (char) serial->read();

			if(incoming_char == RESET) {
				reset();
			}

			if(incoming_char != TOKEN) {
				str += incoming_char;
			} else {
				//Checks if the number of encoders sent on the serial
				//is equals to the expected
				if(i < _size) {
					this->get(i)->add(str.toInt());
					str = "";
					i ++;
				}
			}
		}

		if(has_read) {
			this->get(i)->add(str.toInt());
			serial->println(ACKNOWLEDGEMENT);
		}
		// serial->print("message = ");
		// print();

		return true;
	}
	return false;
}

void EncoderList::print() {
	String message = "";

	for(int i = 0; i < _size - 1; i++) {
		message += this->get(i)->getPulses();
		message += TOKEN;
	}

	if(_size != 0) {
		message += this->get(_size - 1)->getPulses();
		serial->println(message);
	}
}

void EncoderList::reset() {
	for(int i = 0; i < _size; i++) {
		this->get(i)->reset();
	}
	serial->println(RESET);
}

void EncoderList::start() {
	serial->println(ACKNOWLEDGEMENT);
}