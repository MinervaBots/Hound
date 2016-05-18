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

			char incoming_char = (char) serial->read();

			if(incoming_char == RESET) {
				reset();
			}
			Serial.print(incoming_char);
			// Serial.print("Fora do if: \t");
			// Serial.print(_size);Serial.print("\t");Serial.print(i);Serial.print("\t");
			// Serial.println(incoming_char);

			if(incoming_char != TOKEN && incoming_char != '\n') {
				str += incoming_char;
			} else if(i < _size) {
				//Checks if the number of encoders sent on the serial
				//is equals to the expected
				this->get(i)->add(str.toInt());
				str = "";
				i ++;
				delay(200);
			}
			has_read = true;
		}

		if(has_read) {
			serial->println(ACKNOWLEDGEMENT);
			Serial.println();

		}
		return has_read;
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
