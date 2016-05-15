#ifndef ENCODERLIST_H
#define ENCODERLIST_H

#include <inttypes.h>
#include <Arduino.h>

/*
	To get the linked list class, if not available, check https://github.com/ivanseidel/LinkedList
*/
#ifndef LinkedList_h
#include "LinkedList.h"
#endif

#include "simpleencoder.h"

#define ACKNOWLEDGEMENT 'a'
#define TOKEN 			','
#define RESET 			'r'
#define LIGHT_ON 		'l'
#define LIGHT_OFF 		'o'

class EncoderList: public LinkedList<SimpleEncoder*> {
public:
	EncoderList(Stream *serial);
	~EncoderList();

	/*
		Adds an encoder to the list
	*/
	void addEncoder(SimpleEncoder * encoder);

	/*
		Reads the encoders on the serial stream and
		sends an acknowledgement character.
	*/
	bool read();

	/*
		Prints all the encoders with the specified format
	*/
	void print();

	/*
		Resets all the encoders
	*/
	void reset();

	/*
		Start the communication
	*/
	void start();

private:
	Stream *serial;

};

#endif //ENCODERLIST_H
