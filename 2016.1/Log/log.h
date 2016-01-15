/*
	log.h - The Log Class logs information on any Stream that inherits from
	the Print class.

	Created by Bruno Calou Alves, May, 2015.
	Read LICENSE for more information.
*/

#ifndef LOG_H
#define LOG_H

#include <inttypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifndef log_endl
#define log_endl '\n'
#endif

enum LogPriority {
	ASSERT 	= 'a',
	DEBUG  	= 'd',
	ERROR  	= 'e',
	INFO   	= 'i',
	VERBOSE = 'v',
	WARN   	= 'w'
};

class Log : public Print {
public:
	Log() {
		stream = &Serial;
		priority = VERBOSE;
		is_enabled = true;
		operator_tag = "";
		operator_priority = VERBOSE;
		operator_print_tag = true;
		get_tag = false;
	}

	/*
		Enables the log object. If it's enabled, it will
		print normally
	*/
	void enable() {
		is_enabled = true;
	}

	/*
		Disables the log object. If it's disabled, it will
		not print
	*/
	void disable() {
		is_enabled = false;
	}

	/*
		Set the target to print
	*/
	void setTarget(Stream *stream) {
		this->stream = stream;
	}

	/*
		Set the priority to print. Only the messages
		with the same priority will be printed. If the
		priority of the object is VERBOSE, everything
		will be printed
	*/
	void setPriority(LogPriority priority) {
		this->priority = priority;
	}

	/*
		Calls the write method from the current stream
	*/	
	size_t write(uint8_t b) {
		stream->write(b);
	}

	/*
		Calls the write method from the current stream
	*/
	size_t write(const uint8_t *buffer, size_t size) {
		stream->write(buffer, size);
	}

	/*
		Prints the priority, the tag and the message in the following pattern:

		priority/tag: message

		and returns the number of bytes written. The message will be printed if:
		- The log object is enabled and
		- The message's priority is the same as the current priority or
		- The current priority is setted to verbose
	*/
	template <typename T>

	size_t print(LogPriority priority, char const * tag, T msg, bool print_header=true) {
		
		size_t bytes_written = 0;

		if(is_enabled && (this->priority == priority || this->priority == VERBOSE)) {
			if(print_header) {
				String full_message = "";
				full_message += (char)priority;
				full_message += "/";
				full_message += tag;
				full_message += ": ";
				bytes_written += print(full_message);
			}
			bytes_written += print(msg);
		}

		return bytes_written;
	}

	/*
		Calls the print method, prints the new line character "\n"
		and returns the number of bytes written
	*/
	template <typename T>

	size_t println(LogPriority priority, char const * tag, T msg) {

		size_t bytes_written = print(priority, tag, msg);

		if(bytes_written) {
			bytes_written += print("\n");
		}

		return bytes_written;
	}

	/*
		Overloads the << operator for a c++ style printing
	*/
	template <typename T>

	Log& operator<<(const T& msg) {
		if(get_tag) {
			operator_tag += msg;
		} else {
			//Convert string to char const *
			int size = operator_tag.length() + 1;
			char tag[size];
			operator_tag.toCharArray(tag, size);

			print(operator_priority, tag, msg, operator_print_tag);
			operator_print_tag = false;
		}
		get_tag = false;
		return *this;
	}

	/*
		Overloads the << operator and change the priority
	*/
	Log& operator<<(const LogPriority& priority) {
		operator_priority = priority;
		get_tag =true;
		operator_tag = "";
		operator_print_tag = true;
		return *this;
	}

	template <typename T>

	void assert(char const * tag, T msg) {
		println(ASSERT, tag, msg);
	}
	
	template <typename T>

	void debug(char const * tag, T msg) {
		println(DEBUG, tag, msg);
	}
	
	template <typename T>

	void error(char const * tag, T msg) {
		println(ERROR, tag, msg);
	}

	template <typename T>
	
	void info(char const * tag, T msg) {
		println(INFO, tag, msg);
	}

	template <typename T>
	
	void verbose(char const * tag, T msg) {
		println(VERBOSE, tag, msg);
	}
	
	template <typename T>

	void warn(char const * tag, T msg) {
		println(WARN, tag, msg);
	}

	using Print::print;
	using Print::println;
	
private:
	Stream * stream;
	LogPriority priority;
	bool is_enabled;

	//C++ style
	LogPriority operator_priority;
	String operator_tag;
	bool operator_print_tag;
	bool get_tag;
};

#endif //LOG_H