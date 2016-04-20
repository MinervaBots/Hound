## Log
Arduino log library. This library was made to avoid the tedious job of printing something and then commenting sereval lines of print commands when the information to be printed is not useful at that time. It was tested using the Serial object and the SoftwareSerial library (to use the bluetooth, for example), but it should work with (almost) any library that inherits from the Stream class, such as SD, EthernetClient, EthernetServer, ... (note that the library just prints stuff, you must handle the other operations according to the Stream class you are using).

------------------------
##Installation
1. [Download](https://github.com/brunocalou/Log/archive/master.zip) the latest release from github
2. Unzip and rename the folder to "Log" (remove the -master)
3. Paste the modified folder on your Library folder (.../arduino/libraries)
4. Reopen the Arduino IDE

------------------------

##Library Reference

###`Log` class

*  `Log()` - Constructor.

*  `void enable()` - Enables the log object. If it's enabled, it will
		print normally

* `void disable()` - Disables the log object. If it's disabled, it will
		not print

*  `void setTarget(Stream *stream)` - Set the target to print

*  `void setPriority(LogPriority priority)` - Set the priority to print. Only the messages
		with the same priority will be printed. If the
		priority of the object is VERBOSE, everything
		will be printed

*  `size_t write(uint8_t b)` - Calls the write method from the current stream

*  `size_t write(const uint8_t *buffer, size_t size)` - Calls the write method from the current stream

*  `size_t print(LogPriority priority, char const * tag, T msg, bool print_header=true)` - Prints the priority, the tag and the message in the following pattern:
		
      `priority/tag: message`
		
    and returns the number of bytes written. The message will be printed if

		- The log object is enabled and  
		- The message's priority is the same as the current priority or  
		- The current priority is setted to verbose  

*  `Log& operator<<(const T& msg)` - Overloads the << operator for a c++ style printing

*  `Log& operator<<(const LogPriority& priority)` - Overloads the << operator and change the priority

*  `size_t println(LogPriority priority, char const * tag, T msg)` - Calls the print method, prints the new line character "\n" and returns the number of bytes written

*  `void assert(char const * tag, T msg)`

*  `void debug(char const * tag, T msg)`

*  `void error(char const * tag, T msg)`

*  `void info(char const * tag, T msg)`

*  `void verbose(char const * tag, T msg)`

*  `void warn(char const * tag, T msg)`

*  **private** `Stream * stream` - The stream to log

*  **private** `LogPriority priority` - The priority to log

*  **private** `bool is_enabled` - Holds if the log object will log

*  **private** `LogPriority operator_priority` - Holds the current priority (used with the C++ style)

*  **private** `String operator_tag` - Holds the tag (used with the C++ style)

*  **private** `bool operator_print_tag` - Holds if the print method will print the header (used with the C++ style)

*  **private** `bool get_tag` - Holds if the tag must be saved (used with the C++ style)

####`LogPriority` enum

*  `ASSERT` - `a`
*  `DEBUG` - `d`
*  `ERROR` - `e`
*  `INFO` - `i`
*  `VERBOSE` - `v`
*  `WARN` - `w`

###`global`

*  `const char * log_endl` - '\n'
