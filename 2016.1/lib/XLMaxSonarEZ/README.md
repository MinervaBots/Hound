## XLMaxSonarEZ
Library to read the XLMaxSonarEZ sensor on Arduino.

------------------------
##Installation
1. Install the linked list library from https://github.com/ivanseidel/LinkedList
2. [Download](https://github.com/brunocalou/XLMaxSonarEZ/archive/master.zip) the latest release from github
3. Unzip and rename the folder to "XLMaxSonarEZ" (remove the -master)
4. Paste the modified folder on your Library folder (.../arduino/libraries)
5. Reopen the Arduino IDE

------------------------
##Library Reference

###`XLMaxSonarEZ` class

*  `XLMaxSonarEZ(byte tx, byte rx = UNUSED)` - Constructor.

*  `~XLMaxSonarEZ()` - Destructor.

*  `void trigger()` - Trigger the RX pin for 20 microseconds, so the sensor starts reading.

*  `float read()` - Perform an analog read on the TX pin and converts to a unit of length.

*  `float getDistance()` - Get the last distance read. This method will NOT read the sensor (use the read method instead).
*  `byte getRX()` - Get RX pin.

*  `byte getTX()` - get TX pin.

*  **private** `byte rx` - RX pin.

*  **private** `byte tx` - TX pin.

*  **private** `float distance` - The last distance measured.

###`Sonar` namespace

*  `enum OperationMode` - All operation modes defined on the datasheet.
  - `CHAIN`: First sensor is triggered before reading.
  - `CHAIN_LOOP`: First sensor is triggered only once, regardless the
  number of subsequent readings.
  - `SIMULTANEOUS`: All the sensors use the same RX pin and are triggered at
  the same time.
  - `SINGLE`: There is a RX pin for each individual sensor. They are triggered
  before reading.

###`SonarList` class

*  `SonarList(Sonar::OperationMode operation_mode)` - Constructor.

*  `~SonarList()` - Destructor.

*  `void addSonar(XLMaxSonarEZ * sonar)` - Add a sonar to the list.

*  `void addFirst(XLMaxSonarEZ * sonar)` - Add a sonar to the beginning of the list. This sonar is the one that will be 
triggered (unless the operation mode is setted to Single).

*  `void read()` - Performs a reading according to the current operation mode. This method will trigger and read all the sonars properly. To access the values, use the getDistance method on each object.

*  **private** `Sonar::OperationMode operation_mode` - Current operation mode.

*  **private** `bool is_loop_started` - Holds if the loop has started on the chain loop mode.

###`Constants`

*  `const float SONAR_TO_CM` - Convert to centimeter.
*  `int UNUSED` - Represents an unused pin.
