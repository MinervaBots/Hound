/*
	sonarlist.h - Sonar list is a linked list of XLMaxSonarEZ objects. It
	is capable of handling the all the operation modes described on the
	datasheet.

	Created by Bruno Calou Alves, March, 2015.
	Read LICENSE for more information.
*/

#ifndef SONARLIST_H
#define SONARLIST_H

/*
	To get the linked list class, if not available, check https://github.com/ivanseidel/LinkedList
*/
#ifndef LinkedList_h
#include "../LinkedList/LinkedList.h"
#endif

#include "XLMaxSonarEZ.h"

class SonarList: public LinkedList<XLMaxSonarEZ*> {
public:	
	SonarList(Sonar::OperationMode operation_mode);
	~SonarList();

	/*
		Add a sonar to the list
	*/
	void addSonar(XLMaxSonarEZ * sonar);

	/*
		Add a sonar to the beginning of the list. This sonar
		is the one that will be triggered (unless the
		operation mode is setted to Single).
	*/
	void addFirst(XLMaxSonarEZ * sonar);

	/*
		Performs a reading according to the current operation
		mode. This method will trigger and read all the sonars
		properly. To access the values, use the getDistance
		method on each object.
	*/
	void read();

private:
	Sonar::OperationMode operation_mode;

	//Holds if the loop has started on the chain loop mode
	bool is_loop_started;
};

#endif //SONARLIST_H
