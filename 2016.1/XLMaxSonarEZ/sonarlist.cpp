#include "sonarlist.h"

SonarList::SonarList(Sonar::OperationMode operation_mode){
	this->operation_mode = operation_mode;
	is_loop_started = false;
}

SonarList::~SonarList() {

}

void SonarList::addSonar(XLMaxSonarEZ * sonar) {
	this->add(sonar);
}

void SonarList::addFirst(XLMaxSonarEZ * sonar) {
	this->unshift(sonar);
}

void SonarList::read() {

	//If the list is not empty
	if(_size != 0) {

		if(operation_mode == Sonar::CHAIN || operation_mode == Sonar::SIMULTANEOUS) {
			//Trigger the first sonar only
			this->get(0)->trigger();

		} else if(operation_mode == Sonar::CHAIN_LOOP && !is_loop_started) {
			//Trigger the first sonar once and put the rx pin
			//on high impedance
			
            if(this->get(0)->getRX() != UNUSED) {
            	this->get(0)->trigger();
				pinMode(this->get(0)->getRX(), INPUT);
				is_loop_started = true;
			}

		} else if(operation_mode == Sonar::SINGLE) {
			//Trigger all the sonars
			for(int i = 0; i < size(); i++) {
				this->get(i)->trigger();
			}
		}
		
		//Read all the sonars
		for(int i = 0; i < size(); i++) {
			this->get(i)->read();
		}
	}
}
