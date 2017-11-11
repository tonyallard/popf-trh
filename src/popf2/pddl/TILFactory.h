/*
 * TILFactory.h
 *
 *  Created on: 20 October 2017
 *      Author: tony
 */

#ifndef __TILFACTORY
#define __TILFACTORY

#include <string>
#include <list>

#include "TIL.h"

#include "../RPGBuilder.h"

namespace PDDL {

class TILFactory {
public:

	static TILFactory * getInstance();

	PDDL::TIL getTIL(Planner::RPGBuilder::FakeTILAction aTIL, int tilIndex,
		std::list<std::pair<std::string, std::string> > constants = std::list<
				std::pair<std::string, std::string> >());
private:
	//Singleton Instance
	static TILFactory * INSTANCE;
	
	//Private constructor
	TILFactory(){};
	//Singleton
	TILFactory(TILFactory const & other) {
	}
	;
	//Singleton
	TILFactory& operator=(TILFactory const&) {
	}
	;
};

}

#endif /* __TILFACTORY */
