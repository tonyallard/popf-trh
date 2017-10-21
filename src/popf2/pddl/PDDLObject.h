/*
 * PDDLObject.h
 *
 *  Created on: 25 Oct 2015
 *      Author: tony
 */

#ifndef COLIN_PDDL_PDDLOBJECT_H_
#define COLIN_PDDL_PDDLOBJECT_H_

#include <iostream>
#include <string>
#include <list>

using namespace std;
namespace PDDL{

class PDDLObject {
private:
	string name;
	list<string> type;
public:
	PDDLObject(string name, list<string> type) : name(name), type(type) {};
	string getName() const {return name;};
	string getTypeString() const;

	bool operator<(const PDDLObject & other) const;
	bool operator==(const PDDLObject & other);

	friend ostream & operator<<(ostream & out, const PDDLObject & pddlObject);
};
}


#endif /* COLIN_PDDL_PDDLOBJECT_H_ */
