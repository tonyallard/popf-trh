/*
 * Util.cpp
 *
 *  Created on: 5 Sep 2015
 *      Author: tony
 */

#include "Util.h"
#include <stdlib.h>

namespace Util {
	bool isDouble(const char* str)
	{
		char* endptr = 0;
		strtod(str, &endptr);

		if(*endptr != '\0' || endptr == str)
			return false;
		return true;
	}
}