/*
 * Common.cpp
 *
 *  Created on: 08/12/2014
 *      Author: paco
 */

#include "Common.h"

bool isPrefix(std::string const& s1, std::string const&s2)
{
	return s1.compare(s2.substr(0, s1.length()))==0;

}

