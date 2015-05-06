/*
 * Common.h
 *
 *  Created on: 08/12/2014
 *      Author: paco
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <string>
#include <math.h>

bool isPrefix(std::string const& s1, std::string const&s2);

inline double normalizePi(double data)
{
  if (data < M_PI && data >= -M_PI) return data;
  double ndata = data - ((int )(data / (M_PI*2.0)))*(M_PI*2.0);
  while (ndata >= M_PI)
  {
    ndata -= (M_PI*2.0);
  }
  while (ndata < -M_PI)
  {
    ndata += (M_PI*2.0);
  }
  return ndata;
}

#endif /* COMMON_H_ */
