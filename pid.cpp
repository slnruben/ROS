#include "pid.h"
#include <iostream>

using namespace std;

Pid::Pid(){
	p = 0.0;
	ic = 0.0;
	d = 0.0; 
	ed = 0.0;
	ei = 0.0;
	w = 0.0;
	cont = 0;
	for (int i = 0; i < size; ++i)
		arrayI[i] = 0.0;
}

float Pid::getPid(float e){
	if (cont == 9)
		cont = (cont+1)/10;
	arrayI[cont] = e;
	cont++;
	for (int i = 0; i < 9; ++i)
		ei =+ arrayI[i];
	ei = ei/(float)(size -1);
	p = k1*e;
	ic = k2*ei;
	d = k3*ed;
	ed = e;
	w = p+ic+d;
	return w;
}	

