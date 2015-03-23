#ifndef pid_H
#define pid_H

class Pid{
public:
	Pid();
	float getPid(float e);


private:
	static const float k1 = 0.87;
	static const float k2 = 0.05;
	static const float k3 = 0.08;  
	float ed, ei, w, p, ic, d;
	int cont;
	static const int size = 10;
	float arrayI [size];
};

#endif