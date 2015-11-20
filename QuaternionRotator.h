#ifndef __QuaternionRotator_h__
#define __QuaternionRotator_h__

/*
 * Author: Zhang Teng, PhD Candidate, DIIR, CUHK
 * Mailbox: zhangteng630@gmail.com
 * Purpose: Find a rotation matrix from two input vectors
 */

#include <cstring>
#include <cmath>

class QuaternionRotator
{
public:
	QuaternionRotator(void);
	~QuaternionRotator(void);
	void SetInputs( double dest[3], double src[3] );
	void GetOutput( double rotation[3][3] );

private:
	void CrossProduct( double v0[3], double v1[3], double output[3] );
	double DotProduct( double *v, double *u, int size );
	void Normalize( double *v, int size );
	double Length( double *v, int size );
	void CalculateQuaternion( void );
	void CalculateRotationMatix( void );

private:
	double m_v0[3];
	double m_v1[3];
	double m_quaternion[4];
	double m_rotation[3][3];
};

#endif
