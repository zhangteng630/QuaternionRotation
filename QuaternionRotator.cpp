#include "QuaternionRotator.h"


QuaternionRotator::QuaternionRotator(void)
{
}


QuaternionRotator::~QuaternionRotator(void)
{
}

void QuaternionRotator::SetInputs( double v0[3], double v1[3] )
{
	memcpy( m_v0, v0, 3*sizeof(double) );
	memcpy( m_v1, v1, 3*sizeof(double) );
}

void QuaternionRotator::CrossProduct( double u[3], double v[3], double output[3] )
{
	//reference: http://en.wikipedia.org/wiki/Cross_product
	output[0] = u[1]*v[2]-u[2]*v[1];
	output[1] = u[2]*v[0]-u[0]*v[2];
	output[2] = u[0]*v[1]-u[1]*v[0];
}

double QuaternionRotator::Length( double *v, int size )
{
	if( size <= 0 )
		return 0.0;
	double L2 = 0.0;
	for( int i = 0; i < size; i++ )
	{
		L2 += v[i]*v[i];
	}
	return std::sqrt( L2 );	
}

void QuaternionRotator::Normalize( double *v, int size )
{
	double L = this->Length( v, size );
	if( v == 0)
		return;
	for( int i = 0; i < size; i++ )
	{
		v[i] = v[i]/L;
	}
}

double QuaternionRotator::DotProduct( double *u, double *v, int size )
{
	double d = 0.0;
	for( int i = 0; i < size; i++ )
		d += u[i]*v[i];
	return d;
}

void QuaternionRotator::CalculateQuaternion( void )
{
	//reference: http://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
	double cross[3];
	this->CrossProduct( m_v0, m_v1, cross );
	for( int i = 0; i < 3; i++ )
		m_quaternion[i+1] = cross[i];
	m_quaternion[0] = this->Length( m_v0, 3 ) * this->Length( m_v1, 3 ) + this->DotProduct( m_v0, m_v1, 3 );
	this->Normalize( m_quaternion, 4 );
}

void QuaternionRotator::CalculateRotationMatix( void )
{
	//reference: http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Using_quaternion_rotations
	m_rotation[0][0] = 1 - 2*m_quaternion[2]*m_quaternion[2] - 2*m_quaternion[3]*m_quaternion[3];
	m_rotation[0][1] = 2 * ( m_quaternion[1]*m_quaternion[2] + m_quaternion[3]*m_quaternion[0] );
	m_rotation[0][2] = 2 * ( m_quaternion[1]*m_quaternion[3] - m_quaternion[2]*m_quaternion[0] );
	m_rotation[1][0] = 2 * ( m_quaternion[1]*m_quaternion[2] - m_quaternion[3]*m_quaternion[0] );
	m_rotation[1][1] = 1 - 2*m_quaternion[1]*m_quaternion[1] - 2*m_quaternion[3]*m_quaternion[3];
	m_rotation[1][2] = 2 * ( m_quaternion[2]*m_quaternion[3] + m_quaternion[1]*m_quaternion[0] );
	m_rotation[2][0] = 2 * ( m_quaternion[1]*m_quaternion[3] + m_quaternion[2]*m_quaternion[0] );
	m_rotation[2][1] = 2 * ( m_quaternion[2]*m_quaternion[3] - m_quaternion[1]*m_quaternion[0] );
	m_rotation[2][2] = 1 - 2*m_quaternion[1]*m_quaternion[1] - 2*m_quaternion[2]*m_quaternion[2];
}

void QuaternionRotator::GetOutput( double m[3][3] )
{
	this->Normalize( m_v0, 3 );
	this->Normalize( m_v1, 3 );
	this->CalculateQuaternion();
	this->CalculateRotationMatix();
	memcpy( m, m_rotation, 9*sizeof(double) );
}
