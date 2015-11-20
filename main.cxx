#include <iostream>
#include <cstdio>

#include "QuaternionRotator.h"

int main( int argc, char * argv[] )
{
	double v0[3] = {1, 1, 1};
	double v1[3] = {2, 1, 2};
	QuaternionRotator * qr = new QuaternionRotator;
	qr->SetInputs(v0,v1);
	double m[3][3];
	qr->GetOutput(m);
	for( int i = 0; i < 3; i++ )
	{
		for( int j = 0; j < 3; j++ )
		{
                    printf("%.3f\t", m[i][j]);
		}
		std::cout << std::endl;
	}
    return 0;
}
