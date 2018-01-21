#include "vector3d.h"
#include "cinder/app/AppBasic.h"

#include <iostream>

float tripleProduct(const Vector3D &v1, const Vector3D &v2, const Vector3D &v3)
{
    return dot((cross(v1,v2)),v3);
}

void Vector3D::normalize()
{
    if( !(fabs(len) < 1.0e-6) ) {
        vec[0] = vec[0]/len;
        vec[1] = vec[1]/len;
        vec[2] = vec[2]/len;
        len = 1.0;
    } else {
        ci::app::console()<<"Null length vector - Error.";
    }
}

void print(const Vector3D &v)
{
    std::cout<<"("<<v[0]<<","<<v[1]<<","<<v[2]<<")";
}
