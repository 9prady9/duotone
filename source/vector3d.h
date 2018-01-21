#pragma once

#include <cmath>

class Vector3D
{
    public:
        Vector3D() {}
        Vector3D(float x,float y,float z);
        Vector3D(const Vector3D &v) {
            vec[0] = v.DX();
            vec[1] = v.DY();
            vec[2] = v.DZ();
            len	   = sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
        }

        void set(float x,float y,float z);
        void setDX(float _x) { vec[0]=(fabs(_x)<1.0e-3?0.0f:_x); }
        void setDY(float _y) { vec[1]=(fabs(_y)<1.0e-3?0.0f:_y); }
        void setDZ(float _z) { vec[2]=(fabs(_z)<1.0e-3?0.0f:_z); }
        float DX() const { return vec[0]; }
        float DY() const { return vec[1]; }
        float DZ() const { return vec[2]; }

        float length() const;
        float squaredLength() const;
        void normalize();
        float minComponent() const;
        float maxComponent() const;
        float minAbsComponent() const;
        float maxAbsComponent() const;
        int indexOfMinComponent() const;
        int indexOfMinAbsComponent() const;
        int indexOfMaxComponent() const;
        int indexOfMaxAbsComponent() const;
        /* Overloaded operators */
        const Vector3D& operator+() const;
        Vector3D operator-() const;

        float operator[](int i) const { return vec[i]; }
        float& operator[](int i) { return vec[i]; }

        friend bool operator==(const Vector3D &v1, const Vector3D &v2);
        friend bool operator!=(const Vector3D &v1, const Vector3D &v2);
        friend Vector3D operator+(const Vector3D &v1, const Vector3D &v2);
        friend Vector3D operator-(const Vector3D &v1, const Vector3D &v2);
        friend Vector3D operator/(const Vector3D &v, float t);
        friend Vector3D operator*(const Vector3D &v, float t);
        friend Vector3D operator*(float t, const Vector3D &v);

        Vector3D& operator=(const Vector3D& v);
        Vector3D& operator+=(const Vector3D& v);
        Vector3D& operator-=(const Vector3D& v);
        Vector3D& operator*=(const float t);
        Vector3D& operator/=(const float t);
        /* other methods */
        friend Vector3D unitVector(const Vector3D& v);
        friend Vector3D minVec(const Vector3D &v1, const Vector3D &v2);
        friend Vector3D maxVec(const Vector3D &v1, const Vector3D &v2);
        friend Vector3D cross(const Vector3D &v1, const Vector3D &v2);
        friend float dot(const Vector3D &v1, const Vector3D &v2);
        friend float tripleProduct(const Vector3D &v1, const Vector3D &v2, const Vector3D &v3);

        /* Attributes */
        float vec[3];
        float len;
};


/* ------------- Inline Methods of Vector3D class ------------- */

inline Vector3D::Vector3D(float x,float y,float z)
{
    vec[0] = fabs(x)<1.0e-6 ? 0.0f : x;
    vec[1] = fabs(y)<1.0e-6 ? 0.0f : y;
    vec[2] = fabs(z)<1.0e-6 ? 0.0f : z;
    if(vec[0]==0.0f && vec[1]==0.0f && vec[2]==0.0f)
        len = 0.0f;
    else
        len=sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
}

inline void Vector3D::set(float x,float y,float z)
{
    vec[0] = fabs(x)<1.0e-6 ? 0.0f : x;
    vec[1] = fabs(y)<1.0e-6 ? 0.0f : y;
    vec[2] = fabs(z)<1.0e-6 ? 0.0f : z;
    if(vec[0]==0.0f && vec[1]==0.0f && vec[2]==0.0f)
        len = 0.0f;
    else
        len=sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
}

inline float Vector3D::length() const { return len; }

inline float Vector3D::squaredLength() const { return len*len; }

inline float Vector3D::minComponent() const
{
    float temp = vec[0];
    if( vec[1] < temp ) temp = vec[1];
    if( vec[2] < temp ) temp = vec[2];
    return temp;
}

inline float Vector3D::maxComponent() const
{
    float temp = vec[0];
    if( vec[1] > temp ) temp = vec[1];
    if( vec[2] > temp ) temp = vec[2];
    return temp;
}

inline float Vector3D::minAbsComponent() const
{
    float temp = fabs(vec[0]);
    if( fabs(vec[1]) < temp ) temp = fabs(vec[1]);
    if( fabs(vec[2]) < temp ) temp = fabs(vec[2]);
    return temp;
}

inline float Vector3D::maxAbsComponent() const
{
    float temp = fabs(vec[0]);
    if( fabs(vec[1]) > temp ) temp = fabs(vec[1]);
    if( fabs(vec[2]) > temp ) temp = fabs(vec[2]);
    return temp;
}

inline int Vector3D::indexOfMinComponent() const
{
    int index = 0;
    float temp = vec[0];
    if( vec[1] < temp ) { temp = vec[1]; index = 1; }
    if( vec[2] < temp ) index = 2;
    return index;
}

inline int Vector3D::indexOfMinAbsComponent() const
{
    int index = 0;
    float temp = fabs(vec[0]);
    if( fabs(vec[1]) < temp ) { temp = fabs(vec[1]); index = 1; }
    if( fabs(vec[2]) < temp ) index = 2;
    return index;
}

inline int Vector3D::indexOfMaxComponent() const
{
    int index = 0;
    float temp = vec[0];
    if( vec[1] > temp ) { temp = vec[1]; index = 1; }
    if( vec[2] > temp ) index = 2;
    return index;
}

inline int Vector3D::indexOfMaxAbsComponent() const
{
    int index = 0;
    float temp = fabs(vec[0]);
    if( fabs(vec[1]) > temp ) { temp = fabs(vec[1]); index = 1; }
    if( fabs(vec[2]) > temp ) index = 2;
    return index;
}

inline const Vector3D& Vector3D::operator +() const { return *this; }

inline Vector3D Vector3D::operator -() const { return Vector3D(-vec[0],-vec[1],-vec[2]); }

inline bool operator==(const Vector3D &v1, const Vector3D &v2)
{
    if( fabs(v1.vec[0]-v2.vec[0]) < 1.0e-3 ) return false;
    if( fabs(v1.vec[1]-v2.vec[1]) < 1.0e-3 ) return false;
    if( fabs(v1.vec[2]-v2.vec[2]) < 1.0e-3 ) return false;
    return true;
}

inline bool operator!=(const Vector3D &v1, const Vector3D &v2) { return !(v1==v2); }

inline Vector3D operator+(const Vector3D &v1, const Vector3D &v2)
{
    return Vector3D(v1.vec[0]+v2.vec[0], v1.vec[1]+v2.vec[1], v1.vec[2]+v2.vec[2]);
}

inline Vector3D operator-(const Vector3D &v1, const Vector3D &v2)
{
    return Vector3D(v1.vec[0]-v2.vec[0], v1.vec[1]-v2.vec[1], v1.vec[2]-v2.vec[2]);
}

inline Vector3D operator/(const Vector3D &v, float t)
{
    return Vector3D(v.vec[0]/t, v.vec[1]/t, v.vec[2]/t);
}

inline Vector3D operator*(const Vector3D &v, float t)
{
    return Vector3D(v.vec[0]*t, v.vec[1]*t, v.vec[2]*t);
}

inline Vector3D operator*(float t, const Vector3D &v)
{
    return Vector3D(v.vec[0]*t, v.vec[1]*t, v.vec[2]*t);
}

inline Vector3D& Vector3D::operator=(const Vector3D& v)
{
    if( this != &v )
    {
        vec[0] = fabs(v.DX())<1.0e-6 ? 0.0f : v.DX();
        vec[1] = fabs(v.DY())<1.0e-6 ? 0.0f : v.DY();
        vec[2] = fabs(v.DZ())<1.0e-6 ? 0.0f : v.DZ();
        len = sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
    }
    return *this;
}

inline Vector3D& Vector3D::operator +=(const Vector3D& v)
{
    *this = *this + v;
    return *this;
}

inline Vector3D& Vector3D::operator -=(const Vector3D& v)
{
    *this = *this - v;
    return *this;
}

inline Vector3D& Vector3D::operator *=(const float t)
{
    *this = *this * t;
    return *this;
}

inline Vector3D& Vector3D::operator /=(const float t)
{
    *this = *this / t;
    return *this;
}

inline Vector3D unitVector(const Vector3D& v)
{
    float l = v.length();
    return v/l;
}

inline Vector3D minVec(const Vector3D &v1, const Vector3D &v2)
{
    Vector3D v(v1);
    if( v2.DX() < v1.DX() ) v.setDX(v2.DX());
    if( v2.DY() < v1.DY() ) v.setDY(v2.DY());
    if( v2.DZ() < v1.DZ() ) v.setDZ(v2.DZ());
    return v;
}

inline Vector3D maxVec(const Vector3D &v1, const Vector3D &v2)
{
    Vector3D v(v1);
    if( v2.DX() > v1.DX() ) v.setDX(v2.DX());
    if( v2.DY() > v1.DY() ) v.setDY(v2.DY());
    if( v2.DZ() > v1.DZ() ) v.setDZ(v2.DZ());
    return v;
}

inline Vector3D cross(const Vector3D &v1, const Vector3D &v2)
{
    Vector3D temp;

    temp.vec[0] = v1.DY()*v2.DZ() - v1.DZ()*v2.DY();
    temp.vec[1] = v1.DZ()*v2.DX() - v1.DX()*v2.DZ();
    temp.vec[2] = v1.DX()*v2.DY() - v1.DY()*v2.DX();

    temp[0] = ( fabs(temp[0])<1.0e-6 ? 0.0f : temp[0]);
    temp[1] = ( fabs(temp[1])<1.0e-6 ? 0.0f : temp[1]);
    temp[2] = ( fabs(temp[2])<1.0e-6 ? 0.0f : temp[2]);

    temp.len = sqrt(temp[0]*temp[0]+temp[1]*temp[1]+temp[2]*temp[2]);
    return temp;
}

inline float dot(const Vector3D &v1, const Vector3D &v2)
{
    return (v1.DX()*v2.DX() + v1.DY()*v2.DY() + v1.DZ()*v2.DZ());
}

void print(const Vector3D &v);
