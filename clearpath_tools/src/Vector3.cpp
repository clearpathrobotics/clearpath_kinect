#include "Vector3.h"
#include "math.h"

Vector3::Vector3()
{ this->x = 0; this->y = 0; this->z = 0; }

Vector3::Vector3(float x, float y, float z)
{ this->x = x; this->y = y; this->z = z; }

Vector3::~Vector3()
{}

Vector3::Vector3(const Vector3 &vec) :
x(vec.x), y(vec.y), z(vec.z)
{}

Vector3 Vector3::operator+(const Vector3 &vec) const
{
    Vector3 newVec;
    newVec.x = this->x + vec.x;
    newVec.y = this->y + vec.y;
    newVec.z = this->z + vec.z;
    return newVec;
}

Vector3 Vector3::operator-(const Vector3 &vec) const
{
    Vector3 newVec;
    newVec.x = this->x - vec.x;
    newVec.y = this->y - vec.y;
    newVec.z = this->z - vec.z;
    return newVec;
}

Vector3 Vector3::operator*(const float k) const
{
    Vector3 newVec;
    newVec.x = this->x * k;
    newVec.y = this->y * k;
    newVec.z = this->z * k;
    return newVec;
}

void Vector3::Normalize()
{
    float length = Length();
    this->x /= length;
    this->y /= length;
    this->z /= length;
}

float Vector3::Dot(const Vector3 &vec) const
{
    return ( this->x * vec.x + this->y * vec.y + this->z * vec.z );
}

Vector3 Vector3::Cross(const Vector3 &vec) const
{
    Vector3 newVec;
    newVec.x = this->y * vec.z - vec.y * this->z;
    newVec.y = this->z * vec.x - vec.z * this->x;
    newVec.z = this->x * vec.y - vec.x * this->y;
    return newVec;
}

float Vector3::Length()
{
    return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
}

bool Vector3::IsEqual(const Vector3 &vec) const
{
    if (fabs(this->x - vec.x)<0.001f && fabs(this->y - vec.y)<0.001f && fabs(this->z - vec.z)<0.001f)
        return true;
    return false;
}
