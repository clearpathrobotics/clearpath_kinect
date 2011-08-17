#ifndef H_VECTOR3
#define H_VECTOR3

class Vector3
{
public:
    Vector3();
    Vector3(float x, float y, float z);
    ~Vector3();
    Vector3(const Vector3 &vec);

    Vector3 operator+(const Vector3 &vec) const;
    Vector3 operator-(const Vector3 &vec) const;
    Vector3 operator*(const float k) const;

    void    Normalize();
    float   Dot(const Vector3 &vec) const;
    Vector3 Cross(const Vector3 &vec) const;
    float   Length();

    bool IsEqual(const Vector3 &vec) const;

    float x;
    float y;
    float z;
};

#endif
