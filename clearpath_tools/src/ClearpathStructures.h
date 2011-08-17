#ifndef H_CPSTRUCTS
#define H_CPSTRUCTS

#define PCLOUD_FULL 1
#define PCLOUD_SKIM 2
#define PCLOUD_NORM 3

// 6D Transformation
typedef struct CPTransform {
    float rotation[9];
    float translation[3];
};

// Generally for image coordinates
typedef struct CPPoint2D {
    float u;
    float v;
};

// 3D Position
typedef struct CPPoint3D {
    float x;
    float y;
    float z;
};

// "Feature" usually found in an image, has a 2D and a 3D Position
typedef struct CPFeature {
    CPPoint2D point2D; // 2D coordinate found in original image
    CPPoint3D point3D; // 3D coordinate
};

#endif
