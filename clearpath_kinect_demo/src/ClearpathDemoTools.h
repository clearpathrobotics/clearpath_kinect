#ifndef H_CPDEMOTOOLS
#define H_CPDEMOTOOLS

// Includes
#include <vector>

// Constants for Static Methods
#define RANSAC_MAX_ITER 150
#define RANSAC_PLANE_SEPDIST 0.1f
#define RANSAC_PLANE_MINNUM 30
#define RANSAC_PLANE_THRESH 0.01f
#define RANSAC_FLOOR_ANGTHRESH 0.1f
#define RANSAC_FLOOR_MAGTHRESH 0.1f

// Constants for Demo Object
#define MODE_NONE 0
#define MODE_TRACK 1
#define MODE_EXPLORE 2

#define NUM_INC_ANG 13
#define NUM_INC_RNG 20
#define NUM_INC_X 25
#define NUM_INC_Z 25

// Structure to hold values about a map square
struct Square
{
    public:
        double a; // angle (from camera)
        double r; // radial distance (from camera)
        double x; // dist in x (from camera)
        double z; // dist in z (from camera)
        unsigned int c; // number of points projected in 2D square
};

// Structure to hold velocities
struct Twist
{
    public:
        double linear; // lin vel
        double angular; // ang vel
};

// Structure to hold point
struct Point
{
    public:
        float x;
        float y;
        float z;
        float rgb;
};

// Structure to hold point cloud
struct PointCloud
{
    public:
        std::vector<Point> points;
};

class ClearpathDemoTools
{
	public:

		ClearpathDemoTools();
		virtual ~ClearpathDemoTools();

        //
        // Static Helper Functions
        //

        static bool PlaneRANSAC(PointCloud* cloud, double* normal_coeff, bool match_coeff);
        static bool PlaneLS(PointCloud* cloud, double* normal_coeff);
        static void PlaneSegment(PointCloud* cloud, PointCloud* plane, PointCloud* seg_cloud, double* norm, double thresh);
        static bool TransformByNormal(PointCloud* cloud, PointCloud* cloud_out, double* normal_coeff);
        static void VerticalCut(PointCloud* cloud, PointCloud* cloud_out, double min, double max, int skim_factor);
        static Twist DetermineVelocity(double x, double z, double speed);

        //
        // Demo Functions
        //

        void InitTrack(double _lin_speed,
                       double _ang_speed,
                       double _cam_height,
                       double _cam_z_trans,
                       double _window_size);
        void InitExplore(double _lin_speed,
                         double _ang_speed,
                         double _cam_height,
                         double _cam_z_trans,
                         double _wall_buffer,
                         int _random_walk);
        void NIUpdate(float* px, float* py, float* pz, unsigned int len, double* return_lin_vel, double* return_ang_vel);
        Twist Update(PointCloud* cloud, PointCloud* result);

        //
        // Demo Variables
        //

        int mode;

        unsigned int numIncX;
        unsigned int numIncZ;

        double lin_speed;
        double ang_speed;
        double cam_height;
        double cam_z_trans;

        Square* init_map;
        Square* mapc;

        double last_normal[4];
        double base_normal[4];
        bool normal_initialized;

        PointCloud cloudCopy;
        PointCloud cloudCut;
        PointCloud seg;
        PointCloud plane;
        PointCloud final;

        double minAng;
        double maxAng;
        double minX;
        double maxX;
        double minZ;
        double maxZ;
        double incX;
        double incZ;
        unsigned int drawInc;
        unsigned int obstacleThreshold;

        //
        // Track Demo Variables
        //

        double targetX;
        double targetZ;
        double targetW;

        //
        // Explore Demo Variables
        //

        int random_walk;
        double wall_buffer;
        bool old_clearpath;
        int old_dir;

	private:

        //
        // Static Helper Functions
        //

        static bool GetPlaneFromRnd3(PointCloud* cloud, double* normal_coeff);

        //
        // Demo Functions
        //

        void Init(double _lin_speed,
                  double _ang_speed,
                  double _cam_height,
                  double _cam_z_trans,
                  unsigned int _numIncX,
                  unsigned int _numIncZ);

        void getBins(double x, double z, unsigned int* binX, unsigned int* binZ);
};

#endif
