#include "ClearpathDemoTools.h"

#include "Vector3.h"
#include "bestfit.h"
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <ctime>

/******************************************************************

                            STATIC HELPERS

*******************************************************************/

// Uses 3 random points to construct a plane (normal vector + offset)
bool ClearpathDemoTools::GetPlaneFromRnd3(PointCloud* cloud, double* normal_coeff)
{
    int i1, i2, i3, num;
    i1 = i2 = i3 = 0;
    num = cloud->points.size();
    
    // Atleast 3 points must exist
    if (num > 3) 
    {
        Vector3 a1, a2, a3;
        
        // Get a random number
        i1 = i2 = i3 = rand() % num;
        a1.x = cloud->points[i1].x; a1.y = cloud->points[i1].y; a1.z = cloud->points[i1].z;

        // While points are the same or too close together, find a new second point
        while (i2 == i1 || (a1-a2).Length() < RANSAC_PLANE_SEPDIST) { 
            i2 = rand() % num; 
            a2.x = cloud->points[i2].x; a2.y = cloud->points[i2].y; a2.z = cloud->points[i2].z;
            //TODO timeout incase points are all close? 
        }

        // While points are the same or too close together, find a new third point
        while ( i3 == i1 || i3 == i2 || (a3-a1).Length() < RANSAC_PLANE_SEPDIST || (a3-a2).Length() < RANSAC_PLANE_SEPDIST) { 
            i3 = rand() % num;
            a3.x = cloud->points[i3].x; a3.y = cloud->points[i3].y; a3.z = cloud->points[i3].z;
            //TODO timeout incase points are all close? 
        }

        // Get Normal Vector
        Vector3 ax, ay, az;
        ax = a2 - a1; ax.Normalize();
        ay = (a3 - a1) - ax*(ax.Dot(a3-a1)); ay.Normalize();
        az = ax.Cross(ay);
        
        // Flip to make it positive
        if (az.y < 0.0)
            az = az*-1.0;

        // Find the offset of the plane
        double mag = az.Dot(a1);

        // Fill Result
        normal_coeff[0] = az.x;
        normal_coeff[1] = az.y;
        normal_coeff[2] = az.z;
        normal_coeff[3] = mag;

    } else {
        return false;
    }
    return true;
}

// Find the random sample consensus plane
// normal_coeff + match_coeff offer the option to reject planes that are too different from an initial guess
bool ClearpathDemoTools::PlaneRANSAC(PointCloud* cloud, double* normal_coeff, bool match_coeff)
{
    // Make random
    srand ( time(NULL) );

    // Variables
    int iter_count = 0;
    int bestcount = 0;    
    double bestnorm[4];

    // For X Iterations
    while (iter_count < RANSAC_MAX_ITER)
    {   
        iter_count++;
        int count = 0;        
        
        double norm[4];
        
        // Get a plane from a random 3 points
        if (!GetPlaneFromRnd3(cloud, &norm[0]))
            return false;
        
        // Check to see how many pairs agree with the randomly selected transformation
        Vector3 n;
        n.x = norm[0]; n.y = norm[1]; n.z = norm[2];
        for (unsigned int i = 0; i < cloud->points.size(); i++)
        {   
            // Get Point as Vector3
            Vector3 a1;
            a1.x = cloud->points[i].x; a1.y = cloud->points[i].y; a1.z = cloud->points[i].z;
            
            // Get Distance from point to plane
            double mag = n.Dot(a1);
            
            // Check if distance is less than threshold
            if ( fabs(mag-norm[3]) < RANSAC_PLANE_THRESH )
                count++;
        }
        
        // If the current consensus is better than the best, switch the new one in
        if ( count > bestcount )
        {
            if ( !match_coeff || (fabs(norm[0]-normal_coeff[0]) < RANSAC_FLOOR_ANGTHRESH &&
                                  fabs(norm[1]-normal_coeff[1]) < RANSAC_FLOOR_ANGTHRESH &&
                                  fabs(norm[2]-normal_coeff[2]) < RANSAC_FLOOR_ANGTHRESH &&
                                  fabs(norm[3]-normal_coeff[3]) < RANSAC_FLOOR_MAGTHRESH ) ) {
                bestcount = count;
                for (unsigned int r = 0; r < 4; r++) { bestnorm[r] = norm[r]; }
            }
        }
    }

    // Check that the best consensus has atleast RANSAC_PLANE_MINNUM agreements
    if (bestcount < RANSAC_PLANE_MINNUM)
        return false;

    for (unsigned int r = 0; r < 4; r++) { normal_coeff[r] = bestnorm[r]; }

    return true;
}

// Perform Least Squares over the inliers of a plane
bool ClearpathDemoTools::PlaneLS(PointCloud* cloud, double* normal_coeff)
{
    std::vector<Point> points;

    // Collect all inliers around a ransac'd plane
    Vector3 n;
    n.x = normal_coeff[0]; n.y = normal_coeff[1]; n.z = normal_coeff[2];
    for (unsigned int i = 0; i < cloud->points.size(); i++)
    {   
        // Get point as Vector3
        Vector3 a1;
        a1.x = cloud->points[i].x; a1.y = cloud->points[i].y; a1.z = cloud->points[i].z;
        
        // Get Distance to plane
        double mag = n.Dot(a1);
        
        // Check if distance is less than threshold
        if ( fabs(mag-normal_coeff[3]) < RANSAC_PLANE_THRESH )
            points.push_back(cloud->points[i]);
    }

    // Link to BestFit Library to perform LeastSquares
    float normf[4];
    getBestFitPlane(points.size(),     // number of input data points
					&points[0].x,      // starting address of points array.
					sizeof(points[0]), // stride between input points.
					0,                 // optional point weighting values.
					0,                 // weight stride for each vertex.
					&normf[0]);

    // Make sure normal is facing +y
    if (normf[1] < 0.0) {
        for (unsigned int r = 0; r < 3; r++) { normal_coeff[r] = -normf[r]; }
        normal_coeff[3] = normf[3];
    } else {
        for (unsigned int r = 0; r < 4; r++) { normal_coeff[r] = normf[r]; }
        normal_coeff[3] = -normf[3];
    }
}

void ClearpathDemoTools::PlaneSegment(PointCloud* cloud, PointCloud* plane, PointCloud* seg_cloud, double* norm, double thresh)
{
    plane->points.clear();
    seg_cloud->points.clear();

    Vector3 n;
    n.x = norm[0]; n.y = norm[1]; n.z = norm[2];
    for (unsigned int i = 0; i < cloud->points.size(); i++)
    {   
        Vector3 a1;
        a1.x = cloud->points[i].x; a1.y = cloud->points[i].y; a1.z = cloud->points[i].z;
        
        double mag = n.Dot(a1);
        
        // Check if distance is less than threshold
        if ( fabs(mag-norm[3]) < thresh)
            plane->points.push_back(cloud->points[i]);
        else
            seg_cloud->points.push_back(cloud->points[i]);
    }
}

bool ClearpathDemoTools::TransformByNormal(PointCloud* cloud, PointCloud* cloud_out, double* normal_coeff)
{
    cloud_out->points.clear();

    std::vector<Point> points;

    // Check to see how many pairs agree with the randomly selected transformation
    Vector3 n, f, r;
    n.x = normal_coeff[0]; n.y = normal_coeff[1]; n.z = normal_coeff[2];
    f.x = 0.0f; f.y = 0.0f; f.z = 1.0f;
    r = n.Cross(f);
    f = r.Cross(n);

    for (unsigned int i = 0; i < cloud->points.size(); i++)
    {   
        Vector3 a1;
        a1.x = cloud->points[i].x; a1.y = cloud->points[i].y; a1.z = cloud->points[i].z;
        
        Point temp = cloud->points[i];
        temp.x = r.Dot(a1);
        temp.y = n.Dot(a1) - normal_coeff[3];
        temp.z = f.Dot(a1);
        
        cloud_out->points.push_back(temp);
    }
}

// Take a point cloud and return a vertical band of it (y direction)
void ClearpathDemoTools::VerticalCut(PointCloud* cloud, PointCloud* cloud_out, double min, double max, int skim_factor)
{
    cloud_out->points.clear();
    for (int i = 0; i < cloud->points.size(); i=i+skim_factor)
    {
        if (cloud->points[i].y > min && cloud->points[i].y < max)
        {
            cloud_out->points.push_back(cloud->points[i]);
        }
    }
}

// Given a position, calculate the velocity command such that it is along a circle with a common ICR
Twist ClearpathDemoTools::DetermineVelocity(double x, double z, double lin_speed)
{
    Twist tw;
    tw.linear = lin_speed;
    tw.angular = 0.0;

    // If X component is essentiall zero, then just go straight
    if (fabs(x) < 1e-4)
        return tw;

    // Calculate Turning Radius
    double r = fabs((x*x + z*z)/(2*x));

    // Calculate Turning Vel
    tw.angular = tw.linear/r; // a = r0, v = rw
    if (x > 0.0)
        tw.angular = -tw.angular;

    return tw;
}

/******************************************************************

                                 DEMO CODE

*******************************************************************/

ClearpathDemoTools::ClearpathDemoTools()
{
    mode = MODE_NONE;
}

ClearpathDemoTools::~ClearpathDemoTools()
{
    delete [] init_map;
    delete [] mapc;    
}

// General Initialization
void ClearpathDemoTools::Init(double _lin_speed,
                              double _ang_speed,
                              double _cam_height,
                              double _cam_z_trans,
                              unsigned int _numIncX,
                              unsigned int _numIncZ)
{
    lin_speed = _lin_speed;
    ang_speed = _ang_speed;
    cam_height = _cam_height;
    cam_z_trans = _cam_z_trans;

    minAng = -30.0*3.141592/180.0;
    maxAng = 30.0*3.141592/180.0;
    drawInc = 4;
    obstacleThreshold = 10;

    numIncX = _numIncX;
    numIncZ = _numIncZ;
    init_map = new Square[numIncX*numIncZ];
    mapc = new Square[numIncX*numIncZ];

    cloudCopy.points.clear();
    cloudCut.points.clear();
    seg.points.clear();
    plane.points.clear();
    final.points.clear();

    base_normal[0] = 0.0;
    base_normal[1] = 1.0;
    base_normal[2] = 0.0;
    base_normal[3] = cam_height;
    last_normal[0] = base_normal[0];
    last_normal[1] = base_normal[1];
    last_normal[2] = base_normal[2];
    last_normal[3] = base_normal[3];

    normal_initialized = false;
}

// Initialize as the Explore Demo
void ClearpathDemoTools::InitExplore(double _lin_speed,
                                     double _ang_speed,
                                     double _cam_height,
                                     double _cam_z_trans,
                                     double _wall_buffer,
                                     int _random_walk)
{
    mode = MODE_EXPLORE;
    Init(_lin_speed, _ang_speed, _cam_height, _cam_z_trans, NUM_INC_ANG, NUM_INC_RNG);

    minX = -30.0*3.141592/180.0;
    maxX = 30.0*3.141592/180.0;
    minZ = 0.5;
    maxZ = 4.0;
    incX = (maxX-minX)/double(numIncX);
    incZ = (maxZ-minZ)/double(numIncZ);

    random_walk = _random_walk;
    wall_buffer = _wall_buffer;
    old_clearpath = true;
    old_dir = 1.0;
}

// Initialize as the Person Tracking Demo
void ClearpathDemoTools::InitTrack(double _lin_speed,
                                   double _ang_speed,
                                   double _cam_height,
                                   double _cam_z_trans,
                                   double _window_size)
{
    mode = MODE_TRACK;
    Init(_lin_speed, _ang_speed, _cam_height, _cam_z_trans, NUM_INC_X, NUM_INC_Z);

    minX = -2.0;
    maxX = 2.0;
    minZ = 0.0;
    maxZ = 4.0;
    incX = (maxX-minX)/double(numIncX);
    incZ = (maxZ-minZ)/double(numIncZ);

    targetX = 0.0;
    targetZ = 2.0;
    targetW = _window_size;
}

// Calculate which bin a coordinate lies in
void ClearpathDemoTools::getBins(double x, double z, unsigned int* binX, unsigned int* binZ)
{
    *binX = std::min(std::max((int)((x-minX)/incX), 0), int(numIncX)-1);
    *binZ = std::min(std::max((int)((z-minZ)/incZ), 0), int(numIncZ)-1);
}

// Update Interface for Windows (non-ROS) Based Demo
void ClearpathDemoTools::NIUpdate(float* px, float* py, float* pz, unsigned int len, double* return_lin_vel, double* return_ang_vel)
{
    PointCloud c, f;
    for (int i = 0; i < len; i++)
    {
        Point p;  p.x = px[i]; p.y = py[i]; p.z = pz[i]; p.rgb = 0.0f;
        c.points.push_back(p);
    }
    Twist tw = this->Update(&c, &f);
    *return_lin_vel = tw.linear;
    *return_ang_vel = tw.angular;
}

// Main Update Call
Twist ClearpathDemoTools::Update(PointCloud* cloud, PointCloud* result)
{
    // Clear all point clouds
    cloudCopy.points.clear();
    cloudCut.points.clear();
    seg.points.clear();
    plane.points.clear();
    final.points.clear();

    // Copy Cloud
    cloudCopy.points = cloud->points;

    // Initialize Once
    if (!normal_initialized)
    {
        // Setup Initial Map
        for (int i = 0; i < numIncX; i++)
        {
            for (int j = 0; j < numIncZ; j++)
            {
                if (mode == MODE_EXPLORE) {
                    init_map[i*numIncZ + j].a = minAng + incX*i + incX*0.5;
                    init_map[i*numIncZ + j].r = minZ + incZ*j + incZ*0.5;
                    init_map[i*numIncZ + j].x = init_map[i*numIncZ + j].r*sin(init_map[i*numIncZ + j].a);
                    init_map[i*numIncZ + j].z = init_map[i*numIncZ + j].r*cos(init_map[i*numIncZ + j].a);
                    init_map[i*numIncZ + j].c = 0;
                } else if (mode == MODE_TRACK) {
                    init_map[i*numIncZ + j].a = 0.0;
                    init_map[i*numIncZ + j].r = 0.0;
                    init_map[i*numIncZ + j].x = minX + incX*i + incX*0.5;
                    init_map[i*numIncZ + j].z = minZ + incZ*j + incZ*0.5;
                    init_map[i*numIncZ + j].c = 0;
                }
            }
        }

        // Guess at Plane Points
        VerticalCut(&cloudCopy, &cloudCut, base_normal[3]-0.1, base_normal[3]+0.1, 13);

        // ID Plane
        double norm[4];
        norm[0] = base_normal[0]; norm[1] = base_normal[1]; norm[2] = base_normal[2]; norm[3] = base_normal[3];
        if ( ClearpathDemoTools::PlaneRANSAC(&cloudCut, &norm[0], true) ) {
            normal_initialized = true;
        } else {
            printf("BAD INITIALIZATION, NORMAL FAILED, TRY AGAIN \n");
            Twist tw;
            return tw;
        }

        // Refine Plane
        ClearpathDemoTools::PlaneLS(&cloudCut, &norm[0]);
        last_normal[0] = norm[0]; last_normal[1] = norm[1]; last_normal[2] = norm[2]; last_normal[3] = norm[3];
    }

    // Init Map from stored initial map
    memcpy(mapc, init_map, numIncX*numIncZ*sizeof(Square));

    // Segment out floor points
    ClearpathDemoTools::PlaneSegment(&cloudCopy, &plane, &seg, &last_normal[0], 0.15);

    // Transform points to be in 2D coord frame
    ClearpathDemoTools::TransformByNormal(&seg, &final, &last_normal[0]);

    // Transform poitns to be at robot center
    for (int i = 0; i < final.points.size(); i++)
    {    
        final.points[i].z = final.points[i].z - cam_z_trans;
    }

    // Fill Map (Tally how many points are in each bin)
    for (int i = 0; i < final.points.size(); i++)
    {
        double x = final.points[i].x;
        double z = final.points[i].z;

        if (mode == MODE_EXPLORE) {
            // Explore mode uses a radially based grid
            double angle = atan2(x, z);
            double range = sqrt(x*x + z*z);
            if (angle < minAng || angle > maxAng || range < minZ || range > maxZ)
                continue;
            unsigned int binX;
            unsigned int binZ;
            getBins(angle, range, &binX, &binZ);
            unsigned int index = binX*numIncX + binZ;
            if (index >= 0 && index < numIncX*numIncZ)
                mapc[binX*numIncZ + binZ].c++;

        } else if (mode == MODE_TRACK) {
            // Track mode uses a regular rectilinear grid
            unsigned int binX;
            unsigned int binZ;
            getBins(x, z, &binX, &binZ);
            unsigned int index = binX*numIncX + binZ;
            if (index >= 0 && index < numIncX*numIncZ)
                mapc[binX*numIncZ + binZ].c++;

        }
    }

    // If in exploring mode
    if (mode == MODE_EXPLORE) {

        // Add a virtual buffer to all the object edges by adding a object count to all squares within X meters of an occupied square
        // TODO Make More Efficient
        for (int i = 0; i < numIncX; i++)
        {
            for (int j = 0; j < numIncZ; j++)
            {
                bool colored = false;
                for (int a = 0; a < numIncX; a++)
                {
                    for (int b = 0; b < numIncZ; b++)
                    {
                        if ( mapc[i*numIncZ+j].c == 0 && mapc[a*numIncZ+b].c > 1) {
                            double xi = mapc[i*numIncZ+j].x;
                            double zi = mapc[i*numIncZ+j].z;
                            double xa = mapc[a*numIncZ+b].x;
                            double za = mapc[a*numIncZ+b].z;
                            double d = sqrt( (xi-xa)*(xi-xa) + (zi-za)*(zi-za) );
                            if (d < wall_buffer) {
                                colored = true;
                                mapc[i*numIncZ+j].c = 1;
                            }
                        }
                        if (colored)
                            break;
                    }
                    if (colored)
                        break;
                }
            }
        }

        // We can't see behind any square with an obstacle in it, so we project the obstacle backwards
        for (int i = 0; i < numIncX; i++)
        {
            bool killSlice = false;
            for (int j = 0; j < numIncZ; j++)
            {
                if (killSlice == true)
                    mapc[i*numIncZ+j].c = 1;
                else if (mapc[i*numIncZ+j].c > 0) {
                    killSlice = true;
                }
            }
        }

        // Check to see if there is an obstacle in the first row of squares (we should turn in place to avoid this)
        bool clearpath = true;
        int dir = 1.0;
        if (!old_clearpath)
            dir = old_dir;
        for (unsigned int i = 0; i < numIncX; i++)
        {
            if ( mapc[i*numIncZ].c > 0 || mapc[i*numIncZ+1].c > 0)
            {
                clearpath = false;
                if ( old_clearpath && double(i) < double(numIncX)/2.0 )
                    dir = -1.0;
            }
        }

        // Update Info about turn (in place) direction
        old_clearpath = clearpath;
        old_dir = dir;

        // Search for the best square to head towards
        double bestxi[NUM_INC_ANG] = {0.0};
        double bestzi[NUM_INC_ANG] = {0.0};
        double bestd[NUM_INC_ANG] = {0.0};
        unsigned int besti[NUM_INC_ANG] = {0};
        unsigned int bestOverall = 0;
        for (unsigned int i = 0; i < numIncX; i++)
        {
            for (unsigned int j = 0; j < numIncZ; j++)
            {
                if ( mapc[i*numIncZ+j].c == 0)
                {
                    double d = mapc[i*numIncZ+j].z ;//sqrt( xi*xi + zi*zi );
                    if (d > bestd[bestOverall]) {
                        bestOverall = i;
                    }
                    if (d > bestd[i]) {
                        bestxi[i] = mapc[i*numIncZ+j].x;
                        bestzi[i] = mapc[i*numIncZ+j].z;
                        bestd[i] = d;
                        besti[i] = j;
                    }
                }
            }
        }

        if (random_walk > 0)
        {
            srand ( time(NULL) );
            bestOverall = rand() % NUM_INC_ANG;
        }

        // Draw Map (Visualization)
        for (int i = 0; i < numIncX; i++)
        {
            for (int j = 0; j < numIncZ; j++)
            {
                for (int a = 0; a < drawInc; a++)
                {
                    for (int b = 0; b < drawInc; b++)
                    {
                        Point temp;
                        double angle = minAng + incX*i + incX*(double(a)/double(drawInc));
                        double range = minZ + incZ*j + incZ*(double(b)/double(drawInc));
                        temp.x = range*sin(angle);
                        temp.y = 0.0;
                        temp.z = range*cos(angle);                    
                        char* rgbc = (char*)(&temp.rgb);
                        if (mapc[i*numIncZ+j].c == 0) {
                            rgbc[0] = 0; rgbc[1] = 255; rgbc[2] = 0;
                        } else {
                            rgbc[0] = 0; rgbc[1] = 0; rgbc[2] = 255;
                        }
                        final.points.push_back(temp);
                    }
                }
            }
        }

        // Fill result
        result->points = final.points;

        if (!clearpath) {
            // Turn in place
            Twist tw;
            tw.linear = 0.0;
            tw.angular = ang_speed*dir;
            return tw;
        } else if (bestd[bestOverall] > 0.0 && fabs(bestxi[bestOverall]) < 1e-4) {
            // Go Straight Forward
            Twist tw;
            tw.linear = lin_speed;
            tw.angular = 0.0;
            return tw;
        } else if (bestd[bestOverall] > 0.0) {
            // Found a target to turn towards
            Twist *tw = new Twist[numIncX];
            for (unsigned int i = 0; i < numIncX; i++)
            {
                tw[i] = ClearpathDemoTools::DetermineVelocity(bestxi[i], bestzi[i], lin_speed);
            }
            Twist besttw;
            besttw.linear = 0.0;
            besttw.angular = 0.0;
            for (unsigned int i = 0; i < numIncX; i++)
            {
                if (copysign(1.0,tw[bestOverall].angular) == copysign(1.0,tw[i].angular))
                    if ( fabs(tw[i].angular) > fabs(besttw.angular) )
                        besttw = tw[i];
            }
            delete [] tw;
            return besttw;
        } else {
            // Uh-oh? Don't go anywhere.
            Twist tw;
            tw.linear = 0.0;
            tw.angular = 0.0;
            return tw;
        }

    } else if (mode == MODE_TRACK) {
    
        // Find the target square with highest density and head there
        // Experimental results show this works pretty well for following people, although there are probably more elegent solutions
        double bestx = 0.0;
        double bestz = 0.0;
        unsigned int bestc = 0;
        double bestd = 0.0;
        unsigned int bestxbin = 0;
        unsigned int bestzbin = 0;
        for (unsigned int i = 0; i < numIncX; i++)
        {
            for (unsigned int j = 0; j < numIncZ; j++)
            {
                if (mapc[i*numIncZ+j].c > bestc)
                {
                    double x = minX + incX*i + incX*0.5;
                    double z = minZ + incZ*j + incZ*0.5;
                    double d = sqrt((x-targetX)*(x-targetX) + (z-targetZ)*(z-targetZ));
                    if (d < targetW) {
                        bestc = mapc[i*numIncZ+j].c;
                        bestx = x;
                        bestz = z;
                        bestxbin = i;
                        bestzbin = j;
                    }
                }
            }
        }

        // Set New Target
        if (bestc > 0) {
            targetX = bestx;
            targetZ = bestz;
            bestd = sqrt(targetX*targetX + targetZ*targetZ);
        }

        // Draw Map (Visualization)
        for (int i = 0; i < numIncX; i++)
        {
            for (int j = 0; j < numIncZ; j++)
            {
                for (int a = 0; a < drawInc; a++)
                {
                    for (int b = 0; b < drawInc; b++)
                    {
                        Point temp;
                        temp.x = minX + incX*i + incX*(double(a)/double(drawInc));
                        temp.y = 0.0;
                        temp.z = minZ + incZ*j + incZ*(double(b)/double(drawInc));           
                        char* rgbc = (char*)(&temp.rgb);
                        if (bestxbin == i && bestzbin == j) {
                            rgbc[0] = 255; rgbc[1] = 0; rgbc[2] = 0;
                        } else if (mapc[i*numIncZ+j].c == 0) {
                            rgbc[0] = 0; rgbc[1] = 255; rgbc[2] = 0;
                        } else {
                            rgbc[0] = 0; rgbc[1] = 0; rgbc[2] = 255;
                        }
                        final.points.push_back(temp);
                    }
                }
            }
        }

        // Fill result
        result->points = final.points;

        if (bestc > 0) {
            // Found a target, move towards it
            Twist tw;
            if (bestd > 1.0) {
                // Generate twist to move along an arc towards target
                tw = ClearpathDemoTools::DetermineVelocity(targetX, targetZ, lin_speed);
            } else {
                // We are closer than 1 meter, simply turn to face target
                tw.linear = 0.0;
                double tempspd = ang_speed*std::min(fabs(bestx/0.35), 1.0);
                if (fabs(bestx) < 0.1)
                    tempspd = 0.0;
                tw.angular = -1.0*copysign(tempspd, bestx);
            }
            return tw;
        } else {
            // Uh-oh, no target, don't go anywhere
            Twist tw;
            tw.linear = 0.0;
            tw.angular = 0.0;
            return tw;       
        }
    }

}


