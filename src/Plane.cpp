#include "Plane.h"
#include "PointCloud.h"

#include "Vec3.h"
#include <cmath>

std::ostream& operator<<(std::ostream& os, const Plane& p)
{
    return os << "{" << p.normal[0] << "x + " << p.normal[1] << "y + " << p.normal[2] << "z + " << p.d << " : " << p.count << " points, center = (" << p.center[0] << ", " << p.center[1] << ", " << p.center[2] << "), radius = " << p.radius << ", thickness = " << p.thickness << "}";
}


Plane::Plane() :
    m(3, 3, CV_64FC1)
{
    this->init();
    d = 0;
}

Plane::Plane(const std::vector<SharedPoint>& pts) :
    m(3, 3, CV_64FC1)
{
    this->setPoints(pts);
}


double Plane::distance(SharedPoint p)
{
    return std::abs((normal * *p) + d);
}

double Plane::squareDistance(SharedPoint p)
{
    double diff = (normal * *p) + d;
    return diff * diff;
}

bool Plane::accept(SharedPoint p)
{
    return (center.distance(*p) < 3 * radius) && (this->distance(p) < 2 * thickness);
}

void Plane::addPoint(SharedPoint p, UnionFindPlanes& colors)
{
    this->addPoint(*p);
    if (point)
        colors.merge(p, point);
    else
        point = p;
}

void Plane::setPoints(const std::vector<SharedPoint>& pts)
{
    if (!pts.empty())
        point = pts[0];

    this->init();
    for (auto&& p : pts)
        this->addPoint(*p);
    this->computeEquation();
}

void Plane::setColor(RGB color, UnionFindPlanes& colors)
{
    colors.set(point, std::make_pair(color, true));
}

void Plane::destroy(UnionFindPlanes& colors)
{
    colors.reset(point);
}

void Plane::merge(Plane& p, UnionFindPlanes& colors)
{
    for (auto pt: p.points()) {
        mPoints.push_back(pt);
    }
    count += p.count;
    m += p.m;
    sum += p.sum;
    this->computeEquation();
    colors.merge(point, p.point);
    p = Plane();
}

bool Plane::mergeableWith(const Plane& p, double dCos) const {
    if (!(count && p.count))
        return false;

    if (this->getCos(p) < dCos)
        return false;
    if (this->distanceAlong(center, p.center) > thickness && p.distanceAlong(center, p.center) > p.thickness)
        return false;

    Plane tempPlane;
    tempPlane.m = m + p.m;
    tempPlane.sum = sum + p.sum;
    tempPlane.count = count + p.count;
    tempPlane.computeEquation();

    if (this->getCos(tempPlane) < dCos || p.getCos(tempPlane) < dCos)
        return false;
    if (tempPlane.radius > radius + p.radius)
        return false;
    if (tempPlane.thickness > thickness + p.thickness)
        return false;
    return true;
}


void Plane::init()
{
    count = 0;

    for (unsigned int i = 0 ; i < 3 ; ++i)
        for (unsigned int j = 0 ; j < 3 ; ++j)
            m.at<double>(i, j) = 0;
    sum = Vec3d();

    center = Vec3d();
    radius = 0;
}

void Plane::addPoint(const Point& p)
{
    mPoints.push_back(std::make_shared<Point>(p));
    ++count;

    m.at<double>(0, 0) += p.x * p.x;
    m.at<double>(0, 1) += p.x * p.y;
    m.at<double>(0, 2) += p.x * p.z;

    m.at<double>(1, 0) += p.y * p.x;
    m.at<double>(1, 1) += p.y * p.y;
    m.at<double>(1, 2) += p.y * p.z;

    m.at<double>(2, 0) += p.z * p.x;
    m.at<double>(2, 1) += p.z * p.y;
    m.at<double>(2, 2) += p.z * p.z;

    sum += p;
}

void Plane::leastSquares()
{
    cv::Mat mat(3, 3, CV_64FC1);
    for (unsigned int i = 0 ; i < 3 ; ++i)
        for (unsigned int j = 0 ; j < 3 ; ++j)
            mat.at<double>(i, j) = m.at<double>(i, j) - sum[i] * sum[j] / count;

    cv::Mat eigenvals;
    cv::Mat eigenvects;
    cv::eigen(mat, eigenvals, eigenvects);

    normal[0] = eigenvects.at<double>(2, 0);
    normal[1] = eigenvects.at<double>(2, 1);
    normal[2] = eigenvects.at<double>(2, 2);
    normal.normalize();
    d = - (normal * sum) / count;
}

void Plane::computeEquation()
{
    this->leastSquares();

    center = sum / count;

    Vec3d stddev = (Vec3d(m.at<double>(0, 0), m.at<double>(1, 1), m.at<double>(2, 2)) / count)
            - center.cmul(center);
    radius = std::sqrt(stddev.x + stddev.y + stddev.z);

    double meansq = 0;
    for (unsigned int i = 0 ; i < 3 ; ++i)
        for (unsigned int j = 0 ; j < 3 ; ++j)
            meansq += m.at<double>(i, j) * normal[i] * normal[j];
    meansq /= count;

    double mean = d; // (normal * sum) / count;

    if (meansq - mean * mean < 0)
        thickness = 0;
    else
        thickness = std::sqrt(meansq - mean * mean);

    if (thickness < radius / 1000)
        thickness = radius / 1000;
}

double Plane::distanceAlong(Vec3d u, Vec3d v) const
{
    return std::abs((v - u) * normal);
}

double Plane::getCos(const Plane& p) const
{
    return std::abs(normal * p.normal);
}


void Plane::flatten()
{
    cv::Vec3d n = {normal.x, normal.y, normal.z};
    double den1 = std::sqrt(n[0]*n[0] + n[1]*n[1]);
    cv::Mat r1 = (cv::Mat1d(3, 3) << n[0]/den1,  n[1]/den1, 0,  
                                       -n[1]/den1, n[0]/den1, 0,
                                       0,         0,        1);
    double den2 = std::sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
    cv::Mat r2 = (cv::Mat1d(3, 3) << n[2]/den2, 0, -den1/den2,  
                                       0       , 1, 0, 
                                       den1/den2, 0, n[2]/den2);
    cv::Mat r = r2*r1;
    cv::Mat rInv = r.inv();
    for (auto p: mPoints) {
        cv::Mat pNew = r*(cv::Mat1d(3, 1) << p->x + d*n[0], p->y + d*n[1], p->z + d*n[2]);
        pNew = rInv*(cv::Mat1d(3, 1) << pNew.at<double>(0, 0), pNew.at<double>(1, 0), 0);
        p->x = pNew.at<double>(0, 0) - d*n[0];
        p->y = pNew.at<double>(1, 0) - d*n[1];
        p->z = pNew.at<double>(2, 0) - d*n[2];
    }

}
