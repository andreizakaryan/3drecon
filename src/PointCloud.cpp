#include "PointCloud.h"

#include <fstream>

PointCloud::PointCloud()
{
    min = Vec3d(
                std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity());
    max = Vec3d(
                -std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity());
}

void PointCloud::merge(const PointCloud& other)
{
    mCenter *= mPoints.size();
    for (SharedPoint p : other.mPoints)
        addPoint(p, other.mColors.at(p).first);
    this->boundingBox();
}


void PointCloud::addPoint(SharedPoint p, RGB color)
{
    mCenter += *p;
    max.max(*p);
    min.min(*p);

    mPoints.push_back(p);
    mColors.append(p, std::make_pair(color, false));
}

void PointCloud::boundingBox()
{
    mCenter /= mPoints.size();
    mHalfDimension = (max - min) / 2;
}
