#include "Octree.h"

#include "Ransac.h"
#include <algorithm>

Octree::Octree(const PointCloud& cloud, unsigned int maxdepth) :
    mRoot(cloud.center(), cloud.halfDimension())
{
    for (auto&& p : cloud.points())
        mRoot.insert(p, maxdepth);
}

void Octree::detectPlanes(int depthThreshold, double epsilon, int numStartPoints, int numPoints, int steps, double countRatio, std::default_random_engine& generator, std::vector<SharedPlane>& planes, UnionFindPlanes& colors, double dCos) const
{
    std::vector<SharedPoint> pts;
    mRoot.detectPlanes(depthThreshold, epsilon, numStartPoints, numPoints, steps, countRatio, generator, planes, colors, dCos, pts);
}

Octree::Node::Node(const Vec3d& center, const Vec3d& halfSize) :
    center(center), halfSize(halfSize), count(0)
{
}

void Octree::Node::getPoints(std::vector<SharedPoint>& pts) const
{
    if (isLeafNode())
    {
        if (point.get() != nullptr)
            pts.push_back(point);
    }
    else
        for (auto&& child : children)
            child->getPoints(pts);
}

void Octree::Node::removeSmallPlanes(std::vector<SharedPlane>& planes, double countRatio, UnionFind<SharedPoint, std::pair<RGB, bool>>& colors)
{
    if (!planes.empty())
    {
        static auto comp = [](const SharedPlane& a, const SharedPlane& b){return a && b ? a->getCount() > b->getCount() : (bool)a;};
        std::sort(planes.begin(), planes.end(), comp);

        if (planes[0])
        {
            double minCount = planes[0]->getCount() * countRatio;
            for (SharedPlane& p : planes)
            {
                if (p && p->getCount() <= minCount)
                {
                    p->destroy(colors);
                    p.reset();
                }
            }
        }
    }
}

void Octree::Node::detectPlanes(int depthThreshold, double epsilon, int numStartPoints, int numPoints, int steps, double countRatio, std::default_random_engine& generator, std::vector<SharedPlane>& planes, UnionFind<SharedPoint, std::pair<RGB, bool>>& colors, double dCos, std::vector<SharedPoint>& pts) const
{
    if (count > depthThreshold)
    {
        std::vector<SharedPlane> plns;
        for (auto&& child : children)
        {
            std::vector<SharedPoint> child_pts;
            if (child.get() != nullptr)
            {
                child->detectPlanes(depthThreshold, epsilon, numStartPoints, numPoints, steps, countRatio, generator, plns, colors, dCos, child_pts);
                for (auto&& p : child_pts)
                    pts.push_back(p);
            }
        }
        
        removeSmallPlanes(plns, countRatio, colors);

        for (unsigned int i = 0 ; i < plns.size() ; ++i)
        {
            for (unsigned int j = 0 ; j < i ; ++j)
            {
                if (plns[i] && plns[j] && plns[i]->mergeableWith(*plns[j], dCos))
                {
                    plns[i]->merge(*plns[j], colors);
                    plns[j].reset();
                }
            }
        }

        removeSmallPlanes(plns, countRatio, colors);

        for (auto&& p : pts)
        {
            if (!colors.at(p).second)
            {
                std::vector<std::pair<SharedPlane, double> > dist;
                for (SharedPlane plane : plns)
                    if (plane && plane->accept(p))
                        dist.push_back(std::make_pair(plane, plane->squareDistance(p)));

                if (!dist.empty())
                {
                    std::sort(dist.begin(), dist.end(), [](const std::pair<SharedPlane, double>& a, const std::pair<SharedPlane, double>& b){ return a.second < b.second; });
                    dist[0].first->addPoint(p, colors);
                }
            }
        }

        for (SharedPlane plane : plns)
        {
            if (plane)
            {
                plane->computeEquation();
                planes.push_back(plane);
            }
        }
    }
    else
    {
        this->getPoints(pts);
        std::vector<SharedPoint> remaining_pts = pts;
        for (int i = 0 ; i < 2 ; ++i)
        {
            SharedPlane plane = Ransac::ransac(remaining_pts, epsilon, numStartPoints, numPoints, steps, generator, colors);
            if (!plane)
                return;
            planes.push_back(plane);
            std::uniform_int_distribution<int> distribution(0, 255);
            auto random = std::bind(distribution, generator);
            plane->setColor(RGB(random(), random(), random()), colors);
        }
    }
}

int Octree::Node::findOctant(SharedPoint p) const
{
    int oct = 0;
    if (p->x >= center.x) oct |= 4;
    if (p->y >= center.y) oct |= 2;
    if (p->z >= center.z) oct |= 1;
    return oct;
}

bool Octree::Node::isLeafNode() const
{
    return children[0].get() == nullptr;
}

bool Octree::Node::insert(SharedPoint p, unsigned int depth)
{
    bool result = false;

    if (depth == 0)
    {
        return result;
    }
    --depth;

    if (!isLeafNode())
        result = children[findOctant(p)]->insert(p, depth);
    else
    {
        if (point.get() == nullptr)
        {
            point = p;
            result = true;
        }
        else
        {
            SharedPoint oldPoint = point;
            point.reset();

            for (int i = 0 ; i < 8 ; ++i)
            {
                Vec3d newCenter = center;
                newCenter.x += halfSize.x * (i&4 ? 0.5 : -0.5);
                newCenter.y += halfSize.y * (i&2 ? 0.5 : -0.5);
                newCenter.z += halfSize.z * (i&1 ? 0.5 : -0.5);
                children[i] = std::make_shared<Node>(newCenter, halfSize / 2);
            }

            result = children[findOctant(oldPoint)]->insert(oldPoint, depth)
                    && children[findOctant(p)]->insert(p, depth);

            if (!result)
            {
                for (int i = 0 ; i < 8 ; ++i)
                    children[i].reset();
                point = oldPoint;
            }
        }
    }

    if (result)
        ++count;
    return result;
}
