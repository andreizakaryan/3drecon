#ifndef Octree_H
#define Octree_H

#include <vector>
#include <random>
#include "PointCloud.h"

// Octree
class Octree {
public:
    Octree(const PointCloud& cloud, unsigned int maxdepth);

    // Detect planes in the point cloud.
    void detectPlanes(int depthThreshold, double epsilon, int numStartPoints, int numPoints, int steps, double countRatio, std::default_random_engine& generator, std::vector<SharedPlane>& planes, UnionFindPlanes& colors, double dCos) const;

private:
    // Node of the tree
    class Node {
    public:
        Node(const Vec3d& origin, const Vec3d& halfDimension);

        // Insert a point with max recursion depth. Return false if max depth reached, true otherwise.
        bool insert(SharedPoint p, unsigned int maxdepth);
        
        // Detect planes in this subtree.
        void detectPlanes(int depthThreshold, double epsilon, int numStartPoints, int numPoints, int steps, double countRatio, std::default_random_engine& generator, std::vector<SharedPlane>& planes, UnionFindPlanes& colors, double dCos, std::vector<SharedPoint>& pts) const;

        // Remove planes that have too few points, according to countRatio.
        static void removeSmallPlanes(std::vector<SharedPlane>& planes, double countRatio, UnionFind<SharedPoint, std::pair<RGB, bool>>& colors);

    private:

        void getPoints(std::vector<SharedPoint>& pts) const;
        bool isLeafNode() const;
        int findOctant(SharedPoint p) const;

        Vec3d center;
        Vec3d halfSize;

        std::shared_ptr<Node> children[8];
        SharedPoint point;
        unsigned int count;
    };

    Node mRoot;
};

#endif
