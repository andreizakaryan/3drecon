#ifndef PLY_H
#define PLY_H

#include <string>
#include <vector>
#include <memory>
#include "Plane.h"
#include "UnionFind.h"
#include "PointCloud.h"

class Ply
{
    friend class Test;

public:
    bool write(const std::string& filename, PointCloud& cloud);
    void read(const std::string& filename, PointCloud& cloud);
private:
};

#endif // PLY_H
