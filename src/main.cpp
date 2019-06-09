#include "PointCloud.h"
#include "Octree.h"
#include "Ply.h"

#include <fstream>
#include <opencv2/core.hpp>

void run(PointCloud& cloud, const std::string& name)
{
    Ply ply;
    std::default_random_engine random;
    
    Octree octree(cloud, 30);
    std::vector<SharedPlane> planes;
    
    octree.detectPlanes(100, 0.05, 10, 30, 10, 0.005, random, planes, cloud.colors(), std::cos(3.1415/180 * 15));

    std::sort(planes.begin(), planes.end(), [](const SharedPlane& a, const SharedPlane& b){return a->getCount() < b->getCount();});

    std::ofstream out((name + ".planes").c_str());
    for (unsigned int i = 0 ; i < planes.size() ; ++i)
    {
        std::cout << *planes[i] << std::endl;
        out << *planes[planes.size() - i - 1] << std::endl;
    }
    out.close();

    //cloud.toPly(name + ".ply", true);
    
    std::vector<SharedPlane> filtered;
    PointCloud cl;
    
    for (auto p: planes) {
        if (p->points().size() >= 100) {
            p->points().clear();
            for (auto point: cloud.points()) {
                if(p->accept(point)) {
                    p->points().push_back(point);
                    cl.addPoint(point, RGB(255, 255, 255));
                }
            }
            p -> flatten();
            for (auto point : p->points()) {
                cl.addPoint(point, RGB(255, 255, 255));
            }
        }
    }

    ply.write(name, cloud);
}

int main(int argc, char** argv)
{
    PointCloud cloud;
    Ply ply;
    ply.read(argv[1], cloud);
    run(cloud, argv[2]);
    return 0;
}
