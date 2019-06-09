#include "Ply.h"
#include "PointCloud.h"

#include <fstream>
#include <numeric>
#include <string>
#include <functional>

void Ply::read(const std::string& filename, PointCloud& cloud)
{
    std::ifstream infile(filename.c_str());
    std::string line;
    bool start = false;
    while (std::getline(infile, line))
    {
        if (start) {
            std::istringstream iss(line);
            double x, y, z;
            unsigned int r, g, b;
            if (!(iss >> x >> y >> z >> r >> g >> b))
                break; 

            cloud.addPoint(std::make_shared<Point>(x, y, z, RGB(r, g, b)), RGB(r, g, b));
        }
        if (line.find("end_header") != std::string::npos) {
            start = true;
        }
    }

    cloud.boundingBox();
}

bool Ply::write(const std::string& filename, PointCloud& cloud) 
{
    std::ofstream out(filename.c_str());
    if (!out.is_open()) {
        std::cerr << "Cannot save " << filename << std::endl;
        return false;
    }

    out << "ply" << std::endl
        << "format ascii 1.0" << std::endl
        << "element vertex " << cloud.points().size() << std::endl
        << "property float x" << std::endl
        << "property float y" << std::endl
        << "property float z" << std::endl
        << "property uchar red" << std::endl
        << "property uchar green" << std::endl
        << "property uchar blue" << std::endl
        << "end_header" << std::endl;

    for (SharedPoint p : cloud.points()) {
        std::pair<RGB, bool> state = cloud.colors().at(p);
        RGB rgb = p -> color;
        out << p->x << " " << p->y << " " << p->z << " " << int(rgb.r) << " " << int(rgb.g) << " " << int(rgb.b) << " " << std::endl;
    }

    out.close();
    return true;
}