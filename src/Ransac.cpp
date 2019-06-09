#include "Ransac.h"

SharedPlane Ransac::ransac(std::vector<SharedPoint>& points, double epsilon, int numStartPoints, int numPoints, int steps, std::default_random_engine& generator, UnionFindPlanes& colors)
{
    SharedPlane result;
    if (points.size() < numStartPoints || numStartPoints < 3)
        return result;

    Vec3d center;
    Vec3d meansq;

    for (auto&& point : points)
    {
        center += *point;
        meansq += point->cmul(*point);
    }

    center /= points.size();
    meansq /= points.size();

    Vec3d stddev = meansq - center.cmul(center);

    double radius = std::sqrt(stddev.x + stddev.y + stddev.z);

    epsilon *= radius;

    std::vector<SharedPoint> result_pts;
    std::vector<SharedPoint> remaining_pts;
    double score = -1;

    for (int t = 0 ; t < steps ; ++t) {
        std::vector<SharedPoint> pts;
        for (int i = 0 ; i < numStartPoints ; ++i) {
            std::uniform_int_distribution<int> distribution(i, points.size() - 1);
            int k = distribution(generator);

            std::swap(points[i], points[k]);
            pts.push_back(points[i]);
        }

        SharedPlane shared_plane = std::make_shared<Plane>(pts);
        Plane& plane = *shared_plane;

        std::vector<SharedPoint> pts_out;
        matchPoints(points, plane, epsilon, pts, pts_out);

        if (pts.size() > numPoints)
        {
            plane.setPoints(pts);
            double error = 0;
            for (auto&& p : pts)
                error += plane.squareDistance(p);
            if (score < 0 || error < score)
            {
                result = shared_plane;
                result_pts = pts;
                remaining_pts = pts_out;
                score = error;
            }
        }
    }

    for (auto&& p : result_pts)
    {
        colors.merge(p, result_pts[0]);
    }

    points = remaining_pts;
    return result;
}

void Ransac::matchPoints(const std::vector<SharedPoint>& points, Plane& plane, double epsilon, std::vector<SharedPoint>& pts_in, std::vector<SharedPoint>& pts_out)
{
    pts_in.clear();
    pts_out.clear();
    for (auto&& p : points)
    {
        if (plane.squareDistance(p) <= epsilon)
            pts_in.push_back(p);
        else
            pts_out.push_back(p);
    }
}
