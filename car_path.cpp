#include <pybind11/stl.h>
#include <pybind11/pybind11.h>

#include <string>
#include <utility>

#include <vector>

#include "rlphysics.h"


namespace py = pybind11;


class CarPath
{
    public:
        std::vector<double> player_circle;
        std::vector<double> target_circle;
        std::vector<double> player_tangent;
        std::vector<double> target_tangent;
        double total_distance;
        double total_time;
        bool forwards;
};


std::vector<double> tangent_point(const std::vector<double>& circle, double circle_radius,
                                  const std::vector<double>& point, int angle_sign) {

    std::vector<double> circle2d = circle, point2d = point, result(2);
    circle2d.resize(2);
    point2d.resize(2);

    double circle_distance = dist2d(circle2d, point2d) + 1e-9;
    double relative_angle = acos(Range(circle_radius / circle_distance, 1));
    double point_angle = angle2d(point2d, circle2d);
    double tangent_angle = (point_angle - relative_angle * Sign(angle_sign));

    result[0] = cos(tangent_angle) * circle_radius + circle2d[0];
    result[1] = sin(tangent_angle) * circle_radius + circle2d[1];

    return result;

}



std::vector<std::vector<double>> circles_tangent(const std::vector<double>& c1, double c1r, int c1s,
                                                 const std::vector<double>& c2, double c2r, int c2s) {

    bool out = Sign(c1s) != Sign(c2s);
    vec2 vc1 = a2(c1), vc2 = a2(c2), c1t, c2t;

    if (c1r != c2r || !(out)) {
        vec2 pc2t = a2(tangent_point(c2, c2r - c1r * Sign(out), c1, c2s));
        c2t = set_dist_2d(vc2, pc2t, c2r * Sign(c2r > c1r || !(out)));
        c1t = vc1 + c2t - pc2t;
    }
    else {
        c2t = set_dist_ang_2d(vc2, vc1, c2r, -PI / 2 * Sign(c2s));
        c1t = vc1 + c2t - vc2;
    }

    std::vector<std::vector<double>> result(2, std::vector<double>(2));
    result[0] = l2(c1t);
    result[1] = l2(c2t);

    return result;
}


CarPath shootingPath(const std::vector<double>& tL, const std::vector<double>& pL,
                     const std::vector<double>& pR, double trd, double prd,
                     const std::vector<double>& goal, double pyv, double bR = 92, double pB = 100) {

    vec3 tLb = set_dist(a3(tL), a3(goal), -bR);

    vec3 ltL = local_space(a3(tL), a3(pL), a3(pR));
    vec3 ltLb = local_space(tLb, a3(pL), a3(pR));

    std::vector<double> origin = {0, 55};

    CarPath path;
    bool bFirstLoop = true;

    for (int t = 0; t < 2; t++)
    {
        std::vector<double> ltcL = l3(set_dist_ang(ltLb, ltL, trd, PI / 2 * Sign(t)));
        for (int p = 0; p < 2; p++)
        {
            for (int f = 0; f < 2; f++)
            {
                std::vector<double> lpcL = {(float) (prd * Sign(p) * Sign(f)), 55, 0};
                std::vector<std::vector<double>> tangents = circles_tangent(
                    lpcL, prd, -Sign(p), ltcL, trd, -Sign(t));
                std::vector<double> lpcTL = tangents[0], ltcTL = tangents[1];
                double pca = abs(relative_angle(lpcTL, origin, lpcL));
                double tca = Range360(relative_angle(ltcTL, l3(ltLb), ltcL) * Sign(t), PI);
                double td = pca * prd + dist2d(lpcTL, ltcTL) + tca * trd;
                double tt = min_travel_time(td, pyv * Sign(f), pB * f);
                if (bFirstLoop || tt < path.total_time)
                {
                    path.player_circle = lpcL;
                    path.target_circle = ltcL;
                    path.player_tangent = lpcTL;
                    path.target_tangent = ltcTL;
                    path.total_distance = td;
                    path.total_time = tt;
                    path.forwards = (bool) f;
                    bFirstLoop = false;
                }
            }
        }
    }

    return path;
}


PYBIND11_MODULE(car_path, m) {

    m.doc() = "Car path utilities";

    m.def("tangent_point", &tangent_point, "Circle tangent passing through point, angle sign + if clockwise else -");
    m.def("circles_tangent", &circles_tangent, "Return line tangent to two circles");
    m.def("shootingPath", &shootingPath);
    m.def("min_travel_time", &min_travel_time);
    m.def("max_travel_distance", &max_travel_distance);
    

    pybind11::class_<CarPath>(m, "CarPath")
        .def(pybind11::init<>())
        .def_readwrite("player_circle", &CarPath::player_circle)
        .def_readwrite("target_circle", &CarPath::target_circle)
        .def_readwrite("player_tangent", &CarPath::player_tangent)
        .def_readwrite("target_tangent", &CarPath::target_tangent)
        .def_readwrite("total_distance", &CarPath::total_distance)
        .def_readwrite("total_time", &CarPath::total_time)
        .def_readwrite("forwards", &CarPath::forwards);
}