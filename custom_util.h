#include <vector>

#include "linalg.h"

const double PI = 3.141592653589793;


vec2 a2(const std::vector<double>& v) {

    vec2 result;
    result[0] = v[0];
    result[1] = v[1];
    return result;
}


vec3 a3(const std::vector<double>& v) {

    vec3 result;
    result[0] = v[0];
    result[1] = v[1];
    result[2] = v[2];
    return result;
}


std::vector<double> l2(const vec2 v) {

    std::vector<double> result(2);
    result[0] = v[0];
    result[1] = v[1];
    return result;
}


std::vector<double> l3(const vec3 v) {

    std::vector<double> result(3);
    result[0] = v[0];
    result[1] = v[1];
    result[2] = v[2];
    return result;
}



double angle2d(const std::vector<double>& a, const std::vector<double>& b) {
    return atan2(a[1] - b[1], a[0] - b[0]);
}

double angle2_2d(const std::vector<double>& a, const std::vector<double>& b) {
    return atan2(a[0] - b[0], a[1] - b[1]);
}

double dist2d(const std::vector<double>& a, const std::vector<double>& b) {
    return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2));
}


double Sign(const double x) {
    if (x > 0) {
        return 1;
    }
    else {
        return -1;
    }
}

double Range(const double v, const double r) {

    if (abs(v) > r) {
        return r * Sign(v);
    }
    else {
        return v;
    }
}


double Range180(double a, double pi = PI) {
    if (abs(a) >= 2 * pi) {
        a -= (floor(abs(a) / (2 * pi)) * 2 * pi * Sign(a));
    }
    if (abs(a) > pi) {
        a -= 2 * pi * Sign(a);
    }
    return a;
}


double Range360(double a, double pi = PI) {
    return a - floor(a / (2 * pi)) * 2 * pi;
}


double relative_angle(const std::vector<double> a, const std::vector<double> b, const std::vector<double> o) {
    return Range180(angle2_2d(a, o) - angle2_2d(b, o), PI);
}


double  tfs(double throttlespeed) {
    return abs(throttlespeed) / 3600 + 1 / 60;
}


vec2 set_dist_2d(const vec2 a, const vec2 b, double dist = 1) {

    return normalize(b - a) * dist + a;
}


vec3 set_dist(const vec3 a, const vec3 b, double dist = 1) {

    return normalize(b - a) * dist + a;
}


vec2 Rotate2D(double x, double y, double ang) {

    vec2 result;
    result[0] = x * cos(ang) - y * sin(ang);
    result[1] = y * cos(ang) + x * sin(ang);
    return result;
}


vec2 set_dist_ang_2d(const vec2 a, const vec2 b, double dist, double ang = 0) {

    vec2 c = normalize(b - a) * dist;
    return Rotate2D(c[0], c[1], ang) + a;
}

vec3 set_dist_ang(const vec3 a, const vec3 b, double dist, double ang = 0) {

    vec3 c = normalize(b - a) * dist;
    vec2 z = Rotate2D(c[0], c[1], ang);
    c[0] = z[0];
    c[1] = z[1];
    return c + a;
}



vec3 local_space(const vec3 tL, const vec3 oL, const vec3 oR)
{
    vec3 L = tL - oL;
    double pitch = -oR[0], yaw = -Range180(oR[1] - PI / 2, PI), roll = -oR[2];
    vec2 tmp = Rotate2D(L[0], L[1], yaw);
    L[0] = tmp[0]; L[1] = tmp[1];
    tmp = Rotate2D(L[1], L[2], pitch);
    L[1] = tmp[0]; L[2] = tmp[1];
    tmp = Rotate2D(L[0], L[2], roll);
    L[0] = tmp[0]; L[2] = tmp[1];
    return L;
}

