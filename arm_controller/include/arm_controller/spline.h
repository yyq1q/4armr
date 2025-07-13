#pragma once

#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;

namespace spline3d
{

struct Spline1D
{
    VectorXd a, b, c, d;
    VectorXd x; // ノード点
};

inline Spline1D computeSpline1D(const VectorXd& x, const VectorXd& y)
{
    int n = x.size() - 1;
    VectorXd h(n);
    for (int i = 0; i < n; ++i)
        h(i) = x(i+1) - x(i);

    MatrixXd A = MatrixXd::Zero(n + 1, n + 1);
    VectorXd rhs = VectorXd::Zero(n + 1);

    for (int i = 1; i < n; ++i)
    {
        A(i, i-1) = h(i-1);
        A(i, i)   = 2 * (h(i-1) + h(i));
        A(i, i+1) = h(i);
        rhs(i) = 3 * ((y(i+1) - y(i)) / h(i) - (y(i) - y(i-1)) / h(i-1));
    }

    A(0,0) = 1;
    A(n,n) = 1;

    VectorXd c = A.colPivHouseholderQr().solve(rhs);

    VectorXd b(n), d(n), a(n);
    for (int i = 0; i < n; ++i)
    {
        a(i) = y(i);
        b(i) = (y(i+1) - y(i)) / h(i) - h(i)*(2*c(i) + c(i+1)) / 3.0;
        d(i) = (c(i+1) - c(i)) / (3.0 * h(i));
    }

    Spline1D spline;
    spline.a = a;
    spline.b = b;
    spline.c = c.head(n);
    spline.d = d;
    spline.x = x;
    return spline;
}

inline double interpolateSpline1D(const Spline1D& spline, double t)
{
    int n = spline.x.size() - 1;

    if (t <= spline.x(0)) return spline.a(0);
    if (t >= spline.x(n))
    {
        double dx = spline.x(n) - spline.x(n-1);
        return spline.a(n-1) + spline.b(n-1)*dx + spline.c(n-1)*dx*dx + spline.d(n-1)*dx*dx*dx;
    }

    int i = 0;
    for (; i < n; ++i)
    {
        if (t < spline.x(i+1)) break;
    }

    double dx = t - spline.x(i);
    return spline.a(i) + spline.b(i)*dx + spline.c(i)*dx*dx + spline.d(i)*dx*dx*dx;
}

class Spline3D
{
public:
    Spline3D(const std::vector<Vector3d>& points)
    {
        if (points.size() < 2)
            throw std::runtime_error("Need at least 2 points for spline interpolation");

        int N = points.size();
        VectorXd t(N), x(N), y(N), z(N);
        for (int i = 0; i < N; ++i)
        {
            t(i) = i;
            x(i) = points[i].x();
            y(i) = points[i].y();
            z(i) = points[i].z();
        }

        sx_ = computeSpline1D(t, x);
        sy_ = computeSpline1D(t, y);
        sz_ = computeSpline1D(t, z);
        t_min_ = t(0);
        t_max_ = t(N - 1);
    }

    Vector3d interpolate(double t_norm) const
    {
        double t = t_min_ + t_norm * (t_max_ - t_min_);
        return Vector3d(
            interpolateSpline1D(sx_, t),
            interpolateSpline1D(sy_, t),
            interpolateSpline1D(sz_, t)
        );
    }

    /// 指定された距離間隔でスプラインを分割
    std::vector<Vector3d> sampleByDistance(double resolution, double dt = 1e-4, int max_iter = 100000) const
    {
        std::vector<Vector3d> sampled;
        sampled.push_back(interpolate(0.0));

        double t = 0.0;
        Vector3d last_point = sampled.back();
        int iter = 0;

        while (t < 1.0 && iter < max_iter)
        {
            double acc_length = 0.0;
            double t_inner = t;
            Vector3d prev = last_point;

            while (t_inner < 1.0)
            {
                t_inner += dt;
                if (t_inner > 1.0) t_inner = 1.0;

                Vector3d curr = interpolate(t_inner);
                acc_length += (curr - prev).norm();

                if (acc_length >= resolution)
                {
                    sampled.push_back(curr);
                    last_point = curr;
                    t = t_inner;
                    break;
                }

                prev = curr;
            }

            iter++;
        }

        return sampled;
    }

private:
    Spline1D sx_, sy_, sz_;
    double t_min_, t_max_;
};

} // namespace spline3d
