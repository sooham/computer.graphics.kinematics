#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>
#include <algorithm> // std::max
#include <cmath>
#include <iostream>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
        if (keyframes.size() == 0) {
            return Eigen::Vector3d::Zero();
        }
        /*
        std::cout << "query time t is: " << t << "\n";
        std::cout << "keyframes size is " << keyframes.size() << "\n";
        for (int i=0; i<keyframes.size(); i++) {
            std::cout << "index: " << i << "\n";
            std::cout << "time t: " << keyframes[i].first << "\n";
            std::cout << keyframes[i].second << "\n";
        }
        */

        // if query time exceeds keyframe time, loop
        double max_t = keyframes[keyframes.size()-1].first;
        t = std::fmod(t, max_t);

        // find the index for query time t
        int query_idx = -1;
        int i=0;
        for (; i<keyframes.size()-1; i++) {
            double cur_t = keyframes[i].first;
            double nxt_t = keyframes[i+1].first;
            if (cur_t <= t and t < nxt_t) {
                // if is endpoint create the last point
                query_idx = i;
            }
        }

        i = query_idx;
        //std::cout << "query_idx is " << i << "\n";
        // get the points around i
        Eigen::Vector3d p0, p1, p2, p3;
        double t0, t1, t2, t3;
        if (i == 0) {
            p0 = keyframes[i].second;
            p1 = keyframes[i].second;
            p2 = keyframes[i+1].second;
            p3 = keyframes[i+2].second;

            t0 = 2*keyframes[i].first - keyframes[i+1].first;
            t1 = keyframes[i].first;
            t2 = keyframes[i+1].first;
            t3 = keyframes[i+2].first;
        } else if (i == keyframes.size() - 2) {
            p0 = keyframes[i-1].second;
            p1 = keyframes[i].second;
            p2 = keyframes[i+1].second;
            p3 = keyframes[i+1].second;

            t0 = keyframes[i-1].first;
            t1 = keyframes[i].first;
            t2 = keyframes[i+1].first;
            t3 = 2*keyframes[i+1].first - keyframes[i].first;
        } else {
            p0 = keyframes[i-1].second;
            p1 = keyframes[i].second;
            p2 = keyframes[i+1].second;
            p3 = keyframes[i+2].second;

            t0 = keyframes[i-1].first;
            t1 = keyframes[i].first;
            t2 = keyframes[i+1].first;
            t3 = keyframes[i+2].first;
        }

        // interpolate
        double it = (t - t1) / (t2 - t1);
        Eigen::Vector3d m1, m2;
        m1 = (p2 - p0) / (t2 - t0);
        m2 = (p3 - p1) / (t3 - t1);

        // 2t^3-3t^2+1
        // t^3-2t^2+t
        // -2t^3+3t^2
        // t^3-t^2

        return (2*std::pow(it,3)-3*std::pow(it,2)+1) *p1
             + (std::pow(it,3)-2*std::pow(it,2)+it)*(t2-t1) *m1
             + (-2*std::pow(it,3)+3*std::pow(it,2)) *p2
             + (std::pow(it,3)-std::pow(it,2))*(t2-t1) *m2;
}
