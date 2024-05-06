#ifndef VEHICLE_H
#define VEHICLE_H

#include <tuple>
#include <vector>

class Vehicle {
    public:
        float s;
        float theta;
        int in_lane;
        int turn;
        std::vector<float> T;
        std::vector<float> S;
        std::vector<float> V;

    public:
        Vehicle(float s, float v, int in_lane, int turn);
        ~Vehicle();
        void update(float time);
        std::tuple<float, float, double> getXYTheta();
};

#endif