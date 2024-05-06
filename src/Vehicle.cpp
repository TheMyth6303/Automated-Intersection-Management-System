#include <Constants.h>
#include <Vehicle.h>
#include <cmath>
#include <iostream>

Vehicle::Vehicle(float s, float v, int in_lane, int turn) {
    this->s = s;
    S.push_back(s);
    V.push_back(v);
    T.push_back(0);
    this->in_lane = in_lane;
    this->turn = turn;
};

Vehicle::~Vehicle(){};

void Vehicle::update(float time) {
    for (int i = 0; i < 4 * N; i++) {
        if (T[i] < time && T[i + 1] >= time) {
            float t = time - T[i];
            s = S[i] - (V[i] * t + 0.5 * AS[i % 4] * t * t);
        }
    }
    if (time >= T[4 * N] && time <= T[4 * N + 1]) {
        float t = time - T[4 * N];
        s = S[4 * N] - V[4 * N] * t;
    } else if (time >= T[4 * N + 1]) {
        float t = time - T[4 * N + 1];
        s = S[4 * N] - V[4 * N] * (T[4 * N + 1] - T[4 * N]) - VMAX * t;
    }
};

std::tuple<float, float, double> Vehicle::getXYTheta() {
    float x, y;
    double theta;

    // considering vehicle is coming from north
    if (s >= 0) {
        x = W / 2;
        y = -W - s - L / 2;
        theta = M_PI / 2;
    } else {
        float s_abs = -s;
        if (turn == LEFT) {
            float r = (W + L) / 2;
            if (s_abs >= M_PI * r / 2) {
                x = W + L / 2 + s_abs - M_PI * r / 2;
                y = -W / 2;
                theta = 0;
            } else {
                double phi = s_abs / r;
                theta = M_PI / 2 - phi;
                x = W + L / 2 - r * std::cos(phi);
                y = -W - L / 2 + r * std::sin(phi);
            }
        } else if (turn == STRAIGHT) {
            x = +W / 2;
            y = -W - L / 2 + s_abs;
            theta = M_PI / 2;
        } else if (turn == RIGHT) {
            float r = (3 * W + L) / 2;
            if (s_abs >= M_PI * r / 2) {
                x = -W - L / 2 - s_abs + M_PI * r / 2;
                y = +W / 2;
                theta = 0;
            } else {
                double phi = s_abs / r;
                theta = phi - M_PI / 2;
                x = -W - L / 2 + r * std::cos(phi);
                y = -W - L / 2 + r * std::sin(phi);
            }
        }
    }

    float X, Y;
    double THETA;
    // rotating according to actual in-lane direction
    if (in_lane == NORTH) {
        X = x;
        Y = y;
        THETA = theta;
    } else if (in_lane == EAST) {
        X = -y;
        Y = x;
        THETA = theta - M_PI / 2;
    } else if (in_lane == SOUTH) {
        X = -x;
        Y = -y;
        THETA = theta;
    } else {
        X = y;
        Y = -x;
        THETA = theta - M_PI / 2;
    }

    return std::tuple<float, float, double>(W_SIZE / 2 + X, W_SIZE / 2 + Y, THETA * 180 / M_PI);
};
