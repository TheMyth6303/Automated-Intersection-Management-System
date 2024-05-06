#include <Constants.h>
#include <Vehicle.h>
#include <VehicleManager.h>
#include <array>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>

std::vector<Vehicle *> VehicleManager::vehicles;
std::vector<float> VehicleManager::s0;
std::vector<float> VehicleManager::v0;
float VehicleManager::time;

VehicleManager::VehicleManager() {}
VehicleManager::~VehicleManager() {}

bool valid(std::vector<Vehicle *> vehicles, float s, int in_lane) {
    for (std::size_t j = 0; j < vehicles.size(); j++) {
        if (vehicles[j]->in_lane == in_lane && abs(s - vehicles[j]->s) <= 2 * L) {
            return false;
        }
    }
    return true;
}

void VehicleManager::generateRandom(int n) {
    // change to better random generation
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 generator(seed);
    std::uniform_int_distribution<int> inLaneDistribution(1, 4);
    std::uniform_int_distribution<int> turnDistribution(1, 3);
    std::uniform_real_distribution<float> distDistribution(W, 6 * W);
    std::uniform_real_distribution<float> velDistribution(VMAX / 3, VMAX);

    for (int i = 0; i < n; i++) {
        int in_lane = inLaneDistribution(generator);
        int turn = turnDistribution(generator);
        float s = distDistribution(generator);
        while (!valid(vehicles, s, in_lane)) {
            s = distDistribution(generator);
        }
        float v = velDistribution(generator);

        Vehicle *vehicle = new Vehicle(s, v, in_lane, turn);
        vehicles.push_back(vehicle);
        s0.push_back(s);
        v0.push_back(v);
    }
}

VehicleManager *VehicleManager::getInstance() {
    static VehicleManager instance;
    time = 0;
    return &instance;
}

void VehicleManager::update() {
    time += 1.0 / FPS;
    for (Vehicle *v : vehicles) {
        v->update(time);
    }
};

std::string exec(const char *cmd) {
    std::array<char, 1024> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
        throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 1024, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}

bool VehicleManager::getSolution() {
    // call to python file
    std::string pythonCmd = "python solver.py ";
    pythonCmd.append(std::to_string(N) + " ");
    pythonCmd.append(std::to_string(W) + " ");
    pythonCmd.append(std::to_string(L) + " ");
    pythonCmd.append(std::to_string(B) + " ");
    pythonCmd.append(std::to_string(ALPHA) + " ");
    pythonCmd.append(std::to_string(BETA) + " ");
    pythonCmd.append(std::to_string(VMAX) + " ");
    pythonCmd.append(std::to_string(vehicles.size()) + " ");
    for (Vehicle *v : vehicles) {
        pythonCmd.append(std::to_string(v->S[0]) + " ");
        pythonCmd.append(std::to_string(v->V[0]) + " ");
        pythonCmd.append(std::to_string(v->in_lane) + " ");
        pythonCmd.append(std::to_string(v->turn) + " ");
    }
    std::cout << pythonCmd << std::endl;
    std::string pythonOutput = exec(pythonCmd.c_str());

    std::istringstream iss(pythonOutput);

    std::string condition;
    iss >> condition;

    if (condition != "SOLVED") {
        std::cerr << "\033[91m" << condition << "\033[0m";
        return false;
    }

    for (std::size_t i = 0; i < vehicles.size(); i++) {
        float currT = 0;
        for (int j = 1; j <= 4 * N; j++) {
            float t;
            iss >> t;
            currT += t;
            vehicles[i]->T.push_back(currT);
            vehicles[i]->V.push_back(vehicles[i]->V[j - 1] + AS[(j - 1) % 4] * t);
            vehicles[i]->S.push_back(vehicles[i]->S[j - 1] - (vehicles[i]->V[j - 1] * t + 0.5 * AS[(j - 1) % 4] * t * t));
        }
        float t;
        iss >> t;
        vehicles[i]->T.push_back(currT + t); // intersection time
    }
    return true;
};
