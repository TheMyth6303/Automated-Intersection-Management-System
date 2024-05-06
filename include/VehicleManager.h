#ifndef VEHICLE_MANAGER_H
#define VEHICLE_MANAGER_H

#include <Vehicle.h>
#include <vector>

class VehicleManager {
    private:
        VehicleManager();
        VehicleManager(const VehicleManager &) = delete;
        VehicleManager &operator=(const VehicleManager &) = delete;
        ~VehicleManager();
        static float time;
        static std::vector<float> s0;
        static std::vector<float> v0;

    public:
        static std::vector<Vehicle *> vehicles;
        static VehicleManager *getInstance();
        void generateRandom(int n);
        bool getSolution();
        void update();
};

#endif