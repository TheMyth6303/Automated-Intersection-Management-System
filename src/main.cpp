#include <Constants.h>
#include <SDL.h>
#include <Simulator.h>
#include <VehicleManager.h>

int n = 8;

int main(int argc, char *argv[]) {
    // for frame capping
    uint32_t frameStart;
    int frameTime;

    // generate vehicles and get solution
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    vehicleManager->generateRandom(n);
    bool solved = vehicleManager->getSolution();

    if (!solved) {
        return EXIT_FAILURE;
    }

    // simulation
    Simulator *simulator = new Simulator;
    simulator->init("simulation");
    while (simulator->isRunning) {
        frameStart = SDL_GetTicks();
        simulator->render();
        simulator->handleEvents();
        simulator->update();
        frameTime = SDL_GetTicks() - frameStart;
        if (frameTime < 1000 / FPS) {
            SDL_Delay(1000 / FPS - frameTime);
        }
    }
    simulator->destroy();

    return EXIT_SUCCESS;
}