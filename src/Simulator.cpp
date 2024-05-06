#include <Colors.h>
#include <Constants.h>
#include <SDL.h>
#include <Simulator.h>
#include <VehicleManager.h>

SDL_Window *window = NULL;
SDL_Renderer *renderer = NULL;
SDL_Texture *vehicleTexture;

VehicleManager *vehicleManager = VehicleManager::getInstance();

Simulator::Simulator(){};
Simulator::~Simulator(){};

void Simulator::init(const char *title) {
    isRunning = true;
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        isRunning = false;
    }

    window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, W_SIZE, W_SIZE, SDL_WINDOW_SHOWN);
    if (window == NULL) {
        isRunning = false;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == NULL) {
        isRunning = false;
    }

    SDL_Surface *surface = SDL_CreateRGBSurface(0, L, B, 32, 0, 0, 0, 0);
    SDL_FillRect(surface, NULL, SDL_MapRGBA(surface->format, Colors::VEHICLE.r, Colors::VEHICLE.g, Colors::VEHICLE.b, 255));
    vehicleTexture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_FreeSurface(surface);
};

void Simulator::renderBG() {
    SDL_Rect rect;
    SDL_SetRenderDrawColor(renderer, Colors::GROUND.r, Colors::GROUND.g, Colors::GROUND.b, 255);
    rect = {0, 0, W_SIZE, W_SIZE};
    SDL_RenderFillRect(renderer, &rect);

    SDL_SetRenderDrawColor(renderer, Colors::ROAD.r, Colors::ROAD.g, Colors::ROAD.b, 255);
    rect = {W_SIZE / 2 - W, 0, 2 * W, W_SIZE};
    SDL_RenderFillRect(renderer, &rect);
    rect = {0, W_SIZE / 2 - W, W_SIZE, 2 * W};
    SDL_RenderFillRect(renderer, &rect);

    SDL_SetRenderDrawColor(renderer, Colors::DIVIDER.r, Colors::DIVIDER.g, Colors::DIVIDER.b, 255);
    SDL_RenderDrawLine(renderer, W_SIZE / 2, 0, W_SIZE / 2, W_SIZE / 2 - W);
    SDL_RenderDrawLine(renderer, W_SIZE / 2 - W, 0, W_SIZE / 2 - W, W_SIZE / 2 - W);
    SDL_RenderDrawLine(renderer, W_SIZE / 2 + W, 0, W_SIZE / 2 + W, W_SIZE / 2 - W);
    SDL_RenderDrawLine(renderer, W_SIZE / 2, W_SIZE / 2 + W, W_SIZE / 2, W_SIZE);
    SDL_RenderDrawLine(renderer, W_SIZE / 2 - W, W_SIZE / 2 + W, W_SIZE / 2 - W, W_SIZE);
    SDL_RenderDrawLine(renderer, W_SIZE / 2 + W, W_SIZE / 2 + W, W_SIZE / 2 + W, W_SIZE);
    SDL_RenderDrawLine(renderer, 0, W_SIZE / 2, W_SIZE / 2 - W, W_SIZE / 2);
    SDL_RenderDrawLine(renderer, 0, W_SIZE / 2 - W, W_SIZE / 2 - W, W_SIZE / 2 - W);
    SDL_RenderDrawLine(renderer, 0, W_SIZE / 2 + W, W_SIZE / 2 - W, W_SIZE / 2 + W);
    SDL_RenderDrawLine(renderer, W_SIZE / 2 + W, W_SIZE / 2, W_SIZE, W_SIZE / 2);
    SDL_RenderDrawLine(renderer, W_SIZE / 2 + W, W_SIZE / 2 - W, W_SIZE, W_SIZE / 2 - W);
    SDL_RenderDrawLine(renderer, W_SIZE / 2 + W, W_SIZE / 2 + W, W_SIZE, W_SIZE / 2 + W);
}

void Simulator::renderVehicles() {
    SDL_FRect rect;
    SDL_SetRenderDrawColor(renderer, Colors::VEHICLE.r, Colors::VEHICLE.g, Colors::VEHICLE.b, 255);
    for (Vehicle *v : vehicleManager->vehicles) {
        std::tuple<float, float, double> XYTheta = v->getXYTheta();
        float x = std::get<0>(XYTheta);
        float y = std::get<1>(XYTheta);
        double theta = std::get<2>(XYTheta);
        rect = {x - L / 2, y - B / 2, static_cast<float>(L), static_cast<float>(B)};
        // SDL_RenderCopyExF(renderer, vehicleTexture, nullptr, &rect, static_cast<double>(theta), nullptr, SDL_FLIP_NONE);
        SDL_RenderCopyExF(renderer, vehicleTexture, nullptr, &rect, theta, nullptr, SDL_FLIP_NONE);
    }
}

void Simulator::render() {
    SDL_RenderClear(renderer);
    renderBG();
    renderVehicles();
    SDL_RenderPresent(renderer);
}

void Simulator::handleEvents() {
    SDL_Event event;
    while (SDL_PollEvent(&event) > 0) {
        if (event.type == SDL_QUIT) {
            isRunning = false;
        }
    }
}

void Simulator::update() { vehicleManager->update(); }

void Simulator::destroy() {
    SDL_DestroyWindow(window);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyTexture(vehicleTexture);
    SDL_Quit();
}