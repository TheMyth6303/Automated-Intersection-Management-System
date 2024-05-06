#ifndef SIMULATOR_H
#define SIMULATOR_H

class Simulator {
    private:
        void renderBG();
        void renderVehicles();

    public:
        bool isRunning;
        Simulator();
        void init(const char *title);
        void render();
        void handleEvents();
        void update();
        void destroy();
        ~Simulator();
};

#endif