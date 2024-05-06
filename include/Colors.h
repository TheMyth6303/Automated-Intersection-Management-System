#ifndef COLORS_H
#define COLORS_H

struct Color {
    public:
        int r, b, g;
        Color(int R, int G, int B) {
            r = R;
            g = G;
            b = B;
        };
};

namespace Colors {
const Color GROUND(14, 149, 148);
const Color ROAD(64, 59, 59);
const Color DIVIDER(0, 0, 0);
const Color VEHICLE(255, 255, 255);
} // namespace Colors

#endif