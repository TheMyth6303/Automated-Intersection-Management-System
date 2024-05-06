// initial condition properties
const int N = 1;
const int W = 48;
const int L = W / 2;
const int B = W / 4;
const int ALPHA = 2 * W / 3;
const int BETA = 4 * W / 3;
const int AS[4] = {ALPHA, 0, -BETA, 0};
const int VMAX = 3 * W;

// simulation properties
const int W_SIZE = 16 * W;
const int FPS = 24;

// direction constants
const int NORTH = 1;
const int EAST = 2;
const int SOUTH = 3;
const int WEST = 4;
const int LEFT = 1;
const int STRAIGHT = 2;
const int RIGHT = 3;