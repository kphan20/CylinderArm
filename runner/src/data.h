#include <stdint.h>

typedef uint16_t ANGLE;
static const short ANGLE_SHIFT = 1;
typedef uint16_t JOINT_DISTANCE;

struct Point {
    int16_t x;
    int16_t y;
    int16_t z;
};

// follows z-y'-x'' intrinsic representation
// TODO need to decide angle precision
struct Orientation {
    ANGLE z;
    ANGLE y;
    ANGLE x;
};

struct JointState {
    ANGLE j1;
    JOINT_DISTANCE j2;
    JOINT_DISTANCE j3;
    ANGLE j4;
    ANGLE j5;
};