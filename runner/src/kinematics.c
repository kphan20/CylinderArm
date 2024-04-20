#include <stdint.h>
#include <avr/pgmspace.h>
#include "data.h"

static const JOINT_DISTANCE LAST_LINK_LENGTH = 20;
static const JOINT_DISTANCE L1_MIN = 0;
static const JOINT_DISTANCE L1_MAX = 400;
static const JOINT_DISTANCE L2_MIN = 0;
static const JOINT_DISTANCE L2_MAX = 200;
static const int16_t cosLut[90] PROGMEM = {32768,32763,32748,32723,32688,32643,32588,32523,32449,32364,32270,32165,32051,31928,31794,31651,31498,31336,31164,30982,30791,  
30591,30381,30163,29935,29697,29451,29196,28932,28659,28377,28087,27788,27481,27165,26841,26509,26169,25821,25465,25101,        
24730,24351,23964,23571,23170,22762,22347,21926,21497,21062,20621,20173,19720,19260,18794,18323,17846,17364,16876,16384,        
15886,15383,14876,14364,13848,13327,12803,12275,11743,11207,10668,10125,9580,9032,8480,7927,7371,6812,6252,5690,
5126,4560,3993,3425,2855,2285,1714,1143,571};
static const short cosLutShift = 15;

static const int16_t sinLut[90] PROGMEM = {0,571,1143,1714,2285,2855,3425,3993,4560,5126,5690,6252,6812,7371,7927,8480,9032,9580,10125,10668,11207,
11743,12275,12803,13327,13848,14364,14876,15383,15886,16383,16876,17364,17846,18323,18794,19260,19720,20173,20621,21062,        
21497,21926,22347,22762,23170,23571,23964,24351,24730,25101,25465,25821,26169,26509,26841,27165,27481,27788,28087,28377,        
28659,28932,29196,29451,29697,29935,30163,30381,30591,30791,30982,31164,31336,31498,31651,31794,31928,32051,32165,32270,        
32364,32449,32523,32588,32643,32688,32723,32748,32763};
static const short sinLutShift = 15;

// EVERYTHING IS MULTIPLIED BY 2^4?

int16_t cos(ANGLE a) {
    unsigned r1 = 90 << ANGLE_SHIFT;
    unsigned index = 0;
    short flip = 1;
    if (a == r1 || a == r1 * 3)  {
        return 0;
    }
    else if (a < r1) {
        index = a >> ANGLE_SHIFT;
    }
    else if (a < r1 * 3) {
        index = ((180 << ANGLE_SHIFT) - a) >> ANGLE_SHIFT;
        flip = -1;
    } else {
        index = ((360 << ANGLE_SHIFT) - a) >> ANGLE_SHIFT;
    }
    int16_t currVal = cosLut[index];
    if (index * sizeof(cosLut[0]) == sizeof(cosLut) - sizeof(cosLut[0])) return currVal;

    currVal = ((uint16_t)currVal + cosLut[index + 1]) >> 1;

    return currVal;
}

int16_t sin(ANGLE a) {
    uint32_t a2 = a * a;
    uint32_t a3 = a * a * a;
    return (a - );
}

struct Point forward(struct JointState js) {
    struct Point res;
    int16_t j1Cos = cos(js.j1);
    int16_t j4Cos = cos(js.j4);
    res.x = js.j3 * j1Cos + LAST_LINK_LENGTH * j1Cos * j4Cos;
    res.y = js.j3 * j1Cos + LAST_LINK_LENGTH * j1Cos * j4Cos;
    res.z = js.j2 + LAST_LINK_LENGTH * sin(js.j4);
    return res;
}

struct JointState inverse(struct Point p, struct Orientation o) {

}