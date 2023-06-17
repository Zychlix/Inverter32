#include "vectors.h"
#include "math.h"

angle normalize(angle a){
    float lambda = 1 / sqrtf(a.x * a.x + a.y * a.y);
    angle result = {
        lambda * a.x,
        lambda * a.y
    };

    return result;
}

angle rotate90(angle a){
    angle result = {
        -a.y,
        +a.x,
    };

    return result;
}

angle rotate90neg(angle a){
    angle result = {
        a.y,
        -a.x,
    };

    return result;
}

angle rotate120(angle a){
    const float sin120 = 0.8660254037844387;
    const float cos120 = -0.5;

    angle result = {
        a.x * cos120 - a.y * sin120,
        a.x * sin120 + a.y * cos120,
    };

    return result;
}
