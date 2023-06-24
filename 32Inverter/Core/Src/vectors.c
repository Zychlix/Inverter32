#include "vectors.h"
#include "math.h"

// TODO: pass by reference or pointer?

vec_t normalize(vec_t a) {
    float lambda = 1 / sqrtf(a.x * a.x + a.y * a.y);
    vec_t result = {
            lambda * a.x,
            lambda * a.y
    };

    return result;
}

vec_t limit_amplitude(vec_t a, float max_amplitude) {
    float amplitude = sqrtf(a.x * a.x + a.y * a.y);
    float lambda = 1;
    if (amplitude > max_amplitude) lambda = max_amplitude / amplitude;

    vec_t result = {
            lambda * a.x,
            lambda * a.y
    };

    return result;
}

vec_t rotate90(vec_t a) {
    vec_t result = {
            -a.y,
            +a.x,
    };

    return result;
}

vec_t rotate90neg(vec_t a) {
    vec_t result = {
            a.y,
            -a.x,
    };

    return result;
}

vec_t rotate120(vec_t a) {
    const float sin120 = 0.8660254037844387f;
    const float cos120 = -0.5f;

    vec_t result = {
            a.x * cos120 - a.y * sin120,
            a.x * sin120 + a.y * cos120,
    };

    return result;
}

vec_t rotate180(vec_t a) {
    vec_t result = {
            -a.x,
            -a.y,
    };

    return result;
}

vec_t angle(float phi) {
    vec_t result = {
            cosf(phi),
            sinf(phi),
    };

    return result;
}

vec_t clarkeTransform(abc_t in) {
    vec_t result = {
            (2 * in.a - in.b - in.c) / 3,
            sqrtf(3) * (in.b - in.c) / 3
    };

    return result;
};

abc_t inverseClarkeTransform(vec_t in) {
    abc_t result = {
            in.x,
            (-in.x + sqrtf(3) * (in.y)) / 2,
            (-in.x - sqrtf(3) * (in.y)) / 2
    };

    return result;
};

vec_t parkTransform(vec_t in, vec_t phi) {
    // phi has to be normalized
    const float sin = phi.y;
    const float cos = phi.x;
    vec_t result = {
            cos * in.x + sin * in.y,
            -sin * in.x + cos * in.y
    };

    return result;
};

vec_t inverseParkTransform(vec_t in, vec_t phi) {
    // phi has to be normalized
    const float sin = phi.y;
    const float cos = phi.x;

    vec_t result = {
            cos * in.x - sin * in.y,
            sin * in.x + cos * in.y,
    };

    return result;
};

