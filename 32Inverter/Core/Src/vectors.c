#include "vectors.h"

#include <stdbool.h>
#include <stdint.h>

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
            cos_lut(phi),
            sin_lut(phi),
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


float sin_lut_table[] = {
0.0f,
0.012368159663362913f,
0.024734427279994954f,
0.03709691109260531f,
0.049453719922738996f,
0.061802963460084105f,
0.0741427525516462f,
0.08647119949074572f,
0.09878641830579414f,
0.11108652504880456f,
0.12336963808359297f,
0.13563387837362562f,
0.14787736976946894f,
0.16009823929579747f,
0.17229461743791666f,
0.1844646384277559f,
0.19660644052928852f,
0.2087181663233351f,
0.2207979629917062f,
0.23284398260064146f,
0.2448543823835012f,
0.256827325022668f,
0.2687609789306141f,
0.28065351853009307f,
0.2925031245334109f,
0.3043079842207361f,
0.3160662917174045f,
0.32777624827017676f,
0.33943606252240655f,
0.35104395078807765f,
0.362598137324667f,
0.3740968546047932f,
0.3855383435866074f,
0.3969208539828872f,
0.4082426445287904f,
0.41950198324822896f,
0.43069714771882195f,
0.4418264253353867f,
0.45288811357192876f,
0.4638805202420893f,
0.4748019637580112f,
0.48565077338758367f,
0.49642528951002635f,
0.5071238638697732f,
0.517744859828618f,
0.5282866526160837f,
0.5387476295779736f,
0.5491261904230724f,
0.5594207474679531f,
0.5696297258798572f,
0.5797515639176074f,
0.5897847131705194f,
0.599727638795273f,
0.6095788197507078f,
0.6193367490305087f,
0.6289999338937424f,
0.6385668960932144f,
0.6480361721016051f,
0.6574063133353583f,
0.6666758863762793f,
0.6758434731908174f,
0.684907671346991f,
0.693867094228929f,
0.7027203712489901f,
0.7114661480574331f,
0.7201030867496006f,
0.7286298660705873f,
0.7370451816173639f,
0.7453477460383189f,
0.7535362892301954f,
0.7616095585323879f,
0.7695663189185699f,
0.7774053531856258f,
0.7851254621398549f,
0.7927254647804206f,
0.800204198480017f,
0.8075605191627239f,
0.8147933014790244f,
0.8219014389779584f,
0.8288838442763838f,
0.8357394492253214f,
0.8424672050733574f,
0.8490660826270787f,
0.855535072408516f,
0.8618731848095704f,
0.8680794502434017f,
0.8741529192927527f,
0.8800926628551883f,
0.8858977722852274f,
0.8915673595333443f,
0.897100557281821f,
0.9024965190774262f,
0.9077544194609038f,
0.9128734540932493f,
0.9178528398787551f,
0.9226918150848067f,
0.9273896394584102f,
0.9319455943394346f,
0.9363589827705492f,
0.9406291296038439f,
0.94475538160411f,
0.9487371075487712f,
0.9525736983244457f,
0.9562645670201275f,
0.9598091490169692f,
0.9632069020746571f,
0.9664573064143603f,
0.9695598647982466f,
0.9725141026055469f,
0.9753195679051626f,
0.9779758315248003f,
0.9804824871166253f,
0.9828391512194238f,
0.9850454633172633f,
0.9871010858946441f,
0.9890057044881307f,
0.990759027734458f,
0.992360787415103f,
0.9938107384973164f,
0.9951086591716065f,
0.9962543508856719f,
0.9972476383747747f,
0.998088369688552f,
0.9987764162142613f,
0.9993116726964553f,
0.9996940572530831f,
0.9999235113880169f,
1.0f,
};

#define SINE_LUT_LEN (sizeof(sin_lut_table) / sizeof(sin_lut_table[0]))
#define INTERVAL_LENGTH ((float)(M_PI_2 / (SINE_LUT_LEN - 1)))

float sin_lut(float x)
{
    if(x > (float)(2.f * M_PI)){
        x -= (float)(2 * M_PI);
    }
    if (x < 0)
    {
        x += (float)(2 * M_PI);
    }

    bool invert = false;
    if (x > (float)M_PI) {
        x -= (float)M_PI;
        invert = true;
    }

    if (x > (float)M_PI_2) {
        x = (float)M_PI - x;
    }

    uint16_t step = (uint16_t)(x / INTERVAL_LENGTH);
    float a = sin_lut_table[step];
    float b = sin_lut_table[step + 1];
    float k = (x / INTERVAL_LENGTH) - (float)step;

    float result = (1 - k) * a + k * b;

    if (invert)
    {
        return -result;
    }
    return result;
}

float cos_lut(float x)
{
    return sin_lut(x + (float)M_PI_4);
}