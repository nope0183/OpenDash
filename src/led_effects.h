/*
 * OpenDash ARGB effect engine.
 *
 * The renderer has no Arduino/NeoPixel dependency: the caller supplies a
 * LedfxSink that forwards the final RGB values to the strip, and a host test
 * (tools/test_led.cpp) can use a plain array instead.
 *
 * Design notes
 *  - every animation is a function of elapsed milliseconds, not of frame
 *    count, so a late frame does not make the animation stutter or jump;
 *  - moving effects (chase/comet/bounce/fill) use 16.16 fixed point, which
 *    gives sub-pixel motion instead of one-LED steps;
 *  - HSV goes through a 256-entry wheel and the global brightness is applied
 *    here, once; the Adafruit strip is left at full scale (setBrightness(255))
 *    so the library never rescales the buffer behind our back;
 *  - colour and brake transitions are eased, so changing a colour or hitting
 *    the brakes fades instead of snapping;
 *  - static effects report "settled", letting the caller skip show() until
 *    something actually changes.
 */
#ifndef OPENDASH_LED_EFFECTS_H
#define OPENDASH_LED_EFFECTS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LEDFX_SEGMENTS 3
#define LEDFX_MAX_PIXELS 120

enum {
  LEDFX_OFF = 0,
  LEDFX_SOLID,
  LEDFX_PULSE,
  LEDFX_RAINBOW,
  LEDFX_BLINK,
  LEDFX_BREATH,
  LEDFX_CHASE,
  LEDFX_COMET,
  LEDFX_TWINKLE,
  LEDFX_GRADIENT,
  LEDFX_BOUNCE,
  LEDFX_FILL,
  LEDFX_EFFECT_COUNT
};

enum {
  LEDFX_BRAKE_OFF = 0,
  LEDFX_BRAKE_ALL,
  LEDFX_BRAKE_REAR
};

typedef struct {
  uint16_t hue[LEDFX_SEGMENTS];   /* 0..359 */
  uint8_t sat[LEDFX_SEGMENTS];    /* 0..255 */
  uint8_t val[LEDFX_SEGMENTS];    /* 0..255, colour value */
  uint8_t count;                  /* number of addressed LEDs */
  uint8_t seg[LEDFX_SEGMENTS];    /* LEDs per segment, front to back */
  uint8_t effect[LEDFX_SEGMENTS];
  uint8_t bright;                 /* 1..255, global strip brightness */
  uint8_t brake;                  /* LEDFX_BRAKE_* */
} LedfxConfig;

typedef struct {
  void (*set)(void *ctx, uint16_t index, uint8_t r, uint8_t g, uint8_t b);
  void *ctx;
} LedfxSink;

typedef struct {
  uint8_t ready;
  uint8_t animating;
  uint8_t smoothing;
  uint8_t lastEffect[LEDFX_SEGMENTS];
  uint32_t effectSince[LEDFX_SEGMENTS];
  uint16_t curHue[LEDFX_SEGMENTS];
  uint8_t curSat[LEDFX_SEGMENTS];
  uint8_t curVal[LEDFX_SEGMENTS];
  uint8_t brakeMix[LEDFX_SEGMENTS];
} LedfxState;

/* Forget learned colours/phases; the next frame starts from the settings. */
void ledfx_reset(LedfxState *state);

/*
 * Render one frame.  Returns non-zero while the image keeps changing, zero
 * once it has settled (the caller may then skip show() until the settings
 * change).  `braking` is the debounced regen-brake flag.
 */
int ledfx_render(const LedfxSink *sink, const LedfxConfig *cfg,
                 LedfxState *state, uint32_t now_ms, int braking);

/* HSV (0..359, 0..255, 0..255) to RGB, using the same wheel as the effects. */
void ledfx_hsv_rgb(uint16_t hue, uint8_t sat, uint8_t val,
                   uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus
}
#endif

#endif /* OPENDASH_LED_EFFECTS_H */

/* ===================================================================== */
/*                            implementation                             */
/* ===================================================================== */

#ifdef LEDFX_IMPLEMENTATION
#ifndef LEDFX_IMPLEMENTED
#define LEDFX_IMPLEMENTED

#include <math.h>

static uint8_t ledfx_wheel[360][3];
static int8_t ledfx_sine[256];
static uint8_t ledfx_tables_ready;

static void ledfx_build_tables(void) {
  int i;
  if (ledfx_tables_ready) return;
  for (i = 0; i < 360; i++) {
    uint8_t sector = (uint8_t)(i / 60);
    uint8_t rem = (uint8_t)(i % 60);
    uint8_t up = (uint8_t)(((uint16_t)rem * 255) / 59);
    uint8_t down = (uint8_t)(255 - up);
    switch (sector) {
      case 0:  ledfx_wheel[i][0] = 255;  ledfx_wheel[i][1] = up;   ledfx_wheel[i][2] = 0;    break;
      case 1:  ledfx_wheel[i][0] = down; ledfx_wheel[i][1] = 255;  ledfx_wheel[i][2] = 0;    break;
      case 2:  ledfx_wheel[i][0] = 0;    ledfx_wheel[i][1] = 255;  ledfx_wheel[i][2] = up;   break;
      case 3:  ledfx_wheel[i][0] = 0;    ledfx_wheel[i][1] = down; ledfx_wheel[i][2] = 255;  break;
      case 4:  ledfx_wheel[i][0] = up;   ledfx_wheel[i][1] = 0;    ledfx_wheel[i][2] = 255;  break;
      default: ledfx_wheel[i][0] = 255;  ledfx_wheel[i][1] = 0;    ledfx_wheel[i][2] = down; break;
    }
  }
  for (i = 0; i < 256; i++)
    ledfx_sine[i] = (int8_t)lround(sin(6.283185307179586 * (double)i / 256.0) * 127.0);
  ledfx_tables_ready = 1;
}

/* 0..255 sine, phase given in 1/256 of a period */
static uint8_t ledfx_sin01(uint32_t phase) {
  return (uint8_t)((int16_t)ledfx_sine[phase & 255] + 128);
}

static uint8_t ledfx_mul8(uint8_t a, uint8_t b) {
  return (uint8_t)(((uint16_t)a * b) / 255);
}

static uint8_t ledfx_hue_index(uint16_t hue) {
  return (uint8_t)(hue % 360);
}

/* Colour at value 255, then desaturated towards white. */
static void ledfx_hsv_full(uint16_t hue, uint8_t sat, uint8_t *out) {
  const uint8_t *w = ledfx_wheel[ledfx_hue_index(hue)];
  out[0] = (uint8_t)(255 - ledfx_mul8((uint8_t)(255 - w[0]), sat));
  out[1] = (uint8_t)(255 - ledfx_mul8((uint8_t)(255 - w[1]), sat));
  out[2] = (uint8_t)(255 - ledfx_mul8((uint8_t)(255 - w[2]), sat));
}

void ledfx_hsv_rgb(uint16_t hue, uint8_t sat, uint8_t val,
                   uint8_t *r, uint8_t *g, uint8_t *b) {
  uint8_t full[3];
  ledfx_build_tables();
  ledfx_hsv_full(hue, sat, full);
  *r = ledfx_mul8(full[0], val);
  *g = ledfx_mul8(full[1], val);
  *b = ledfx_mul8(full[2], val);
}

void ledfx_reset(LedfxState *state) {
  uint8_t i;
  state->ready = 0;
  state->animating = 1;
  state->smoothing = 0;
  for (i = 0; i < LEDFX_SEGMENTS; i++) {
    state->lastEffect[i] = 0xFF;
    state->effectSince[i] = 0;
    state->curHue[i] = 0;
    state->curSat[i] = 0;
    state->curVal[i] = 0;
    state->brakeMix[i] = 0;
  }
}

static uint8_t ledfx_scale(uint8_t channel, uint8_t scale) {
  return (uint8_t)(((uint16_t)channel * scale) / 255);
}

static void ledfx_put(const LedfxSink *sink, uint16_t index,
                      uint8_t r, uint8_t g, uint8_t b,
                      uint8_t brakeMix, uint8_t bright) {
  if (brakeMix) {
    uint16_t inv = (uint16_t)(255 - brakeMix);
    r = (uint8_t)(((uint16_t)r * inv + (uint16_t)bright * brakeMix) / 255);
    g = (uint8_t)(((uint16_t)g * inv) / 255);
    b = (uint8_t)(((uint16_t)b * inv) / 255);
  }
  sink->set(sink->ctx, index, r, g, b);
}

/* Fade a scalar towards its target; returns 1 while it is still moving. */
static int ledfx_ease8(uint8_t *cur, uint8_t target) {
  int diff;
  if (*cur == target) return 0;
  diff = (int)target - (int)*cur;
  if (diff > 0) {
    diff = (diff + 2) / 3;
    *cur = (uint8_t)(*cur + diff);
    if (*cur > target) *cur = target;
  } else {
    diff = (-diff + 2) / 3;
    *cur = (uint8_t)(*cur - diff);
    if (*cur < target) *cur = target;
  }
  return 1;
}

/* Fade hue along the shortest arc. */
static int ledfx_ease_hue(uint16_t *cur, uint16_t target) {
  int diff;
  int next;
  if (*cur == target) return 0;
  diff = (int)target - (int)*cur;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  next = (int)*cur + diff / 3;
  if (next == (int)*cur) next += (diff > 0) ? 1 : -1;
  if (next < 0) next += 360;
  if (next >= 360) next -= 360;
  *cur = (uint16_t)next;
  return 1;
}

static uint8_t ledfx_sine_level(uint32_t t, uint32_t period,
                                uint8_t lo, uint8_t hi, uint16_t phase) {
  uint32_t p = (t + phase) % period;
  uint8_t s = ledfx_sin01((p * 256UL) / period);
  return (uint8_t)(lo + (uint16_t)(hi - lo) * s / 255);
}

static uint32_t ledfx_hash(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7FEB352Du;
  x ^= x >> 15;
  x *= 0x846CA68Bu;
  x ^= x >> 16;
  return x;
}

/* 16.16 head position, one LED per `ms_per_led` milliseconds. */
static uint32_t ledfx_head16(uint32_t t, uint16_t len, uint32_t ms_per_led) {
  uint64_t span = (uint64_t)len << 16;
  uint64_t p = ((uint64_t)t << 16) / ms_per_led;
  return (uint32_t)(p % span);
}

static void ledfx_segment(const LedfxSink *sink, const LedfxConfig *cfg,
                          uint8_t seg, uint16_t start, uint16_t end,
                          uint32_t t, uint16_t hue, uint8_t sat, uint8_t val,
                          uint8_t brakeMix) {
  uint16_t len = (uint16_t)(end - start);
  uint16_t i;
  uint8_t bright = cfg->bright ? cfg->bright : 1;
  uint8_t effect = cfg->effect[seg];
  uint8_t base[3];
  uint8_t colourScale;

  if (effect >= LEDFX_EFFECT_COUNT) effect = LEDFX_OFF;

  if (effect == LEDFX_RAINBOW) {
    uint32_t head = ((t % 4000UL) * 360UL) / 4000UL;
    uint16_t span = cfg->count ? cfg->count : 1;
    for (i = 0; i < len; i++) {
      uint16_t h = (uint16_t)((head + ((uint32_t)(start + i) * 360UL) / span +
                               (uint32_t)seg * 90UL + hue) % 360UL);
      uint8_t full[3];
      ledfx_hsv_full(h, sat, full);
      colourScale = (uint8_t)(((uint32_t)val * bright) / 255UL);
      ledfx_put(sink, (uint16_t)(start + i),
                ledfx_scale(full[0], colourScale),
                ledfx_scale(full[1], colourScale),
                ledfx_scale(full[2], colourScale), brakeMix, bright);
    }
    return;
  }

  if (effect == LEDFX_GRADIENT) {
    colourScale = (uint8_t)(((uint32_t)val * bright) / 255UL);
    for (i = 0; i < len; i++) {
      uint16_t h = (uint16_t)((hue + ((uint32_t)i * 120UL) / len) % 360UL);
      uint8_t full[3];
      ledfx_hsv_full(h, sat, full);
      ledfx_put(sink, (uint16_t)(start + i),
                ledfx_scale(full[0], colourScale),
                ledfx_scale(full[1], colourScale),
                ledfx_scale(full[2], colourScale), brakeMix, bright);
    }
    return;
  }

  ledfx_hsv_full(hue, sat, base);

  for (i = 0; i < len; i++) {
    uint32_t level = 0;
    uint16_t pixel = (uint16_t)(start + i);

    switch (effect) {
      case LEDFX_SOLID:
        level = 255;
        break;

      case LEDFX_PULSE:
        level = ledfx_sine_level(t, 1600UL, 40, 255, (uint16_t)(seg * 533));
        break;

      case LEDFX_BLINK: {
        uint32_t p = (t + (uint32_t)seg * 300UL) % 1000UL;
        level = (p < 500UL) ? 255 : 0;
        break;
      }

      case LEDFX_BREATH: {
        uint32_t p = (t + (uint32_t)seg * 1333UL) % 4000UL;
        uint8_t s = ledfx_sin01((p * 256UL) / 4000UL);
        s = ledfx_mul8(s, s);
        level = (uint32_t)(8 + (uint16_t)247 * s / 255);
        break;
      }

      case LEDFX_CHASE: {
        uint32_t span = (uint32_t)len << 16;
        uint32_t pos = ledfx_head16(t + (uint32_t)seg * 200UL, len, 55);
        uint32_t d = (((uint32_t)i << 16) + span - pos) % span;
        uint32_t tail = (uint32_t)(len > 8 ? 8 : (len > 1 ? len / 2 : 1)) << 16;
        if (d < tail) level = 255UL - (d * 255UL) / tail;
        break;
      }

      case LEDFX_COMET: {
        uint32_t span = (uint32_t)len << 16;
        uint32_t pos = ledfx_head16(t + (uint32_t)seg * 200UL, len, 70);
        uint32_t d = (((uint32_t)i << 16) + span - pos) % span;
        uint32_t tail = (uint32_t)(len > 24 ? 24 : (len > 1 ? len / 2 : 1)) << 16;
        if (d < tail) {
          uint32_t x = 255UL - (d * 255UL) / tail;
          level = (x * x) / 255UL;
        }
        break;
      }

      case LEDFX_TWINKLE: {
        uint32_t seed = ledfx_hash(((uint32_t)pixel + 1U) * 2654435761U +
                                   (uint32_t)seg * 97U);
        uint32_t period = 700UL + seed % 1700UL;
        uint32_t phase = (t + (seed >> 8)) % period;
        uint32_t flash = 120UL + ((seed >> 16) % 260UL);
        if (phase < flash) {
          uint32_t x = 255UL - (phase * 255UL) / flash;
          level = (x * x) / 255UL;
        }
        break;
      }

      case LEDFX_BOUNCE: {
        uint32_t span = ((uint32_t)(len > 1 ? len - 1 : 1)) << 16;
        uint32_t period = span * 2UL;
        uint32_t p = (uint32_t)((((uint64_t)(t + (uint32_t)seg * 300UL)) << 16) / 70UL) % period;
        uint32_t pos = (p < span) ? p : period - p;
        int32_t d = (int32_t)((uint32_t)i << 16) - (int32_t)pos;
        if (d < 0) d = -d;
        /* 1.5 LED wide ball so the bounce reads as movement, not flicker */
        if (d < 98304) level = 255UL - ((uint32_t)d * 255UL) / 98304UL;
        break;
      }

      case LEDFX_FILL: {
        uint32_t span = (uint32_t)len << 16;
        uint32_t half = 1600UL;
        uint32_t p = (t + (uint32_t)seg * 500UL) % (half * 2UL);
        uint32_t fill = (p < half)
                            ? (uint32_t)(((uint64_t)p * span) / half)
                            : (uint32_t)(((uint64_t)(half * 2UL - p) * span) / half);
        int32_t d = (int32_t)fill - (int32_t)((uint32_t)i << 16);
        if (d >= 65536) level = 255;
        else if (d > 0) level = ((uint32_t)d * 255UL) / 65536UL;
        break;
      }

      default:
        level = 0;
        break;
    }

    if (level) {
      uint8_t scale = (uint8_t)(((uint32_t)val * level * bright) / 65025UL);
      ledfx_put(sink, pixel,
                ledfx_scale(base[0], scale),
                ledfx_scale(base[1], scale),
                ledfx_scale(base[2], scale), brakeMix, bright);
    } else {
      ledfx_put(sink, pixel, 0, 0, 0, brakeMix, bright);
    }
  }
}

int ledfx_render(const LedfxSink *sink, const LedfxConfig *cfg,
                 LedfxState *state, uint32_t now_ms, int braking) {
  uint8_t seg;
  uint8_t count;
  uint16_t start;
  uint16_t covered;
  uint8_t dynamic = 0;
  uint8_t brakeSeg = 0xFF;

  ledfx_build_tables();
  state->smoothing = 0;

  count = cfg->count;
  if (count > LEDFX_MAX_PIXELS) count = LEDFX_MAX_PIXELS;

  {
    uint16_t pos = 0;
    for (seg = 0; seg < LEDFX_SEGMENTS; seg++) {
      uint8_t effect = cfg->effect[seg];
      uint16_t len = cfg->seg[seg];
      if (len && pos < count &&
          effect != LEDFX_OFF && effect != LEDFX_SOLID && effect != LEDFX_GRADIENT)
        dynamic = 1;
      if (braking && cfg->brake == LEDFX_BRAKE_REAR && len && pos < count)
        brakeSeg = seg;
      pos = (uint16_t)(pos + len);
    }
  }

  if (!state->ready) {
    for (seg = 0; seg < LEDFX_SEGMENTS; seg++) {
      state->curHue[seg] = (uint16_t)(cfg->hue[seg] % 360);
      state->curSat[seg] = cfg->sat[seg];
      state->curVal[seg] = cfg->val[seg];
      state->lastEffect[seg] = cfg->effect[seg];
      state->effectSince[seg] = now_ms;
      state->brakeMix[seg] = 0;
    }
    state->ready = 1;
  }

  start = 0;
  covered = 0;
  for (seg = 0; seg < LEDFX_SEGMENTS; seg++) {
    uint16_t len = cfg->seg[seg];
    uint16_t end = (uint16_t)(start + len);
    uint8_t mixTarget = 0;
    uint8_t mix;

    if (state->lastEffect[seg] != cfg->effect[seg]) {
      state->lastEffect[seg] = cfg->effect[seg];
      state->effectSince[seg] = now_ms;
    }
    if (ledfx_ease_hue(&state->curHue[seg], (uint16_t)(cfg->hue[seg] % 360))) state->smoothing = 1;
    if (ledfx_ease8(&state->curSat[seg], cfg->sat[seg])) state->smoothing = 1;
    if (ledfx_ease8(&state->curVal[seg], cfg->val[seg])) state->smoothing = 1;

    if (braking && cfg->brake == LEDFX_BRAKE_ALL) mixTarget = 255;
    if (braking && cfg->brake == LEDFX_BRAKE_REAR && seg == brakeSeg) mixTarget = 255;
    mix = state->brakeMix[seg];
    if (ledfx_ease8(&mix, mixTarget)) state->smoothing = 1;
    state->brakeMix[seg] = mix;

    if (start < count) {
      if (end > count) end = count;
      if (end > start) {
        uint32_t t = (uint32_t)(now_ms - state->effectSince[seg]) + (uint32_t)seg * 137UL;
        ledfx_segment(sink, cfg, seg, start, end, t,
                      state->curHue[seg], state->curSat[seg], state->curVal[seg], mix);
        covered = end;
      }
    }
    start = (uint16_t)(start + len);
  }

  while (covered < count) {
    ledfx_put(sink, covered, 0, 0, 0, 0, cfg->bright ? cfg->bright : 1);
    covered++;
  }

  state->animating = (uint8_t)(dynamic || state->smoothing);
  return state->animating;
}

#endif /* LEDFX_IMPLEMENTATION */
#endif /* LEDFX_IMPLEMENTED */
