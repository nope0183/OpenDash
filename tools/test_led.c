/*
 * Host test for src/led_effects.h (build: tools/test_led.c, see README).
 *
 *   tcc -run test_led.c        (TinyCC)
 *   cc -O2 test_led.c -o test_led -lm && ./test_led
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define LEDFX_IMPLEMENTATION
#include "../src/led_effects.h"

typedef struct {
  uint8_t r[LEDFX_MAX_PIXELS];
  uint8_t g[LEDFX_MAX_PIXELS];
  uint8_t b[LEDFX_MAX_PIXELS];
  uint16_t max_index;
  uint32_t writes;
} Buffer;

static void buffer_set(void *ctx, uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
  Buffer *buf = (Buffer *)ctx;
  if (i >= LEDFX_MAX_PIXELS) {
    fprintf(stderr, "FATAL: pixel %u out of range\n", (unsigned)i);
    exit(2);
  }
  buf->r[i] = r;
  buf->g[i] = g;
  buf->b[i] = b;
  if (i > buf->max_index) buf->max_index = i;
  buf->writes++;
}

static void buffer_clear(Buffer *b) {
  memset(b, 0, sizeof(*b));
}

static int failures;

static void check(int cond, const char *what) {
  if (!cond) {
    printf("FAIL: %s\n", what);
    failures++;
  }
}

static void check_near(int got, int want, int tol, const char *what) {
  if (abs(got - want) > tol) {
    printf("FAIL: %s (got %d, want %d +-%d)\n", what, got, want, tol);
    failures++;
  }
}

static LedfxConfig make_cfg(void) {
  LedfxConfig c;
  memset(&c, 0, sizeof(c));
  c.count = 30;
  c.seg[0] = 10;
  c.seg[1] = 10;
  c.seg[2] = 10;
  c.effect[0] = c.effect[1] = c.effect[2] = LEDFX_SOLID;
  c.hue[0] = c.hue[1] = c.hue[2] = 0;
  c.sat[0] = c.sat[1] = c.sat[2] = 255;
  c.val[0] = c.val[1] = c.val[2] = 255;
  c.bright = 255;
  c.brake = LEDFX_BRAKE_OFF;
  return c;
}

static void test_hsv(void) {
  uint8_t r, g, b;
  ledfx_hsv_rgb(0, 255, 255, &r, &g, &b);
  check_near(r, 255, 1, "hsv red r"); check_near(g, 0, 1, "hsv red g"); check_near(b, 0, 1, "hsv red b");
  ledfx_hsv_rgb(120, 255, 255, &r, &g, &b);
  check_near(r, 0, 1, "hsv green r"); check_near(g, 255, 1, "hsv green g"); check_near(b, 0, 1, "hsv green b");
  ledfx_hsv_rgb(240, 255, 255, &r, &g, &b);
  check_near(r, 0, 1, "hsv blue r"); check_near(g, 0, 1, "hsv blue g"); check_near(b, 255, 1, "hsv blue b");
  ledfx_hsv_rgb(60, 255, 255, &r, &g, &b);
  check_near(r, 255, 1, "hsv yellow r"); check_near(g, 255, 1, "hsv yellow g"); check_near(b, 0, 1, "hsv yellow b");
  ledfx_hsv_rgb(0, 0, 128, &r, &g, &b);
  check_near(r, 128, 1, "hsv grey r"); check_near(g, 128, 1, "hsv grey g"); check_near(b, 128, 1, "hsv grey b");
}

static void test_bounds_and_brightness(void) {
  LedfxConfig c = make_cfg();
  LedfxState s;
  LedfxSink sink;
  Buffer buf;
  int i;

  memset(&s, 0, sizeof(s));
  ledfx_reset(&s);
  buffer_clear(&buf);
  sink.set = buffer_set;
  sink.ctx = &buf;
  c.count = 5;
  c.seg[0] = 10; c.seg[1] = 10; c.seg[2] = 10;
  ledfx_render(&sink, &c, &s, 0, 0);
  check(buf.max_index == 4, "segments clipped to count");
  for (i = 0; i < 5; i++) check_near(buf.r[i], 255, 1, "clipped solid red");
  check_near(buf.r[5], 0, 0, "pixel past count untouched");

  c = make_cfg();
  c.bright = 128;
  ledfx_reset(&s);
  buffer_clear(&buf);
  ledfx_render(&sink, &c, &s, 0, 0);
  check_near(buf.r[0], 128, 2, "global brightness 128");
  check_near(buf.g[0], 0, 1, "brightness keeps hue");

  c.bright = 1;
  ledfx_reset(&s);
  buffer_clear(&buf);
  ledfx_render(&sink, &c, &s, 0, 0);
  check_near(buf.r[0], 1, 1, "global brightness 1");
}

static void test_static_settles(void) {
  LedfxConfig c = make_cfg();
  LedfxState s;
  LedfxSink sink;
  Buffer buf;

  memset(&s, 0, sizeof(s));
  ledfx_reset(&s);
  buffer_clear(&buf);
  sink.set = buffer_set; sink.ctx = &buf;
  check(ledfx_render(&sink, &c, &s, 0, 0) == 0, "solid settles on first frame");
  check(ledfx_render(&sink, &c, &s, 20, 0) == 0, "solid stays settled");
  c.effect[0] = LEDFX_GRADIENT;
  check(ledfx_render(&sink, &c, &s, 40, 0) == 0, "gradient settles");
  c.effect[0] = LEDFX_PULSE;
  check(ledfx_render(&sink, &c, &s, 60, 0) != 0, "pulse animates");
}

static void test_effects(void) {
  const char *names[] = {"OFF", "SOLID", "PULSE", "RAINBOW", "BLINK", "BREATH",
                         "CHASE", "COMET", "TWINKLE", "GRADIENT", "BOUNCE", "FILL"};
  int fx;
  for (fx = LEDFX_OFF; fx < LEDFX_EFFECT_COUNT; fx++) {
    LedfxConfig c = make_cfg();
    LedfxState s;
    LedfxSink sink;
    Buffer buf;
    uint32_t lit = 0, total = 0, changes = 0;
    uint8_t prev[LEDFX_MAX_PIXELS][3];
    int frame;
    memset(&s, 0, sizeof(s));
    ledfx_reset(&s);
    sink.set = buffer_set; sink.ctx = &buf;
    c.count = 120;
    c.seg[0] = c.seg[1] = c.seg[2] = 40;
    c.effect[0] = c.effect[1] = c.effect[2] = (uint8_t)fx;
    c.hue[0] = 0; c.hue[1] = 120; c.hue[2] = 240;
    memset(prev, 0, sizeof(prev));
    for (frame = 0; frame < 120; frame++) {
      uint32_t now = (uint32_t)frame * 20;
      int i;
      buffer_clear(&buf);
      ledfx_render(&sink, &c, &s, now, 0);
      for (i = 0; i < 120; i++) {
        total++;
        if (buf.r[i] || buf.g[i] || buf.b[i]) lit++;
        if (memcmp(&prev[i][0], &buf.r[i], 3) != 0) changes = 1;
        memcpy(&prev[i][0], &buf.r[i], 3);
      }
      check(buf.max_index < 120, "effect pixel in range");
    }
    if (fx == LEDFX_OFF) check(lit == 0, "OFF produces black");
    else check(lit > 0, "effect produces light");
    if (fx != LEDFX_OFF && fx != LEDFX_SOLID && fx != LEDFX_GRADIENT)
      check(changes != 0, "animated effect changes frames");
    printf("  %-9s lit=%3u%%\n", names[fx], (unsigned)(lit * 100 / total));
  }
}

static void test_brake(void) {
  LedfxConfig c = make_cfg();
  LedfxState s;
  LedfxSink sink;
  Buffer buf;
  int i, frame;

  memset(&s, 0, sizeof(s));
  ledfx_reset(&s);
  sink.set = buffer_set; sink.ctx = &buf;

  c.effect[0] = c.effect[1] = c.effect[2] = LEDFX_SOLID;
  c.hue[0] = c.hue[1] = c.hue[2] = 120; /* green */
  c.brake = LEDFX_BRAKE_ALL;
  for (frame = 0; frame < 80; frame++) {
    buffer_clear(&buf);
    ledfx_render(&sink, &c, &s, (uint32_t)frame * 20, 1);
  }
  for (i = 0; i < 30; i++) {
    check_near(buf.r[i], 255, 2, "brake all red");
    check_near(buf.g[i], 0, 2, "brake all no green");
  }

  /* release the brake and let the effect come back */
  for (frame = 0; frame < 80; frame++) {
    buffer_clear(&buf);
    ledfx_render(&sink, &c, &s, 2000 + (uint32_t)frame * 20, 0);
  }
  check_near(buf.g[0], 255, 3, "effect returns after brake");

  /* rear segment only */
  c.brake = LEDFX_BRAKE_REAR;
  for (frame = 0; frame < 80; frame++) {
    buffer_clear(&buf);
    ledfx_render(&sink, &c, &s, 4000 + (uint32_t)frame * 20, 1);
  }
  check_near(buf.g[0], 255, 3, "front segment keeps colour");
  check_near(buf.r[25], 255, 3, "rear segment red");
  check_near(buf.g[25], 0, 3, "rear segment no green");
}

static void test_colour_fade(void) {
  LedfxConfig c = make_cfg();
  LedfxState s;
  LedfxSink sink;
  Buffer buf;
  int frame;
  uint8_t last = 0;

  memset(&s, 0, sizeof(s));
  ledfx_reset(&s);
  sink.set = buffer_set; sink.ctx = &buf;
  c.effect[0] = c.effect[1] = c.effect[2] = LEDFX_SOLID;
  for (frame = 0; frame < 60; frame++) ledfx_render(&sink, &c, &s, 0, 0);

  c.hue[0] = c.hue[1] = c.hue[2] = 120;
  for (frame = 0; frame < 60; frame++) {
    buffer_clear(&buf);
    ledfx_render(&sink, &c, &s, 1000 + (uint32_t)frame * 20, 0);
    check_near(buf.b[0], 0, 2, "fade keeps blue at zero");
    if (frame > 0) check(abs((int)buf.g[0] - (int)last) <= 90, "fade step is smooth");
    last = buf.g[0];
  }
  check_near(buf.g[0], 255, 3, "fade reaches target");
}

static void test_motion_smooth(void) {
  LedfxConfig c = make_cfg();
  LedfxState s;
  LedfxSink sink;
  Buffer buf;
  int frame, i;
  int two_lit = 0, frames = 0;

  memset(&s, 0, sizeof(s));
  ledfx_reset(&s);
  sink.set = buffer_set; sink.ctx = &buf;
  c.count = 30;
  c.seg[0] = 30;
  c.seg[1] = c.seg[2] = 0;
  c.effect[0] = LEDFX_BOUNCE;

  /* The old renderer lit exactly one LED per frame.  The new one moves a
     1.5-LED ball in 16.16 space, so most frames have two or three LEDs
     partially lit; that is what makes the bounce look continuous. */
  for (frame = 0; frame < 400; frame++) {
    int lit = 0;
    buffer_clear(&buf);
    ledfx_render(&sink, &c, &s, 10000 + (uint32_t)frame * 20, 0);
    for (i = 0; i < 30; i++) if (buf.r[i] > 8) lit++;
    if (lit >= 2) two_lit++;
    frames++;
  }
  check(two_lit * 100 > frames * 60, "bounce uses sub-pixel positions");
}

static void test_perf(void) {
  LedfxConfig c = make_cfg();
  LedfxState s;
  LedfxSink sink;
  Buffer buf;
  int frame;
  clock_t t0;
  double ms;

  memset(&s, 0, sizeof(s));
  ledfx_reset(&s);
  sink.set = buffer_set; sink.ctx = &buf;
  c.count = 120;
  c.seg[0] = c.seg[1] = c.seg[2] = 40;
  c.effect[0] = LEDFX_RAINBOW;
  c.effect[1] = LEDFX_COMET;
  c.effect[2] = LEDFX_TWINKLE;
  t0 = clock();
  for (frame = 0; frame < 5000; frame++) {
    buffer_clear(&buf);
    ledfx_render(&sink, &c, &s, (uint32_t)frame * 20, 0);
  }
  ms = 1000.0 * (double)(clock() - t0) / (double)CLOCKS_PER_SEC;
  printf("  perf: %.3f ms/frame (120 LEDs, 3 animated segments)\n", ms / 5000.0);
  check(ms / 5000.0 < 50.0, "frame render below 50 ms even on a slow host");
}

int main(void) {
  printf("LED effect tests\n");
  test_hsv();
  test_bounds_and_brightness();
  test_static_settles();
  test_effects();
  test_brake();
  test_colour_fade();
  test_motion_smooth();
  test_perf();
  if (failures) {
    printf("LED_TESTS_FAILED: %d\n", failures);
    return 1;
  }
  printf("LED_TESTS_OK\n");
  return 0;
}
