#include <stdio.h>
#include <math.h>
#include "../src/ui_theme.h"

static void unpack(uint16_t c, int *r, int *g, int *b) {
  *r = ((c >> 11) & 0x1F) * 255 / 31;
  *g = ((c >> 5) & 0x3F) * 255 / 63;
  *b = (c & 0x1F) * 255 / 31;
}

static float lin(int v) {
  float s = v / 255.0f;
  return s * s;
}

static float lum(uint16_t c) {
  int r, g, b;
  unpack(c, &r, &g, &b);
  return 0.2126f * lin(r) + 0.7152f * lin(g) + 0.0722f * lin(b);
}

static float contrast(uint16_t a, uint16_t b) {
  float la = lum(a), lb = lum(b);
  if (la < lb) { float t = la; la = lb; lb = t; }
  return (la + 0.05f) / (lb + 0.05f);
}

static int failures;

static void pair(const char *name, uint16_t fg, uint16_t bg, float min) {
  float c = contrast(fg, bg);
  if (c < min) {
    printf("  LOW %-22s %.2f (min %.1f)\n", name, c, min);
    failures++;
  }
}

int main(void) {
  uint16_t accents[8];
  accents[0] = ui565(0, 200, 255);
  accents[1] = ui565(70, 120, 255);
  accents[2] = ui565(170, 90, 255);
  accents[3] = ui565(255, 80, 170);
  accents[4] = ui565(255, 70, 70);
  accents[5] = ui565(255, 140, 30);
  accents[6] = ui565(255, 210, 40);
  accents[7] = ui565(60, 225, 120);
  const char *names[4] = {"TONAL", "BLACK", "SLATE", "LIGHT"};
  for (int m = 0; m < 4; m++) {
    for (int a = 0; a < 8; a++) {
      UiColors c = uiTheme((uint8_t)m, accents[a]);
      printf("%s accent %d: surf %04X low %04X cont %04X high %04X pri %04X onPri %04X\n",
             names[m], a, c.surface, c.surfaceLow, c.surfaceContainer, c.surfaceHigh,
             c.primary, c.onPrimary);
      pair("onSurface/surface", c.onSurface, c.surface, 4.5f);
      pair("onSurfaceVariant/surf", c.onSurfaceVariant, c.surface, 3.0f);
      pair("onSurface/low", c.onSurface, c.surfaceLow, 4.5f);
      pair("onPrimary/primary", c.onPrimary, c.primary, 3.5f);
      pair("onPriCont/priCont", c.onPrimaryContainer, c.primaryContainer, 4.5f);
      pair("onSecCont/secCont", c.onSecondaryContainer, c.secondaryContainer, 4.5f);
      pair("onErrCont/errCont", c.onErrorContainer, c.errorContainer, 4.5f);
      pair("onSurface/container", c.onSurface, c.surfaceContainer, 4.5f);
    }
  }
  if (failures) { printf("UI_THEME_CONTRAST_FAILED: %d\n", failures); return 1; }
  printf("UI_THEME_CONTRAST_OK\n");
  return 0;
}
