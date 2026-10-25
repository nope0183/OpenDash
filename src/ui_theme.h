/*
 * Material You (Material 3) colour tokens for OpenDash.
 *
 * The palette is generated from the selected accent hue, so the eight presets
 * and the custom hue slider all produce a coherent tonal scheme instead of a
 * fixed "panel/accent" pair.  Everything is RGB565 to match TFT_eSPI.
 *
 * This header deliberately has no Arduino dependency; it only needs
 * <math.h> and <stdint.h>.
 */
#ifndef OPENDASH_UI_THEME_H
#define OPENDASH_UI_THEME_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint16_t surface;
  uint16_t surfaceLow;
  uint16_t surfaceContainer;
  uint16_t surfaceHigh;
  uint16_t surfaceHighest;
  uint16_t onSurface;
  uint16_t onSurfaceVariant;
  uint16_t outline;
  uint16_t outlineVariant;
  uint16_t primary;
  uint16_t onPrimary;
  uint16_t primaryContainer;
  uint16_t onPrimaryContainer;
  uint16_t secondaryContainer;
  uint16_t onSecondaryContainer;
  uint16_t tertiary;
  uint16_t tertiaryContainer;
  uint16_t error;
  uint16_t onError;
  uint16_t errorContainer;
  uint16_t onErrorContainer;
  uint16_t success;
  uint16_t warning;
  uint8_t dark;
} UiColors;

static inline uint16_t ui565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static inline uint8_t ui_clamp8(float v) {
  if (v <= 0.0f) return 0;
  if (v >= 255.0f) return 255;
  return (uint8_t)(v + 0.5f);
}

static inline float ui_absf(float v) { return v < 0.0f ? -v : v; }

/* HSL (h in degrees, s/l in 0..1) to RGB565. */
static inline uint16_t uiHsl(float h, float s, float l) {
  float c, x, m, r, g, b;
  int sector;
  float frac;
  h = h - 360.0f * (float)(int)(h / 360.0f);
  if (h < 0.0f) h += 360.0f;
  c = (1.0f - ui_absf(2.0f * l - 1.0f)) * s;
  sector = (int)(h / 60.0f);
  if (sector > 5) sector = 5;
  frac = h / 60.0f - (float)sector;
  x = c * ((sector & 1) ? (1.0f - frac) : frac);
  if (sector == 0)      { r = c; g = x; b = 0; }
  else if (sector == 1) { r = x; g = c; b = 0; }
  else if (sector == 2) { r = 0; g = c; b = x; }
  else if (sector == 3) { r = 0; g = x; b = c; }
  else if (sector == 4) { r = x; g = 0; b = c; }
  else                  { r = c; g = 0; b = x; }
  m = l - c * 0.5f;
  return ui565(ui_clamp8((r + m) * 255.0f),
               ui_clamp8((g + m) * 255.0f),
               ui_clamp8((b + m) * 255.0f));
}

static inline void uiHueOf(uint16_t c, float *h) {
  int r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, b = c & 0x1F;
  float rf = r * 255.0f / 31.0f, gf = g * 255.0f / 63.0f, bf = b * 255.0f / 31.0f;
  float mx = rf > gf ? (rf > bf ? rf : bf) : (gf > bf ? gf : bf);
  float mn = rf < gf ? (rf < bf ? rf : bf) : (gf < bf ? gf : bf);
  float d = mx - mn;
  float hh;
  if (d < 1.0f) { *h = 200.0f; return; }
  if (mx == rf) {
    hh = (gf - bf) / d;
    hh = 60.0f * (hh - 6.0f * (float)(int)(hh / 6.0f));
  } else if (mx == gf) {
    hh = 60.0f * ((bf - rf) / d + 2.0f);
  } else {
    hh = 60.0f * ((rf - gf) / d + 4.0f);
  }
  if (hh < 0.0f) hh += 360.0f;
  *h = hh;
}

/* Dark or light ink for an arbitrary fill (used for chips/swatches). */
static inline uint16_t uiInkOn(uint16_t fill) {
  int r = ((fill >> 11) & 0x1F) * 255 / 31;
  int g = ((fill >> 5) & 0x3F) * 255 / 63;
  int b = (fill & 0x1F) * 255 / 31;
  int lum = (r * 299 + g * 587 + b * 114) / 1000;
  return lum > 90 ? ui565(0, 0, 0) : ui565(250, 250, 252);
}

/*
 * mode: 0 tonal dark, 1 black, 2 slate, 3 light.
 * accent: RGB565 accent; the caller resolves a custom hue to RGB565 first.
 */
static inline UiColors uiTheme(uint8_t mode, uint16_t accent) {
  UiColors c;
  float h = 200.0f;
  float chroma, baseL;
  bool light;

  uiHueOf(accent, &h);
  if (mode > 3) mode = 0;
  light = (mode == 3);

  if (light) {
    baseL = 0.985f;
    chroma = 0.22f;
    c.surface        = uiHsl(h, 0.28f, baseL);
    c.surfaceLow     = uiHsl(h, 0.26f, 0.962f);
    c.surfaceContainer = uiHsl(h, 0.24f, 0.938f);
    c.surfaceHigh    = uiHsl(h, 0.22f, 0.908f);
    c.surfaceHighest = uiHsl(h, 0.20f, 0.872f);
    c.onSurface      = uiHsl(h, 0.28f, 0.105f);
    c.onSurfaceVariant = uiHsl(h, 0.14f, 0.34f);
    c.outline        = uiHsl(h, 0.10f, 0.50f);
    c.outlineVariant = uiHsl(h, 0.10f, 0.80f);
    c.primary        = uiHsl(h, 0.62f, 0.36f);
    c.onPrimary      = uiInkOn(c.primary);
    c.primaryContainer = uiHsl(h, 0.72f, 0.88f);
    c.onPrimaryContainer = uiHsl(h, 0.55f, 0.16f);
    c.secondaryContainer = uiHsl(h, 0.34f, 0.92f);
    c.onSecondaryContainer = uiHsl(h, 0.40f, 0.18f);
    c.tertiary       = uiHsl(h + 60.0f, 0.48f, 0.40f);
    c.tertiaryContainer = uiHsl(h + 60.0f, 0.55f, 0.90f);
    c.error          = uiHsl(0.0f, 0.68f, 0.44f);
    c.onError        = ui565(255, 255, 255);
    c.errorContainer = uiHsl(0.0f, 0.80f, 0.92f);
    c.onErrorContainer = uiHsl(0.0f, 0.55f, 0.18f);
    c.success        = uiHsl(145.0f, 0.55f, 0.34f);
    c.warning        = uiHsl(40.0f, 0.90f, 0.38f);
    c.dark = 0;
    return c;
  }

  if (mode == 0)      { baseL = 0.065f; chroma = 0.20f; }  /* tonal navy   */
  else if (mode == 1) { baseL = 0.000f; chroma = 0.12f; }  /* black         */
  else                { baseL = 0.150f; chroma = 0.055f; } /* slate / dim   */

  c.surface        = uiHsl(h, chroma, baseL);
  c.surfaceLow     = uiHsl(h, chroma, baseL + 0.030f);
  c.surfaceContainer = uiHsl(h, chroma, baseL + 0.060f);
  c.surfaceHigh    = uiHsl(h, chroma, baseL + 0.100f);
  c.surfaceHighest = uiHsl(h, chroma, baseL + 0.140f);
  c.onSurface      = uiHsl(h, 0.10f, 0.94f);
  c.onSurfaceVariant = uiHsl(h, 0.12f, 0.76f);
  c.outline        = uiHsl(h, 0.09f, 0.48f);
  c.outlineVariant = uiHsl(h, 0.09f, 0.30f);
  c.primary        = uiHsl(h, 0.68f, 0.78f);
  c.onPrimary      = uiHsl(h, 0.55f, 0.12f);
  c.primaryContainer = uiHsl(h, 0.48f, 0.30f);
  c.onPrimaryContainer = uiHsl(h, 0.80f, 0.92f);
  c.secondaryContainer = uiHsl(h, 0.22f, 0.26f);
  c.onSecondaryContainer = uiHsl(h, 0.38f, 0.90f);
  c.tertiary       = uiHsl(h + 60.0f, 0.45f, 0.78f);
  c.tertiaryContainer = uiHsl(h + 60.0f, 0.32f, 0.28f);
  c.error          = uiHsl(0.0f, 0.78f, 0.78f);
  c.onError        = uiHsl(0.0f, 0.60f, 0.10f);
  c.errorContainer = uiHsl(0.0f, 0.45f, 0.30f);
  c.onErrorContainer = uiHsl(0.0f, 0.80f, 0.92f);
  c.success        = uiHsl(145.0f, 0.52f, 0.64f);
  c.warning        = uiHsl(45.0f, 0.90f, 0.66f);
  c.dark = 1;
  return c;
}

#endif /* OPENDASH_UI_THEME_H */
