#include "xmb_ui.h"
#include <string.h>
#ifdef __CELLOS_LV2__
#include <sys/syscall.h>
#endif

/* --------------------------------------------------------------------------
 * Minimal 8x8 monochrome bitmap font (ASCII 0x20..0x7E)
 * -------------------------------------------------------------------------- */
static const uint8_t s_font8x8[95][8] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ' ' */
  {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, /* '!' */
  {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00}, /* '"' */
  {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00}, /* '#' */
  {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00}, /* '$' */
  {0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00}, /* '%' */
  {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00}, /* '&' */
  {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, /* ''' */
  {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, /* '(' */
  {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, /* ')' */
  {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, /* '*' */
  {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00}, /* '+' */
  {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x06}, /* ',' */
  {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00}, /* '-' */
  {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00}, /* '.' */
  {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00}, /* '/' */
  {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, /* '0' */
  {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, /* '1' */
  {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, /* '2' */
  {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, /* '3' */
  {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, /* '4' */
  {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, /* '5' */
  {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, /* '6' */
  {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, /* '7' */
  {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, /* '8' */
  {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, /* '9' */
  {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00}, /* ':' */
  {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x06}, /* ';' */
  {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00}, /* '<' */
  {0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00}, /* '=' */
  {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, /* '>' */
  {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00}, /* '?' */
  {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, /* '@' */
  {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00}, /* 'A' */
  {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00}, /* 'B' */
  {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00}, /* 'C' */
  {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00}, /* 'D' */
  {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00}, /* 'E' */
  {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00}, /* 'F' */
  {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00}, /* 'G' */
  {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00}, /* 'H' */
  {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, /* 'I' */
  {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00}, /* 'J' */
  {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00}, /* 'K' */
  {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00}, /* 'L' */
  {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00}, /* 'M' */
  {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00}, /* 'N' */
  {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00}, /* 'O' */
  {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00}, /* 'P' */
  {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00}, /* 'Q' */
  {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00}, /* 'R' */
  {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00}, /* 'S' */
  {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, /* 'T' */
  {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00}, /* 'U' */
  {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00}, /* 'V' */
  {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, /* 'W' */
  {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00}, /* 'X' */
  {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00}, /* 'Y' */
  {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00}, /* 'Z' */
  {0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00}, /* '[' */
  {0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00}, /* '\' */
  {0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00}, /* ']' */
  {0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00}, /* '^' */
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, /* '_' */
  {0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00}, /* '`' */
  {0x00,0x00,0x1E,0x30,0x3E,0x33,0x6E,0x00}, /* 'a' */
  {0x07,0x06,0x06,0x3E,0x66,0x66,0x3B,0x00}, /* 'b' */
  {0x00,0x00,0x1E,0x33,0x03,0x33,0x1E,0x00}, /* 'c' */
  {0x38,0x30,0x30,0x3E,0x33,0x33,0x6E,0x00}, /* 'd' */
  {0x00,0x00,0x1E,0x33,0x3F,0x03,0x1E,0x00}, /* 'e' */
  {0x1C,0x36,0x06,0x0F,0x06,0x06,0x0F,0x00}, /* 'f' */
  {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x1F}, /* 'g' */
  {0x07,0x06,0x36,0x6E,0x66,0x66,0x67,0x00}, /* 'h' */
  {0x0C,0x00,0x0E,0x0C,0x0C,0x0C,0x1E,0x00}, /* 'i' */
  {0x30,0x00,0x30,0x30,0x30,0x33,0x33,0x1E}, /* 'j' */
  {0x07,0x06,0x66,0x36,0x1E,0x36,0x67,0x00}, /* 'k' */
  {0x0E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, /* 'l' */
  {0x00,0x00,0x33,0x7F,0x7F,0x6B,0x63,0x00}, /* 'm' */
  {0x00,0x00,0x1F,0x33,0x33,0x33,0x33,0x00}, /* 'n' */
  {0x00,0x00,0x1E,0x33,0x33,0x33,0x1E,0x00}, /* 'o' */
  {0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x0F}, /* 'p' */
  {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x78}, /* 'q' */
  {0x00,0x00,0x3B,0x6E,0x66,0x06,0x0F,0x00}, /* 'r' */
  {0x00,0x00,0x3E,0x03,0x1E,0x30,0x1F,0x00}, /* 's' */
  {0x08,0x0C,0x3E,0x0C,0x0C,0x2C,0x18,0x00}, /* 't' */
  {0x00,0x00,0x33,0x33,0x33,0x33,0x6E,0x00}, /* 'u' */
  {0x00,0x00,0x33,0x33,0x33,0x1E,0x0C,0x00}, /* 'v' */
  {0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00}, /* 'w' */
  {0x00,0x00,0x63,0x36,0x1C,0x36,0x63,0x00}, /* 'x' */
  {0x00,0x00,0x33,0x33,0x33,0x3E,0x30,0x1F}, /* 'y' */
  {0x00,0x00,0x3F,0x19,0x0C,0x26,0x3F,0x00}, /* 'z' */
  {0x38,0x0C,0x0C,0x07,0x0C,0x0C,0x38,0x00}, /* '{' */
  {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, /* '|' */
  {0x07,0x0C,0x0C,0x38,0x0C,0x0C,0x07,0x00}, /* '}' */
  {0x6E,0x3B,0x00,0x00,0x00,0x00,0x00,0x00}  /* '~' */
};

typedef struct {
  uint8_t is_open;
  uint8_t dirty;
  int32_t selected;
  uint32_t prev_nav;
  uint32_t repeat_ticks;
  xpad_ui_config_t cfg;
  xmb_ui_callbacks_t cb;
} xmb_ui_state_t;

static xmb_ui_state_t s_ui;

#ifdef __CELLOS_LV2__
#define VSH_RSX_VRAM_BASE   0xC0000000UL
#define VSH_RSX_FB_OFF_ADDR 0x60201104UL

typedef uint32_t (*paf_get_dim_fn)(void);

static paf_get_dim_fn s_paf_get_disp_w = NULL;
static paf_get_dim_fn s_paf_get_disp_h = NULL;
static uint8_t s_paf_resolved = 0;
static uint8_t s_rsx_paused = 0;
static uint32_t s_refresh_ticks = 0;

static void rsx_fifo_pause(uint8_t pause) {
  /* lv2 sys_rsx_context_attribute(0x55555555, pause ? 2 : 3, 0, 0, 0, 0) */
  system_call_6(0x2A2, 0x55555555ULL, (uint64_t)(pause ? 2 : 3), 0, 0, 0, 0);
}
#endif

void xmb_ui_init(const xmb_ui_callbacks_t *cb) {
  memset(&s_ui, 0, sizeof(s_ui));
  s_ui.cfg.conn_mode = XPAD_CONN_BLUETOOTH;
  s_ui.cfg.switch_layout = 0;
  s_ui.cfg.gyro_mode = XPAD_GYRO_OFF;
  s_ui.cfg.swap_triggers = 0;
  s_ui.cfg.rumble_enabled = 1;
  if (cb != NULL) {
    s_ui.cb = *cb;
  }
}

void xmb_ui_toggle(void) {
  if (s_ui.is_open) {
    xmb_ui_close();
  } else {
    xmb_ui_open();
  }
}

void xmb_ui_open(void) {
  s_ui.is_open = 1;
  s_ui.selected = UI_ITEM_SWITCH_LAYOUT;
  s_ui.dirty = 1;
  s_ui.prev_nav = 0xFFFFFFFFU;
  s_ui.repeat_ticks = 0;
}

void xmb_ui_close(void) {
  s_ui.is_open = 0;
  s_ui.dirty = 0;
  s_ui.prev_nav = 0;
  s_ui.repeat_ticks = 0;
#ifdef __CELLOS_LV2__
  if (s_rsx_paused) {
    rsx_fifo_pause(0);
    s_rsx_paused = 0;
  }
#endif
}

uint8_t xmb_ui_is_open(void) {
  return s_ui.is_open;
}

int32_t xmb_ui_get_selected(void) {
  return s_ui.selected;
}

xpad_ui_config_t *xmb_ui_get_config(void) {
  return &s_ui.cfg;
}

void xmb_ui_set_config(const xpad_ui_config_t *cfg) {
  if (cfg != NULL) {
    s_ui.cfg = *cfg;
  }
}

static void ui_notify(const char *msg) {
  if (s_ui.cb.on_notify != NULL && msg != NULL) {
    s_ui.cb.on_notify(msg);
  }
}

static void ui_activate_item(int32_t idx, int32_t dir) {
  switch (idx) {
    case UI_ITEM_CONN_MODE: {
      s_ui.cfg.conn_mode = (s_ui.cfg.conn_mode == XPAD_CONN_BLUETOOTH)
                             ? XPAD_CONN_WIRED
                             : XPAD_CONN_BLUETOOTH;
      if (s_ui.cb.on_conn_mode_changed != NULL) {
        s_ui.cb.on_conn_mode_changed(s_ui.cfg.conn_mode);
      }
      ui_notify(s_ui.cfg.conn_mode == XPAD_CONN_BLUETOOTH
                  ? "XPAD: Connection -> BLUETOOTH"
                  : "XPAD: Connection -> WIRED USB");
      break;
    }

    case UI_ITEM_DONGLE_RESCAN: {
      if (dir == 0) {
        if (s_ui.cb.on_dongle_rescan != NULL) {
          s_ui.cb.on_dongle_rescan();
        }
      }
      break;
    }

    case UI_ITEM_SWITCH_LAYOUT: {
      s_ui.cfg.switch_layout = s_ui.cfg.switch_layout ? 0 : 1;
      ui_notify(s_ui.cfg.switch_layout
                  ? "XPAD: Button Layout -> SWITCH (A=Cross, B=Circle)"
                  : "XPAD: Button Layout -> PS3 POSITIONAL");
      break;
    }

    case UI_ITEM_GYRO_MODE: {
      if (dir < 0) {
        s_ui.cfg.gyro_mode = (uint8_t)((s_ui.cfg.gyro_mode + 2) % 3);
      } else {
        s_ui.cfg.gyro_mode = (uint8_t)((s_ui.cfg.gyro_mode + 1) % 3);
      }
      if (s_ui.cb.on_gyro_mode_changed != NULL) {
        s_ui.cb.on_gyro_mode_changed(s_ui.cfg.gyro_mode);
      }
      if (s_ui.cfg.gyro_mode == XPAD_GYRO_OFF) {
        ui_notify("XPAD: Gyro Mode -> OFF (SIXAXIS Only)");
      } else if (s_ui.cfg.gyro_mode == XPAD_GYRO_HOLD_ZL) {
        ui_notify("XPAD: Gyro Mode -> ON (Hold L2/ZL)");
      } else {
        ui_notify("XPAD: Gyro Mode -> ON (Always Active)");
      }
      break;
    }

    case UI_ITEM_SWAP_TRIGGERS: {
      s_ui.cfg.swap_triggers = s_ui.cfg.swap_triggers ? 0 : 1;
      ui_notify(s_ui.cfg.swap_triggers
                  ? "XPAD: Triggers -> SWAPPED (L1<->L2, R1<->R2)"
                  : "XPAD: Triggers -> NORMAL");
      break;
    }

    case UI_ITEM_RUMBLE: {
      s_ui.cfg.rumble_enabled = s_ui.cfg.rumble_enabled ? 0 : 1;
      if (s_ui.cb.on_rumble_changed != NULL) {
        s_ui.cb.on_rumble_changed(s_ui.cfg.rumble_enabled);
      }
      ui_notify(s_ui.cfg.rumble_enabled
                  ? "XPAD: Rumble -> ON (Test Pulse)"
                  : "XPAD: Rumble -> OFF");
      break;
    }

    case UI_ITEM_RECAL_GYRO: {
      if (dir == 0) {
        if (s_ui.cb.on_recalibrate_gyro != NULL) {
          s_ui.cb.on_recalibrate_gyro();
        }
        ui_notify("XPAD: Gyro Zero-Bias Recalibrated");
      }
      break;
    }

    case UI_ITEM_CLOSE: {
      if (dir == 0) {
        xmb_ui_close();
      }
      break;
    }

    default:
      break;
  }
}

void xmb_ui_handle_input(uint32_t nav_mask) {
  uint32_t edge;

  if (!s_ui.is_open) {
    return;
  }

  /* Clear latched guard bits once the user releases them */
  if (s_ui.prev_nav == 0xFFFFFFFFU) {
    if (nav_mask == 0) {
      s_ui.prev_nav = 0;
    }
    return;
  }

  edge = nav_mask & ~s_ui.prev_nav;
  if ((nav_mask & (UI_NAV_UP | UI_NAV_DOWN)) != 0 &&
      (nav_mask & (UI_NAV_UP | UI_NAV_DOWN)) == (s_ui.prev_nav & (UI_NAV_UP | UI_NAV_DOWN))) {
    s_ui.repeat_ticks++;
    if (s_ui.repeat_ticks >= 22 && (s_ui.repeat_ticks % 5) == 0) {
      edge |= (nav_mask & (UI_NAV_UP | UI_NAV_DOWN));
    }
  } else {
    s_ui.repeat_ticks = 0;
  }
  s_ui.prev_nav = nav_mask;

  if (edge != 0) {
    s_ui.dirty = 1;
  }

  if (edge & UI_NAV_BACK) {
    xmb_ui_close();
    return;
  }
  if (edge & UI_NAV_UP) {
    s_ui.selected = (s_ui.selected + UI_ITEM_COUNT - 1) % UI_ITEM_COUNT;
  }
  if (edge & UI_NAV_DOWN) {
    s_ui.selected = (s_ui.selected + 1) % UI_ITEM_COUNT;
  }
  if (edge & UI_NAV_LEFT) {
    ui_activate_item(s_ui.selected, -1);
  }
  if (edge & UI_NAV_RIGHT) {
    ui_activate_item(s_ui.selected, 1);
  }
  if (edge & UI_NAV_CONFIRM) {
    ui_activate_item(s_ui.selected, 0);
  }
}

/* --------------------------------------------------------------------------
 * Software ARGB8888 Drawing Primitives (Purple Terminal Theme)
 * -------------------------------------------------------------------------- */
static uint32_t blend_argb(uint32_t dst, uint32_t src) {
  uint32_t a = (src >> 24) & 0xFFU;
  uint32_t inv_a;
  uint32_t r, g, b;

  if (a == 0xFFU) return src;
  if (a == 0x00U) return dst;

  inv_a = 255U - a;
  r = ((((src >> 16) & 0xFFU) * a) + (((dst >> 16) & 0xFFU) * inv_a)) / 255U;
  g = ((((src >> 8) & 0xFFU) * a) + (((dst >> 8) & 0xFFU) * inv_a)) / 255U;
  b = (((src & 0xFFU) * a) + ((dst & 0xFFU) * inv_a)) / 255U;
  return 0xFF000000U | (r << 16) | (g << 8) | b;
}

static void fill_rect(uint32_t *fb, uint32_t fb_w, uint32_t fb_h, uint32_t stride_px,
                      int32_t x, int32_t y, int32_t w, int32_t h, uint32_t col) {
  int32_t x0 = x < 0 ? 0 : x;
  int32_t y0 = y < 0 ? 0 : y;
  int32_t x1 = (x + w > (int32_t)fb_w) ? (int32_t)fb_w : (x + w);
  int32_t y1 = (y + h > (int32_t)fb_h) ? (int32_t)fb_h : (y + h);
  int32_t px, py;

  if (x0 >= x1 || y0 >= y1) return;

  if ((col >> 24) == 0xFFU) {
    for (py = y0; py < y1; py++) {
      uint32_t *row = fb + (uint32_t)py * stride_px;
      for (px = x0; px < x1; px++) {
        row[px] = col;
      }
    }
    return;
  }

  for (py = y0; py < y1; py++) {
    uint32_t *row = fb + (uint32_t)py * stride_px;
    for (px = x0; px < x1; px++) {
      row[px] = blend_argb(row[px], col);
    }
  }
}

static void draw_char(uint32_t *fb, uint32_t fb_w, uint32_t fb_h, uint32_t stride_px,
                      int32_t x, int32_t y, char ch, uint32_t col, int32_t scale) {
  uint8_t idx;
  int32_t row, bit, sy, sx;

  if (ch < 0x20 || ch > 0x7E) {
    ch = '?';
  }
  idx = (uint8_t)(ch - 0x20);

  for (row = 0; row < 8; row++) {
    uint8_t bits = s_font8x8[idx][row];
    for (bit = 0; bit < 8; bit++) {
      if (bits & (1U << bit)) {
        for (sy = 0; sy < scale; sy++) {
          int32_t py = y + row * scale + sy;
          if (py < 0 || py >= (int32_t)fb_h) continue;
          for (sx = 0; sx < scale; sx++) {
            int32_t px = x + bit * scale + sx;
            if (px >= 0 && px < (int32_t)fb_w) {
              fb[(uint32_t)py * stride_px + (uint32_t)px] = col;
            }
          }
        }
      }
    }
  }
}

static void draw_text(uint32_t *fb, uint32_t fb_w, uint32_t fb_h, uint32_t stride_px,
                      int32_t x, int32_t y, const char *str, uint32_t col, int32_t scale) {
  int32_t adv = 8 * scale;
  while (str != NULL && *str != '\0') {
    draw_char(fb, fb_w, fb_h, stride_px, x, y, *str, col, scale);
    x += adv;
    str++;
  }
}

static void get_item_text(int32_t idx, const char **label, const char **val, uint32_t *val_col) {
  *val_col = UI_COL_VALUE;
  switch (idx) {
    case UI_ITEM_CONN_MODE:
      *label = "CONNECTION MODE";
      if (s_ui.cfg.conn_mode == XPAD_CONN_BLUETOOTH) {
        *val = "[BLUETOOTH]";
        *val_col = UI_COL_ACTIVE;
      } else {
        *val = "[WIRED USB]";
      }
      break;
    case UI_ITEM_DONGLE_RESCAN:
      *label = "RESCAN BT DONGLE";
      if (s_ui.cb.is_dongle_present != NULL && s_ui.cb.is_dongle_present()) {
        *val = "[SCAN NOW]";
        *val_col = UI_COL_WARN;
      } else {
        *val = "[NO DONGLE]";
        *val_col = UI_COL_MUTED;
      }
      break;
    case UI_ITEM_SWITCH_LAYOUT:
      *label = "BUTTON LAYOUT";
      if (s_ui.cfg.switch_layout) {
        *val = "[SWITCH A/B]";
        *val_col = UI_COL_ACTIVE;
      } else {
        *val = "[PS3 POSITIONAL]";
      }
      break;
    case UI_ITEM_GYRO_MODE:
      *label = "GYRO AIM MODE";
      if (s_ui.cfg.gyro_mode == XPAD_GYRO_OFF) {
        *val = "[OFF - SIXAXIS]";
      } else if (s_ui.cfg.gyro_mode == XPAD_GYRO_HOLD_ZL) {
        *val = "[HOLD L2 / ZL]";
        *val_col = UI_COL_ACTIVE;
      } else {
        *val = "[ALWAYS ACTIVE]";
        *val_col = UI_COL_ACTIVE;
      }
      break;
    case UI_ITEM_SWAP_TRIGGERS:
      *label = "SWAP L1/R1 <-> L2/R2";
      *val = s_ui.cfg.swap_triggers ? "[ON]" : "[OFF]";
      *val_col = s_ui.cfg.swap_triggers ? UI_COL_ACTIVE : UI_COL_VALUE;
      break;
    case UI_ITEM_RUMBLE:
      *label = "RUMBLE / VIBRATION";
      *val = s_ui.cfg.rumble_enabled ? "[ON]" : "[MUTED]";
      *val_col = s_ui.cfg.rumble_enabled ? UI_COL_ACTIVE : UI_COL_MUTED;
      break;
    case UI_ITEM_RECAL_GYRO:
      *label = "RECALIBRATE GYRO";
      *val = "[ZERO BIAS]";
      break;
    case UI_ITEM_CLOSE:
    default:
      *label = "CLOSE OVERLAY";
      *val = "[EXIT]";
      *val_col = UI_COL_MUTED;
      break;
  }
}

static uint32_t s_row_buf[720];

static void row_fill(uint32_t *row, int32_t win_w, int32_t row_y,
                     int32_t x, int32_t y, int32_t w, int32_t h, uint32_t col) {
  int32_t x0, x1, px;
  if (row_y < y || row_y >= y + h) return;
  x0 = x < 0 ? 0 : x;
  x1 = (x + w > win_w) ? win_w : (x + w);
  for (px = x0; px < x1; px++) {
    row[px] = col;
  }
}

static void row_text(uint32_t *row, int32_t win_w, int32_t row_y,
                     int32_t x, int32_t y, const char *str, uint32_t col, int32_t scale) {
  int32_t adv = 8 * scale;
  int32_t glyph_row;
  if (str == NULL || row_y < y || row_y >= y + (8 * scale)) return;
  glyph_row = (row_y - y) / scale;
  while (*str != '\0') {
    uint8_t ch = (uint8_t)*str;
    uint8_t bits;
    int32_t bit, sx;
    if (ch < 0x20 || ch > 0x7E) ch = '?';
    bits = s_font8x8[ch - 0x20][glyph_row];
    for (bit = 0; bit < 8; bit++) {
      if (bits & (1U << bit)) {
        int32_t px0 = x + bit * scale;
        for (sx = 0; sx < scale; sx++) {
          int32_t px = px0 + sx;
          if (px >= 0 && px < win_w) {
            row[px] = col;
          }
        }
      }
    }
    x += adv;
    str++;
  }
}

static void copy_row_to_fb(uint32_t *dst, const uint32_t *src, int32_t count) {
  int32_t i = 0;
  if ((((uintptr_t)dst & 7U) == 0) && (((uintptr_t)src & 7U) == 0)) {
    uint64_t *d64 = (uint64_t *)dst;
    const uint64_t *s64 = (const uint64_t *)src;
    int32_t n64 = count >> 1;
    for (i = 0; i < n64; i++) {
      d64[i] = s64[i];
    }
    i = n64 << 1;
  }
  for (; i < count; i++) {
    dst[i] = src[i];
  }
}

void xmb_ui_render_to_buffer(uint32_t *fb, uint32_t width, uint32_t height, uint32_t pitch_bytes) {
  int32_t scale, win_w, win_h, win_x, win_y;
  int32_t pad_x, header_h, row_h, status_h, footer_h, status_sep_h, status_gap_h;
  int32_t ry, i;
  uint32_t stride_px;
  const char *dongle_str = "NO DONGLE";
  uint32_t dongle_col;

  if (!s_ui.is_open || fb == NULL || width < 320 || height < 240 || pitch_bytes < width * 4U) {
    return;
  }

  stride_px = pitch_bytes / 4U;
  scale = (width >= 1000 && height >= 600) ? 2 : 1;

  pad_x = 12 * scale;
  header_h = (height <= 260) ? (22 * scale) : (28 * scale);
  row_h = (height <= 260) ? (16 * scale) : (20 * scale);
  status_h = (height <= 260) ? (16 * scale) : (24 * scale);
  footer_h = (height <= 260) ? (18 * scale) : (26 * scale);
  status_sep_h = (height <= 260) ? (14 * scale) : (20 * scale);
  status_gap_h = (height <= 260) ? (2 * scale) : (4 * scale);

  win_w = 340 * scale;
  win_h = header_h + status_h + (UI_ITEM_COUNT * row_h) + footer_h;
  if (win_w > (int32_t)width - 16) win_w = (int32_t)width - 16;
  if (win_w > 720) win_w = 720;
  if (win_h > (int32_t)height - 8) win_h = (int32_t)height - 8;

  /* Keep win_x even so 64-bit aligned VRAM writes work cleanly */
  win_x = (((int32_t)width - win_w) / 2) & ~1;
  win_y = ((int32_t)height - win_h) / 2;

  if (s_ui.cb.get_dongle_status != NULL) {
    const char *s = s_ui.cb.get_dongle_status();
    if (s != NULL) dongle_str = s;
  }
  dongle_col = (s_ui.cb.is_bt_connected != NULL && s_ui.cb.is_bt_connected()) ? UI_COL_ACTIVE : UI_COL_VALUE;

  /* Compose each scanline completely in RAM, then write atomically to framebuffer */
  for (ry = 0; ry < win_h; ry++) {
    int32_t y_cur = header_h;
    int32_t footer_y = win_h - ((height <= 260) ? (16 * scale) : (22 * scale));

    /* Base background or header fill */
    uint32_t bg = (ry < header_h) ? UI_COL_HEADER : UI_COL_BG;
    for (i = 0; i < win_w; i++) {
      s_row_buf[i] = bg;
    }

    /* Outer neon purple border (2px * scale) */
    row_fill(s_row_buf, win_w, ry, 0, 0, win_w, 2 * scale, UI_COL_BORDER);
    row_fill(s_row_buf, win_w, ry, 0, win_h - 2 * scale, win_w, 2 * scale, UI_COL_BORDER);
    row_fill(s_row_buf, win_w, ry, 0, 0, 2 * scale, win_h, UI_COL_BORDER);
    row_fill(s_row_buf, win_w, ry, win_w - 2 * scale, 0, 2 * scale, win_h, UI_COL_BORDER);

    /* Header title & divider */
    row_text(s_row_buf, win_w, ry, pad_x, 10 * scale,
             "XPAD://TERM_UI", UI_COL_TEXT, scale);
    row_fill(s_row_buf, win_w, ry, 2 * scale, y_cur, win_w - 4 * scale, scale, UI_COL_BORDER);

    /* Live status bar */
    row_text(s_row_buf, win_w, ry, pad_x, y_cur + 6 * scale, "BT DONGLE:", UI_COL_MUTED, scale);
    row_text(s_row_buf, win_w, ry, pad_x + (11 * 8 * scale), y_cur + 6 * scale,
             dongle_str, dongle_col, scale);
    y_cur += status_sep_h;
    row_fill(s_row_buf, win_w, ry, pad_x, y_cur, win_w - 2 * pad_x, scale, UI_COL_BORDER_INNER);
    y_cur += status_gap_h;

    /* Menu rows */
    for (i = 0; i < UI_ITEM_COUNT; i++) {
      if (ry >= y_cur && ry < y_cur + row_h) {
        const char *label = "";
        const char *val = "";
        uint32_t val_col = UI_COL_VALUE;
        int32_t val_len;
        int32_t val_x;

        get_item_text(i, &label, &val, &val_col);

        if (i == s_ui.selected) {
          row_fill(s_row_buf, win_w, ry, 4 * scale, y_cur, win_w - 8 * scale, row_h - 2 * scale, UI_COL_SEL_BG);
          row_fill(s_row_buf, win_w, ry, 4 * scale, y_cur, 3 * scale, row_h - 2 * scale, UI_COL_SEL_ACCENT);
          row_text(s_row_buf, win_w, ry, pad_x, y_cur + 5 * scale, ">", UI_COL_SEL_ACCENT, scale);
        }

        row_text(s_row_buf, win_w, ry, pad_x + 12 * scale, y_cur + 5 * scale, label, UI_COL_TEXT, scale);
        val_len = (int32_t)strlen(val);
        val_x = win_w - pad_x - (val_len * 8 * scale);
        row_text(s_row_buf, win_w, ry, val_x, y_cur + 5 * scale, val, val_col, scale);
      }
      y_cur += row_h;
    }

    /* Footer */
    row_fill(s_row_buf, win_w, ry, pad_x, footer_y, win_w - 2 * pad_x, scale, UI_COL_BORDER_INNER);
    row_text(s_row_buf, win_w, ry,
             pad_x, footer_y + ((height <= 260) ? (4 * scale) : (7 * scale)),
             "D-PAD:NAV  A/X:SELECT  B/STAR:CLOSE", UI_COL_MUTED, scale);

    copy_row_to_fb(&fb[(uint32_t)(win_y + ry) * stride_px + (uint32_t)win_x], s_row_buf, win_w);
  }
}

void xmb_ui_render_xmb(void) {
#ifdef __CELLOS_LV2__
  static const char *s_last_status = NULL;
  static int32_t s_last_bt_conn = -1;
  uint32_t w = 1280;
  uint32_t h = 720;
  uint32_t pitch;
  uint32_t fb_offset;
  uint32_t *fb;
  const char *cur_status;
  int32_t cur_bt_conn;

  if (!s_ui.is_open) {
    if (s_rsx_paused) {
      rsx_fifo_pause(0);
      s_rsx_paused = 0;
    }
    s_last_status = NULL;
    s_last_bt_conn = -1;
    return;
  }

  if (!s_rsx_paused) {
    rsx_fifo_pause(1);
    s_rsx_paused = 1;
    s_ui.dirty = 1;
    s_refresh_ticks = 0;
  }

  cur_status = (s_ui.cb.get_dongle_status != NULL) ? s_ui.cb.get_dongle_status() : NULL;
  cur_bt_conn = (s_ui.cb.is_bt_connected != NULL) ? s_ui.cb.is_bt_connected() : 0;

  if (!s_ui.dirty) {
    if (cur_status != s_last_status || cur_bt_conn != s_last_bt_conn) {
      s_ui.dirty = 1;
    } else {
      return;
    }
  }
  s_ui.dirty = 0;
  s_last_status = cur_status;
  s_last_bt_conn = cur_bt_conn;

  /* Lazy NID resolution from VSH's exported 'paf' module */
  if (!s_paf_resolved) {
    s_paf_resolved = 1;
    if (s_ui.cb.resolve_nid != NULL) {
      s_paf_get_disp_w = (paf_get_dim_fn)s_ui.cb.resolve_nid("paf", 0xF476E8AAU, 0);
      s_paf_get_disp_h = (paf_get_dim_fn)s_ui.cb.resolve_nid("paf", 0xAC984A12U, 0);
    }
  }

  if (s_paf_get_disp_w != NULL && s_paf_get_disp_h != NULL) {
    uint32_t rw = s_paf_get_disp_w();
    uint32_t rh = s_paf_get_disp_h();
    if (rw >= 320 && rw <= 1920 && rh >= 240 && rh <= 1080) {
      w = rw;
      h = rh;
    }
  }

  if (h == 1080) {
    pitch = 0x2000U; /* 8192 bytes */
  } else if (h == 720) {
    pitch = 0x1400U; /* 5120 bytes */
  } else if (h == 576 || h == 480) {
    pitch = 0x0C00U; /* 3072 bytes */
  } else {
    pitch = ((w * 4U) + 63U) & ~63U;
  }

  fb_offset = *(volatile uint32_t *)VSH_RSX_FB_OFF_ADDR;
  if (fb_offset >= 0x10000000U) {
    return;
  }

  fb = (uint32_t *)(VSH_RSX_VRAM_BASE + fb_offset);
  xmb_ui_render_to_buffer(fb, w, h, pitch);
#endif
}
