#ifndef _XMB_UI_H_
#define _XMB_UI_H_

#include <stdint.h>

/* Connection mode constants */
#define XPAD_CONN_BLUETOOTH 0
#define XPAD_CONN_WIRED     1

/* Gyro aiming mode constants */
#define XPAD_GYRO_OFF       0
#define XPAD_GYRO_HOLD_ZL   1
#define XPAD_GYRO_ALWAYS    2

/* Purple Terminal Color Palette (ARGB8888) */
#define UI_COL_BG           0xFF140A22U /* Opaque deep dark violet (avoids uncached VRAM reads) */
#define UI_COL_HEADER       0xFF24103DU /* Rich dark purple header */
#define UI_COL_BORDER       0xFF9D4EDDU /* Neon purple border */
#define UI_COL_BORDER_INNER 0xFF5A189AU /* Subtle inner divider */
#define UI_COL_SEL_BG       0xFF5A189AU /* Active row highlight */
#define UI_COL_SEL_ACCENT   0xFFC77DFFU /* Left cursor accent bar */
#define UI_COL_TEXT         0xFFF3E8FFU /* Lavender-white primary text */
#define UI_COL_VALUE        0xFFE0AAFFU /* Bright violet-pink value text */
#define UI_COL_ACTIVE       0xFF80FFDBU /* Mint-cyan active badge */
#define UI_COL_MUTED        0xFF9D84B7U /* Muted purple-gray footer/hint */
#define UI_COL_WARN         0xFFFF9E00U /* Amber warning/scanning badge */

/* Navigation bitmask passed from controller polling */
#define UI_NAV_UP           (1U << 0)
#define UI_NAV_DOWN         (1U << 1)
#define UI_NAV_LEFT         (1U << 2)
#define UI_NAV_RIGHT        (1U << 3)
#define UI_NAV_CONFIRM      (1U << 4)
#define UI_NAV_BACK         (1U << 5)

/* Menu item indices */
typedef enum {
  UI_ITEM_CONN_MODE = 0,
  UI_ITEM_DONGLE_RESCAN,
  UI_ITEM_SWITCH_LAYOUT,
  UI_ITEM_GYRO_MODE,
  UI_ITEM_SWAP_TRIGGERS,
  UI_ITEM_RUMBLE,
  UI_ITEM_RECAL_GYRO,
  UI_ITEM_CLOSE,
  UI_ITEM_COUNT
} xmb_ui_item_t;

/* Shared runtime configuration controlled by the overlay */
typedef struct {
  uint8_t conn_mode;      /* XPAD_CONN_BLUETOOTH or XPAD_CONN_WIRED */
  uint8_t switch_layout;  /* 0 = PS3 positional, 1 = Switch (A=Cross, B=Circle, X=Square, Y=Triangle) */
  uint8_t gyro_mode;      /* XPAD_GYRO_OFF, XPAD_GYRO_HOLD_ZL, XPAD_GYRO_ALWAYS */
  uint8_t swap_triggers;  /* 0 = Normal, 1 = Swap L1/R1 <-> L2/R2 */
  uint8_t rumble_enabled; /* 1 = Enabled, 0 = Muted */
} xpad_ui_config_t;

/* Action callback invoked when user triggers an option in the overlay */
typedef struct {
  void (*on_conn_mode_changed)(uint8_t new_mode);
  void (*on_dongle_rescan)(void);
  void (*on_gyro_mode_changed)(uint8_t new_mode);
  void (*on_recalibrate_gyro)(void);
  void (*on_rumble_changed)(uint8_t enabled);
  void (*on_notify)(const char *msg);
  int32_t (*is_dongle_present)(void);
  int32_t (*is_bt_connected)(void);
  const char *(*get_dongle_status)(void);
  void *(*resolve_nid)(const char *vsh_module, uint32_t fnid, int32_t offset);
} xmb_ui_callbacks_t;

void xmb_ui_init(const xmb_ui_callbacks_t *cb);
void xmb_ui_toggle(void);
void xmb_ui_open(void);
void xmb_ui_close(void);
uint8_t xmb_ui_is_open(void);
int32_t xmb_ui_get_selected(void);

xpad_ui_config_t *xmb_ui_get_config(void);
void xmb_ui_set_config(const xpad_ui_config_t *cfg);

/* Handle D-Pad / face button input while overlay is open */
void xmb_ui_handle_input(uint32_t nav_mask);

/* Render overlay into an ARGB8888 framebuffer (used by PS3 GCM and host tests) */
void xmb_ui_render_to_buffer(uint32_t *fb, uint32_t width, uint32_t height, uint32_t pitch_bytes);

/* Render overlay onto the active PS3 XMB display buffer (no-op on host) */
void xmb_ui_render_xmb(void);

#endif /* _XMB_UI_H_ */
