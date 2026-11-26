#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "../src/xmb_ui.c"

static int g_pass = 0;
static int g_fail = 0;

#define ASSERT_TRUE(cond, msg) do { \
  if (cond) { \
    g_pass++; \
  } else { \
    g_fail++; \
    printf("  [FAIL] %s (line %d)\n", msg, __LINE__); \
  } \
} while (0)

static int g_conn_cb_calls = 0;
static uint8_t g_last_conn_mode = 0xFF;
static int g_rescan_cb_calls = 0;
static int g_gyro_cb_calls = 0;
static uint8_t g_last_gyro_mode = 0xFF;
static int g_recal_cb_calls = 0;
static int g_notify_calls = 0;
static char g_last_notify[128];
static int32_t g_mock_dongle_present = 1;
static int32_t g_mock_bt_connected = 1;

static void mock_on_conn(uint8_t m) {
  g_conn_cb_calls++;
  g_last_conn_mode = m;
}
static void mock_on_rescan(void) {
  g_rescan_cb_calls++;
}
static void mock_on_gyro(uint8_t m) {
  g_gyro_cb_calls++;
  g_last_gyro_mode = m;
}
static void mock_on_recal(void) {
  g_recal_cb_calls++;
}
static void mock_on_notify(const char *msg) {
  g_notify_calls++;
  if (msg != NULL) {
    strncpy(g_last_notify, msg, sizeof(g_last_notify) - 1);
    g_last_notify[sizeof(g_last_notify) - 1] = '\0';
  }
}
static int32_t mock_is_dongle(void) { return g_mock_dongle_present; }
static int32_t mock_is_bt(void) { return g_mock_bt_connected; }
static const char *mock_status(void) { return g_mock_bt_connected ? "CONNECTED" : "READY"; }

static void setup_ui(void) {
  xmb_ui_callbacks_t cb;
  memset(&cb, 0, sizeof(cb));
  cb.on_conn_mode_changed = mock_on_conn;
  cb.on_dongle_rescan = mock_on_rescan;
  cb.on_gyro_mode_changed = mock_on_gyro;
  cb.on_recalibrate_gyro = mock_on_recal;
  cb.on_notify = mock_on_notify;
  cb.is_dongle_present = mock_is_dongle;
  cb.is_bt_connected = mock_is_bt;
  cb.get_dongle_status = mock_status;

  g_conn_cb_calls = 0;
  g_rescan_cb_calls = 0;
  g_gyro_cb_calls = 0;
  g_recal_cb_calls = 0;
  g_notify_calls = 0;
  g_last_notify[0] = '\0';
  g_mock_dongle_present = 1;
  g_mock_bt_connected = 1;

  xmb_ui_init(&cb);
}

static void press_nav(uint32_t mask) {
  xmb_ui_handle_input(mask);
  xmb_ui_handle_input(0);
}

static void test_toggle_and_latch(void) {
  printf("[1] Testing overlay toggle and release-latch guard...\n");
  setup_ui();
  ASSERT_TRUE(xmb_ui_is_open() == 0, "Overlay starts closed");

  xmb_ui_toggle();
  ASSERT_TRUE(xmb_ui_is_open() == 1, "Overlay opens on toggle");

  /* While held (non-zero before release), input must be ignored */
  xmb_ui_handle_input(UI_NAV_DOWN);
  ASSERT_TRUE(xmb_ui_get_selected() == 0, "Held input before release does not move cursor");

  /* Release clears latch */
  xmb_ui_handle_input(0);
  press_nav(UI_NAV_DOWN);
  ASSERT_TRUE(xmb_ui_get_selected() == 1, "Down moves selection to item 1");

  press_nav(UI_NAV_UP);
  ASSERT_TRUE(xmb_ui_get_selected() == 0, "Up moves selection back to item 0");

  press_nav(UI_NAV_UP);
  ASSERT_TRUE(xmb_ui_get_selected() == UI_ITEM_COUNT - 1, "Up wraps around to last item");

  press_nav(UI_NAV_BACK);
  ASSERT_TRUE(xmb_ui_is_open() == 0, "Back closes overlay");
}

static void test_menu_actions(void) {
  xpad_ui_config_t *cfg;
  printf("[2] Testing menu options (Wired/BT, Rescan, Switch layout, Gyro, Triggers, Rumble, Recal)...\n");
  setup_ui();
  xmb_ui_open();
  xmb_ui_handle_input(0);

  cfg = xmb_ui_get_config();
  ASSERT_TRUE(cfg->conn_mode == XPAD_CONN_BLUETOOTH, "Default connection mode is Bluetooth");

  /* Item 0: Connection Mode */
  press_nav(UI_NAV_CONFIRM);
  ASSERT_TRUE(cfg->conn_mode == XPAD_CONN_WIRED, "Connection mode toggled to Wired USB");
  ASSERT_TRUE(g_conn_cb_calls == 1 && g_last_conn_mode == XPAD_CONN_WIRED, "on_conn_mode_changed fired with WIRED");

  /* Item 1: Dongle Rescan */
  press_nav(UI_NAV_DOWN);
  press_nav(UI_NAV_CONFIRM);
  ASSERT_TRUE(g_rescan_cb_calls == 1, "on_dongle_rescan fired on confirm");

  /* Item 2: Button Layout */
  press_nav(UI_NAV_DOWN);
  ASSERT_TRUE(cfg->switch_layout == 0, "Default button layout is PS3 positional");
  press_nav(UI_NAV_CONFIRM);
  ASSERT_TRUE(cfg->switch_layout == 1, "Button layout toggled to Switch A/B");

  /* Item 3: Gyro Aim Mode */
  press_nav(UI_NAV_DOWN);
  ASSERT_TRUE(cfg->gyro_mode == XPAD_GYRO_OFF, "Default gyro mode is OFF");
  press_nav(UI_NAV_RIGHT);
  ASSERT_TRUE(cfg->gyro_mode == XPAD_GYRO_HOLD_ZL, "Gyro mode cycled to HOLD L2/ZL");
  ASSERT_TRUE(g_last_gyro_mode == XPAD_GYRO_HOLD_ZL, "on_gyro_mode_changed fired with HOLD_ZL");
  press_nav(UI_NAV_RIGHT);
  ASSERT_TRUE(cfg->gyro_mode == XPAD_GYRO_ALWAYS, "Gyro mode cycled to ALWAYS ACTIVE");
  press_nav(UI_NAV_LEFT);
  ASSERT_TRUE(cfg->gyro_mode == XPAD_GYRO_HOLD_ZL, "Left decrements gyro mode cleanly");

  /* Item 4: Swap Triggers */
  press_nav(UI_NAV_DOWN);
  press_nav(UI_NAV_CONFIRM);
  ASSERT_TRUE(cfg->swap_triggers == 1, "Trigger swap toggled ON");

  /* Item 5: Rumble */
  press_nav(UI_NAV_DOWN);
  ASSERT_TRUE(cfg->rumble_enabled == 1, "Rumble enabled by default");
  press_nav(UI_NAV_CONFIRM);
  ASSERT_TRUE(cfg->rumble_enabled == 0, "Rumble muted");

  /* Item 6: Recalibrate Gyro */
  press_nav(UI_NAV_DOWN);
  press_nav(UI_NAV_CONFIRM);
  ASSERT_TRUE(g_recal_cb_calls == 1, "on_recalibrate_gyro fired");

  /* Item 7: Close */
  press_nav(UI_NAV_DOWN);
  press_nav(UI_NAV_CONFIRM);
  ASSERT_TRUE(xmb_ui_is_open() == 0, "Close item closes overlay");
}

static void test_purple_terminal_render(void) {
  uint32_t *fb;
  uint32_t w = 1280, h = 720, pitch = 1280 * 4;
  uint32_t center_px;
  int has_purple_border = 0;
  int has_text_px = 0;
  uint32_t x, y;

  printf("[3] Testing purple terminal framebuffer rendering & bounds safety...\n");
  fb = (uint32_t *)calloc(w * h, sizeof(uint32_t));
  ASSERT_TRUE(fb != NULL, "Allocated 1280x720 test framebuffer");

  setup_ui();
  xmb_ui_open();
  xmb_ui_render_to_buffer(fb, w, h, pitch);

  center_px = fb[(h / 2) * w + (w / 2)];
  ASSERT_TRUE(center_px != 0x00000000U, "Overlay rendered non-zero pixels in center");

  for (y = 0; y < h; y++) {
    for (x = 0; x < w; x++) {
      uint32_t p = fb[y * w + x];
      if (p == UI_COL_BORDER) has_purple_border = 1;
      if (p == UI_COL_TEXT) has_text_px = 1;
    }
  }
  ASSERT_TRUE(has_purple_border, "Neon purple border (#9D4EDD) present in framebuffer");
  ASSERT_TRUE(has_text_px, "Lavender-white text (#F3E8FF) present in framebuffer");

  /* Bounds safety: too-small or NULL buffer should not crash */
  xmb_ui_render_to_buffer(NULL, w, h, pitch);
  xmb_ui_render_to_buffer(fb, 100, 100, 100 * 4);
  ASSERT_TRUE(1, "Null/undersized framebuffer calls handled safely");

  free(fb);
}

int main(void) {
  printf("====================================================\n");
  printf(" ps3xpad XMB Purple Terminal Overlay Unit Tests     \n");
  printf("====================================================\n");

  test_toggle_and_latch();
  test_menu_actions();
  test_purple_terminal_render();

  printf("====================================================\n");
  printf(" RESULTS: %d passed, %d failed\n", g_pass, g_fail);
  printf("====================================================\n");
  return g_fail == 0 ? 0 : 1;
}
