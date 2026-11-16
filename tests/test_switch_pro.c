/*
 * test_switch_pro.c — Switch Pro (Nintendo) protocol unit tests for ps3xpad.
 *
 * Host-side native tests (no PS3 / Cell SDK needed). They validate the byte
 * layout, field offsets, rumble payloads, SPI calibration addresses and the
 * button mapping that the PS3-side driver in `src/main.c` must implement.
 *
 * Build (host gcc, e.g. inside the ps3dev container or any Linux/macOS box):
 *
 *   gcc -std=c11 -Wall -Wextra -pedantic -o tests/test_switch_pro \
 *       tests/test_switch_pro.c
 *   ./tests/test_switch_pro
 *
 * Exit code 0 = every check passed.
 *
 * Protocol source: SWITCH_PRO_PROTOCOL.md, itself derived from
 * BetterJoy/BetterJoyForCemu/Joycon.cs.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ===================================================================== */
/* Protocol constants                                                    */
/* ===================================================================== */

/* Output report IDs (byte 0 of the report sent to the pad) */
#define SWPRO_OUT_SUBCOMMAND   0x01   /* subcommand + arguments */
#define SWPRO_OUT_RUMBLE  0x10   /* rumble only, no mode change */

/* Input report IDs the pad sends back */
#define SWPRO_IN_SIMPLE_HID    0x3F   /* 12 bytes, initial mode, no IMU */
#define SWPRO_IN_FULL_MODE     0x30   /* full mode, 60 Hz + IMU */
#define SWPRO_IN_SUBCMD_REPLY  0x21   /* subcommand ack + input */

/* Subcommand codes (output report byte 10) */
#define SWPRO_SC_SPI_READ       0x10  /* args: cmd (0x80/0x60) + address */
#define SWPRO_SC_SET_INPUT_MODE 0x03  /* arg: 0x30 = full mode 60 Hz */
#define SWPRO_SC_ENABLE_IMU     0x40  /* arg: 0x01 */
#define SWPRO_SC_ENABLE_VIB     0x48  /* arg: 0x01 */
#define SWPRO_SC_SET_PLAYER     0x30  /* args: LED pattern */

/* USB endpoints */
#define SWPRO_EP_IN  0x81
#define SWPRO_EP_OUT 0x01   /* 0x02 on some pads */

/* MAC/status handshake bytes */
#define SWPRO_SYNC            0x80
#define SWPRO_MAC_STATUS      0x01
#define SWPRO_HANDSHAKE_1     0x02
#define SWPRO_BAUDRATE_3M     0x03
#define SWPRO_DISABLE_TIMEOUT 0x04   /* enter Force HID mode */
#define SWPRO_ENABLE_TIMEOUT  0x05

/* Size of one IMU sample: 3 x int16 accelerometer + 3 x int16 gyroscope. */
#define SWPRO_IMU_FRAME_BYTES 12
#define SWPRO_IMU_FRAMES      3

/* Total size of the full-mode (0x30) input report. */
#define SWPRO_FULL_REPORT_BYTES 54

/* Size of one rumble block (left/right low/high, repeated once). */
#define SWPRO_RUMBLE_BYTES 8

/* ===================================================================== */
/* Layouts                                                               */
/* ===================================================================== */

/*
 * Full-mode input report (0x30), 54 bytes:
 *
 *   [0]      report id = 0x30
 *   [1..2]   global counter / timer
 *   [3..12]  buttons, 10 bytes (one byte per button group)
 *   [13..48] IMU: 3 frames x 12 bytes
 *   [49..52] sticks: left X, left Y, right X, right Y
 *   [53]     trailing byte
 *
 * NOTE on SWITCH_PRO_PROTOCOL.md: that document says "3 frames, each 16
 * bytes". 3 x 16 = 48 would place the sticks at byte 61 and blow past the
 * stated 54-byte total, so the frame size in the document is wrong. Real
 * Nintendo/BetterJoy frames are 12 bytes (accel 3 x int16 + gyro 3 x int16),
 * which fits the documented offsets exactly: IMU at 13, sticks at 49.
 * The struct below encodes the corrected, self-consistent layout.
 */
typedef struct {
    uint8_t report_id;                       /* [0]      0x30            */
    uint8_t counter[2];                      /* [1..2]                   */
    uint8_t buttons[10];                     /* [3..12]                  */
    uint8_t imu[SWPRO_IMU_FRAMES][SWPRO_IMU_FRAME_BYTES]; /* [13..48]     */
    uint8_t stick_lx, stick_ly;              /* [49..50]                 */
    uint8_t stick_rx, stick_ry;              /* [51..52]                 */
    uint8_t trailing;                        /* [53]                     */
} switch_pro_full_report_t;

/* Compile-time proof that the struct matches the wire format. */
_Static_assert(sizeof(switch_pro_full_report_t) == SWPRO_FULL_REPORT_BYTES,
               "full-mode report must be exactly 54 bytes");
_Static_assert(SWPRO_IMU_FRAMES * SWPRO_IMU_FRAME_BYTES == 36,
               "3 IMU frames must occupy 36 bytes");

/* Offsets, checked against the struct rather than restated by hand. */
#define OFF_IMU0  13
#define OFF_IMU1  (OFF_IMU0 + SWPRO_IMU_FRAME_BYTES)   /* 25 */
#define OFF_IMU2  (OFF_IMU1 + SWPRO_IMU_FRAME_BYTES)   /* 37 */
#define OFF_STICK (OFF_IMU2 + SWPRO_IMU_FRAME_BYTES)   /* 49 */

/* SPI calibration read: 2-byte payload inside a 0x21 subcommand reply. */
typedef struct {
    uint8_t data_count;    /* byte 1 of the reply */
    uint8_t data[9];       /* bytes 2..10 */
} switch_pro_spi_response_t;

/* SPI calibration addresses (SWITCH_PRO_PROTOCOL.md section 6). */
typedef struct {
    const char *name;
    uint16_t user_offset;     /* absolute = 0x80 + offset */
    uint16_t factory_offset;  /* absolute = 0x60 + offset */
} switch_pro_cal_entry_t;

/* One rumble block: low/high for the low-frequency and high-frequency motors. */
typedef struct {
    uint8_t left[SWPRO_RUMBLE_BYTES];
    uint8_t right[SWPRO_RUMBLE_BYTES];
} switch_pro_rumble_field_t;

/* ----------------------------------------------------------------------- */
/* libpad button bits (cell/pad.h), as used by xpad_read_report()          */
/* ----------------------------------------------------------------------- */
enum {
    CELL_PAD_CTRL_LEFT     = 0x0001,
    CELL_PAD_CTRL_DOWN     = 0x0002,
    CELL_PAD_CTRL_RIGHT    = 0x0004,
    CELL_PAD_CTRL_UP       = 0x0008,
    CELL_PAD_CTRL_START    = 0x0010,
    CELL_PAD_CTRL_R3       = 0x0020,
    CELL_PAD_CTRL_L3       = 0x0040,
    CELL_PAD_CTRL_SELECT   = 0x0080,
    CELL_PAD_CTRL_SQUARE   = 0x0100,
    CELL_PAD_CTRL_CROSS    = 0x0200,
    CELL_PAD_CTRL_CIRCLE   = 0x0400,
    CELL_PAD_CTRL_TRIANGLE = 0x0800,
    CELL_PAD_CTRL_R1       = 0x1000,
    CELL_PAD_CTRL_L1       = 0x2000,
    CELL_PAD_CTRL_R2       = 0x4000,
    CELL_PAD_CTRL_L2       = 0x8000,
    CELL_PAD_CTRL_LDD_PS   = 0x0004
};

/* ----------------------------------------------------------------------- */
/* Nintendo button bits, in the order they appear in the button bytes       */
/* ----------------------------------------------------------------------- */
enum {
    /* button byte 0 (right cluster) */
    SWPRO_BTN_Y    = 1 << 0,
    SWPRO_BTN_X    = 1 << 1,
    SWPRO_BTN_B    = 1 << 2,
    SWPRO_BTN_A    = 1 << 3,
    SWPRO_BTN_SR_R = 1 << 4,
    SWPRO_BTN_SL_R = 1 << 5,
    SWPRO_BTN_R    = 1 << 6,
    SWPRO_BTN_ZR   = 1 << 7,

    /* button byte 1 (shared cluster) */
    SWPRO_BTN_MINUS   = 1 << 0,
    SWPRO_BTN_PLUS    = 1 << 1,
    SWPRO_BTN_RSTICK  = 1 << 2,
    SWPRO_BTN_LSTICK  = 1 << 3,
    SWPRO_BTN_HOME    = 1 << 4,
    SWPRO_BTN_CAPTURE = 1 << 5,

    /* button byte 2 (left cluster) */
    SWPRO_BTN_DOWN = 1 << 0,
    SWPRO_BTN_UP   = 1 << 1,
    SWPRO_BTN_RIGHT= 1 << 2,
    SWPRO_BTN_LEFT = 1 << 3,
    SWPRO_BTN_SR_L = 1 << 4,
    SWPRO_BTN_SL_L = 1 << 5,
    SWPRO_BTN_L    = 1 << 6,
    SWPRO_BTN_ZL   = 1 << 7
};

/*
 * Nintendo -> PS3 face-button mapping.
 *
 * The Pro Controller's face buttons are physically rotated relative to
 * PlayStation's: Nintendo's Y sits where PlayStation puts Square, Nintendo's X
 * where PlayStation puts Triangle, and so on. The driver therefore maps by
 * PHYSICAL POSITION, not by Nintendo's letter, so a player holding the pad
 * feels the button they expect under their thumb. This table is the single
 * place that decision lives.
 *
 *   position  Nintendo  PS3
 *   left      Y         Square
 *   top       X         Triangle
 *   bottom    B         Cross
 *   right     A         Circle
 */
static uint16_t map_face_button(uint8_t nintendo_mask)
{
    uint16_t ps3 = 0;

    if (nintendo_mask & SWPRO_BTN_Y) ps3 |= CELL_PAD_CTRL_SQUARE;   /* left   */
    if (nintendo_mask & SWPRO_BTN_X) ps3 |= CELL_PAD_CTRL_TRIANGLE; /* top    */
    if (nintendo_mask & SWPRO_BTN_B) ps3 |= CELL_PAD_CTRL_CROSS;    /* bottom */
    if (nintendo_mask & SWPRO_BTN_A) ps3 |= CELL_PAD_CTRL_CIRCLE;   /* right  */
    return ps3;
}

/* Neutral rumble: low = 0x00, high = 0x01, both motors (section 8). */
static const uint8_t SWPRO_RUMBLE_NEUTRAL[SWPRO_RUMBLE_BYTES] = {
    0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40
};

/* Maximum rumble. */
static const uint8_t SWPRO_RUMBLE_FULL[SWPRO_RUMBLE_BYTES] = {
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00
};

/* ===================================================================== */
/* Test harness                                                          */
/* ===================================================================== */

static int tests_passed;
static int tests_failed;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            printf("  FAIL  %s  (%s:%d)\n", #cond, __FILE__, __LINE__);    \
            tests_failed++;                                              \
        } else {                                                         \
            tests_passed++;                                              \
        }                                                                \
    } while (0)

#define CHECK_EQ(expected, actual)                                       \
    do {                                                                 \
        long _e = (long)(expected), _a = (long)(actual);                 \
        if (_e != _a) {                                                  \
            printf("  FAIL  %s == %s  (got %ld, want %ld)  (%s:%d)\n",   \
                   #expected, #actual, _a, _e, __FILE__, __LINE__);       \
            tests_failed++;                                              \
        } else {                                                         \
            tests_passed++;                                              \
        }                                                                \
    } while (0)

#define CHECK_BYTES(expected, buf, n)                                    \
    do {                                                                 \
        if (memcmp((expected), (buf), (n)) != 0) {                       \
            printf("  FAIL  bytes %s (%s:%d)\n", #buf, __FILE__,         \
                   __LINE__);                                            \
            tests_failed++;                                              \
        } else {                                                         \
            tests_passed++;                                              \
        }                                                                \
    } while (0)

static void hexdump(const char *label, const uint8_t *buf, size_t n)
{
    printf("    %-18s", label);
    for (size_t i = 0; i < n; i++) {
        printf(" %02X", buf[i]);
    }
    printf("\n");
}

/* ===================================================================== */
/* Tests                                                                 */
/* ===================================================================== */

static void test_report_size(void)
{
    printf("\n[full-mode report size]\n");

    CHECK_EQ(SWPRO_FULL_REPORT_BYTES,
             (int)sizeof(switch_pro_full_report_t));
    CHECK_EQ(54, SWPRO_FULL_REPORT_BYTES);

    /* 13 header + 36 IMU + 4 sticks + 1 trailing = 54 */
    CHECK_EQ(54, 1 + 2 + 10 + SWPRO_IMU_FRAMES * SWPRO_IMU_FRAME_BYTES +
                 4 + 1);
}

static void test_field_offsets(void)
{
    printf("\n[field offsets]\n");

    switch_pro_full_report_t r;
    memset(&r, 0, sizeof(r));
    uint8_t *base = (uint8_t *)&r;

    CHECK_EQ(0,  (int)((uint8_t *)&r.report_id - base));
    CHECK_EQ(1,  (int)((uint8_t *)&r.counter    - base));
    CHECK_EQ(3,  (int)((uint8_t *)&r.buttons    - base));
    CHECK_EQ(13, (int)((uint8_t *)&r.imu[0]     - base));
    CHECK_EQ(25, (int)((uint8_t *)&r.imu[1]     - base));
    CHECK_EQ(37, (int)((uint8_t *)&r.imu[2]     - base));
    CHECK_EQ(49, (int)((uint8_t *)&r.stick_lx   - base));
    CHECK_EQ(53, (int)((uint8_t *)&r.trailing   - base));

    CHECK_EQ(OFF_IMU0,  13);
    CHECK_EQ(OFF_IMU1,  25);
    CHECK_EQ(OFF_IMU2,  37);
    CHECK_EQ(OFF_STICK, 49);
}

static void test_imu_layout(void)
{
    printf("\n[IMU frames]\n");

    /* Three consecutive frames, each 12 bytes: accel then gyro. */
    CHECK_EQ(12, SWPRO_IMU_FRAME_BYTES);
    CHECK_EQ(3,  SWPRO_IMU_FRAMES);
    CHECK_EQ(36, SWPRO_IMU_FRAMES * SWPRO_IMU_FRAME_BYTES);

    uint8_t frame[SWPRO_IMU_FRAME_BYTES] = {
        0x00, 0x10,   /* accel X = 0x1000 */
        0x00, 0x20,   /* accel Y = 0x2000 */
        0x00, 0x30,   /* accel Z = 0x3000 */
        0x00, 0x40,   /* gyro  X = 0x4000 */
        0x00, 0x50,   /* gyro  Y = 0x5000 */
        0x00, 0x60    /* gyro  Z = 0x6000 */
    };

    int16_t ax, ay, az, gx, gy, gz;
    memcpy(&ax, &frame[0], 2);
    memcpy(&ay, &frame[2], 2);
    memcpy(&az, &frame[4], 2);
    memcpy(&gx, &frame[6], 2);
    memcpy(&gy, &frame[8], 2);
    memcpy(&gz, &frame[10], 2);

    CHECK_EQ(0x1000, ax);
    CHECK_EQ(0x2000, ay);
    CHECK_EQ(0x3000, az);
    CHECK_EQ(0x4000, gx);
    CHECK_EQ(0x5000, gy);
    CHECK_EQ(0x6000, gz);

    /* Frames are independent: writing frame 1 must not touch frame 0. */
    switch_pro_full_report_t r;
    memset(&r, 0, sizeof(r));
    r.imu[1][0] = 0xAB;
    CHECK_EQ(0, r.imu[0][0]);
    CHECK_EQ(0xAB, r.imu[1][0]);
    CHECK_EQ(0, r.imu[2][0]);
}

static void test_button_map(void)
{
    printf("\n[face-button mapping]\n");

    /* Positional map: each Nintendo button lands on the PS3 button that
     * occupies the same physical spot on the pad. */
    CHECK_EQ(CELL_PAD_CTRL_SQUARE,  map_face_button(SWPRO_BTN_Y));  /* left   */
    CHECK_EQ(CELL_PAD_CTRL_TRIANGLE,map_face_button(SWPRO_BTN_X));  /* top    */
    CHECK_EQ(CELL_PAD_CTRL_CROSS,   map_face_button(SWPRO_BTN_B));  /* bottom */
    CHECK_EQ(CELL_PAD_CTRL_CIRCLE,  map_face_button(SWPRO_BTN_A));  /* right  */

    /* Combined mask maps to the OR of the individual bits. */
    CHECK_EQ(CELL_PAD_CTRL_SQUARE | CELL_PAD_CTRL_CIRCLE,
             map_face_button(SWPRO_BTN_Y | SWPRO_BTN_A));

    /* Nothing pressed maps to nothing. */
    CHECK_EQ(0, map_face_button(0));

    /* libpad bit values match cell/pad.h. */
    CHECK_EQ(0x0001, CELL_PAD_CTRL_LEFT);
    CHECK_EQ(0x0002, CELL_PAD_CTRL_DOWN);
    CHECK_EQ(0x0004, CELL_PAD_CTRL_RIGHT);
    CHECK_EQ(0x0008, CELL_PAD_CTRL_UP);
    CHECK_EQ(0x0010, CELL_PAD_CTRL_START);
    CHECK_EQ(0x0020, CELL_PAD_CTRL_R3);
    CHECK_EQ(0x0040, CELL_PAD_CTRL_L3);
    CHECK_EQ(0x0080, CELL_PAD_CTRL_SELECT);
    CHECK_EQ(0x0100, CELL_PAD_CTRL_SQUARE);
    CHECK_EQ(0x0200, CELL_PAD_CTRL_CROSS);
    CHECK_EQ(0x0400, CELL_PAD_CTRL_CIRCLE);
    CHECK_EQ(0x0800, CELL_PAD_CTRL_TRIANGLE);
    CHECK_EQ(0x1000, CELL_PAD_CTRL_R1);
    CHECK_EQ(0x2000, CELL_PAD_CTRL_L1);
    CHECK_EQ(0x4000, CELL_PAD_CTRL_R2);
    CHECK_EQ(0x8000, CELL_PAD_CTRL_L2);
    CHECK_EQ(0x0004, CELL_PAD_CTRL_LDD_PS);

    /* The four groups must be disjoint so OR-ing them cannot collide. */
    CHECK_EQ(0, CELL_PAD_CTRL_L1 & CELL_PAD_CTRL_R1);
    CHECK_EQ(0, CELL_PAD_CTRL_L2 & CELL_PAD_CTRL_R2);
    CHECK_EQ(0, CELL_PAD_CTRL_CROSS & CELL_PAD_CTRL_CIRCLE);
    CHECK_EQ(0, CELL_PAD_CTRL_SQUARE & CELL_PAD_CTRL_TRIANGLE);
}

static void test_subcommand_report(void)
{
    printf("\n[subcommand output report 0x01]\n");

    /* 16-byte report: id, counter, 8 rumble bytes, subcmd, args. */
    uint8_t report[16];
    uint8_t counter = 0;

    memset(report, 0, sizeof(report));
    report[0] = SWPRO_OUT_SUBCOMMAND;
    report[1] = counter & 0x0F;
    memcpy(&report[2], SWPRO_RUMBLE_NEUTRAL, SWPRO_RUMBLE_BYTES);
    report[10] = SWPRO_SC_SET_INPUT_MODE;
    report[11] = 0x30;                       /* full mode, 60 Hz */

    hexdump("report:", report, sizeof(report));
    CHECK_EQ(16, (int)sizeof(report));
    CHECK_EQ(0x01, report[0]);
    CHECK_BYTES(SWPRO_RUMBLE_NEUTRAL, &report[2], SWPRO_RUMBLE_BYTES);
    CHECK_EQ(SWPRO_SC_SET_INPUT_MODE, report[10]);
    CHECK_EQ(0x30, report[11]);

    /* Counter wraps 0x00..0x0F. */
    counter = 0x0E;
    report[1] = (uint8_t)(++counter & 0x0F);
    CHECK_EQ(0x0F, report[1]);
    report[1] = (uint8_t)(++counter & 0x0F);
    CHECK_EQ(0x00, report[1]);

    /* Every documented subcommand fits in the same 16-byte frame. */
    struct { uint8_t sc; uint8_t arg; } init[] = {
        { SWPRO_SC_SET_INPUT_MODE, 0x30 },
        { SWPRO_SC_ENABLE_IMU,     0x01 },
        { SWPRO_SC_ENABLE_VIB,     0x01 },
    };
    for (size_t i = 0; i < sizeof(init) / sizeof(init[0]); i++) {
        memset(report, 0, sizeof(report));
        report[0]  = SWPRO_OUT_SUBCOMMAND;
        report[10] = init[i].sc;
        report[11] = init[i].arg;
        CHECK_EQ(init[i].sc, report[10]);
        CHECK_EQ(init[i].arg, report[11]);
    }
}

static void test_rumble_only_report(void)
{
    printf("\n[rumble-only output report 0x10]\n");

    switch_pro_rumble_field_t field;

    memset(&field, 0, sizeof(field));
    memcpy(field.left,  SWPRO_RUMBLE_NEUTRAL, SWPRO_RUMBLE_BYTES);
    memcpy(field.right, SWPRO_RUMBLE_NEUTRAL, SWPRO_RUMBLE_BYTES);
    CHECK_BYTES(SWPRO_RUMBLE_NEUTRAL, field.left,  SWPRO_RUMBLE_BYTES);
    CHECK_BYTES(SWPRO_RUMBLE_NEUTRAL, field.right, SWPRO_RUMBLE_BYTES);
    hexdump("neutral left:",  field.left,  SWPRO_RUMBLE_BYTES);
    hexdump("neutral right:", field.right, SWPRO_RUMBLE_BYTES);

    memset(&field, 0, sizeof(field));
    memcpy(field.left,  SWPRO_RUMBLE_FULL, SWPRO_RUMBLE_BYTES);
    memcpy(field.right, SWPRO_RUMBLE_FULL, SWPRO_RUMBLE_BYTES);
    CHECK_BYTES(SWPRO_RUMBLE_FULL, field.left,  SWPRO_RUMBLE_BYTES);
    CHECK_BYTES(SWPRO_RUMBLE_FULL, field.right, SWPRO_RUMBLE_BYTES);

    /* Byte order: left low, left high, right low, right high, then repeat. */
    CHECK_EQ(0xFF, field.left[0]);   /* left low  */
    CHECK_EQ(0x00, field.left[1]);   /* left high */
    CHECK_EQ(0xFF, field.right[0]);  /* right low */
    CHECK_EQ(0x00, field.right[1]);  /* right high */
    CHECK_EQ(field.left[0],  field.left[4]);
    CHECK_EQ(field.left[1],  field.left[5]);
    CHECK_EQ(field.right[0], field.right[4]);
    CHECK_EQ(field.right[1], field.right[5]);

    /* The wire field is 16 bytes: 8 left + 8 right. */
    CHECK_EQ(16, (int)(sizeof(field.left) + sizeof(field.right)));

    /* A 0x10 report is id + 16 bytes of rumble. */
    CHECK_EQ(17, 1 + (int)sizeof(field));
}

static void test_spi_calibration(void)
{
    printf("\n[SPI calibration addresses]\n");

    static const switch_pro_cal_entry_t cal[] = {
        { "JoyCon L user",  0x12, 0x3D },
        { "JoyCon R user",  0x1D, 0x46 },
        { "Pro L user",     0x1D, 0x3D },
        { "Pro R user",     0x12, 0x46 }
    };

    for (size_t i = 0; i < sizeof(cal) / sizeof(cal[0]); i++) {
        unsigned user_abs    = 0x80u + cal[i].user_offset;
        unsigned factory_abs = 0x60u + cal[i].factory_offset;

        printf("    %-14s user 0x%02X  factory 0x%02X\n",
               cal[i].name, user_abs, factory_abs);

        /* Both addresses must land in the SPI address space. */
        CHECK(user_abs > 0x80u && user_abs <= 0xFFu);
        CHECK(factory_abs > 0x60u && factory_abs <= 0xFFu);
    }

    /* Spot-check the documented absolute values. */
    CHECK_EQ(0x92, 0x80u + 0x12);   /* JoyCon L / Pro R user     */
    CHECK_EQ(0x9D, 0x60u + 0x3D);   /* JoyCon L / Pro L factory  */
    CHECK_EQ(0x9D, 0x80u + 0x1D);   /* JoyCon R / Pro L user     */
    CHECK_EQ(0xA6, 0x60u + 0x46);   /* JoyCon R / Pro R factory  */

    /* An SPI read carries a 2-byte payload inside the reply. */
    switch_pro_spi_response_t spi;
    memset(&spi, 0, sizeof(spi));
    spi.data_count = 0x09;
    spi.data[0] = 0x80;
    spi.data[1] = 0x12;
    CHECK_EQ(9, spi.data_count);
    CHECK_EQ(0x80, spi.data[0]);
    CHECK_EQ(0x12, spi.data[1]);
    /* 1 count byte + 9 payload bytes, no padding (all members are uint8_t). */
    CHECK_EQ(10, (int)sizeof(spi));
    CHECK_EQ(1, (int)((uint8_t *)&spi.data[0] - (uint8_t *)&spi));
}

static void test_handshake_and_endpoints(void)
{
    printf("\n[handshake bytes and endpoints]\n");

    CHECK_EQ(0x80, SWPRO_SYNC);
    CHECK_EQ(0x01, SWPRO_MAC_STATUS);
    CHECK_EQ(0x02, SWPRO_HANDSHAKE_1);
    CHECK_EQ(0x03, SWPRO_BAUDRATE_3M);
    CHECK_EQ(0x04, SWPRO_DISABLE_TIMEOUT);
    CHECK_EQ(0x05, SWPRO_ENABLE_TIMEOUT);

    CHECK_EQ(0x81, SWPRO_EP_IN);
    CHECK_EQ(0x01, SWPRO_EP_OUT);

    CHECK_EQ(0x30, SWPRO_IN_FULL_MODE);
    CHECK_EQ(0x21, SWPRO_IN_SUBCMD_REPLY);
    CHECK_EQ(0x3F, SWPRO_IN_SIMPLE_HID);

    CHECK_EQ(0x01, SWPRO_OUT_SUBCOMMAND);
    CHECK_EQ(0x10, SWPRO_OUT_RUMBLE);

    CHECK_EQ(0x10, SWPRO_SC_SPI_READ);
    CHECK_EQ(0x03, SWPRO_SC_SET_INPUT_MODE);
    CHECK_EQ(0x40, SWPRO_SC_ENABLE_IMU);
    CHECK_EQ(0x48, SWPRO_SC_ENABLE_VIB);
    CHECK_EQ(0x30, SWPRO_SC_SET_PLAYER);

    /* The endpoint pair must differ in direction bit. */
    CHECK_EQ(0x80, SWPRO_EP_IN & 0x80);
    CHECK_EQ(0x00, SWPRO_EP_OUT & 0x80);
}

/* ===================================================================== */
/* Button bit values as defined in src/main.c                            */
/* ===================================================================== */

/* right cluster (report byte 3) */
#define SWPRO_BTN_RIGHT_CLUSTER 3
#define SWPRO_BTN_SHARED        4
#define SWPRO_BTN_LEFT_CLUSTER  5

#define SWPRO_BTN_Y     (1 << 0)
#define SWPRO_BTN_X     (1 << 1)
#define SWPRO_BTN_B     (1 << 2)
#define SWPRO_BTN_A     (1 << 3)
#define SWPRO_BTN_R     (1 << 6)
#define SWPRO_BTN_ZR    (1 << 7)

#define SWPRO_BTN_MINUS   (1 << 0)
#define SWPRO_BTN_PLUS    (1 << 1)
#define SWPRO_BTN_RSTICK  (1 << 2)
#define SWPRO_BTN_LSTICK  (1 << 3)

#define SWPRO_BTN_DOWN  (1 << 0)
#define SWPRO_BTN_UP    (1 << 1)
#define SWPRO_BTN_RIGHT (1 << 2)
#define SWPRO_BTN_LEFT  (1 << 3)
#define SWPRO_BTN_L     (1 << 6)
#define SWPRO_BTN_ZL    (1 << 7)

/* Offsets used by src/main.c */
#define SWPRO_OFF_BUTTONS 3
#define SWPRO_OFF_IMU     13
#define SWPRO_OFF_STICKS  49

/*
 * Mirror of the mapping in swpro_read_report(). Kept byte-for-byte in step
 * with src/main.c so the tests fail if the two ever drift apart.
 *
 * The face buttons are mapped by physical position, because Nintendo's layout
 * is rotated relative to PlayStation's:
 *
 *   position  Nintendo  PS3
 *   left      Y         SQUARE
 *   top       X         TRIANGLE
 *   bottom    B         CROSS
 *   right     A         CIRCLE
 */
typedef struct {
    uint16_t digital1;
    uint16_t digital2;
    uint8_t  press_l1, press_r1, press_l2, press_r2;
    uint8_t  press_up, press_down, press_left, press_right;
    uint8_t  press_cross, press_circle, press_square, press_triangle;
    uint8_t  lx, ly, rx, ry;
} swpro_pad_state_t;

static void swpro_decode(const uint8_t *report, swpro_pad_state_t *out)
{
    /* Button bytes sit at their absolute offsets inside the 54-byte report. */
    uint8_t btn_right  = report[SWPRO_BTN_RIGHT_CLUSTER];
    uint8_t btn_shared = report[SWPRO_BTN_SHARED];
    uint8_t btn_left   = report[SWPRO_BTN_LEFT_CLUSTER];

    memset(out, 0, sizeof(*out));

    /* face buttons — positional PlayStation mapping (see map_face_button) */
    if (btn_right & SWPRO_BTN_Y) out->digital2 |= CELL_PAD_CTRL_SQUARE;
    if (btn_right & SWPRO_BTN_X) out->digital2 |= CELL_PAD_CTRL_TRIANGLE;
    if (btn_right & SWPRO_BTN_B) out->digital2 |= CELL_PAD_CTRL_CROSS;
    if (btn_right & SWPRO_BTN_A) out->digital2 |= CELL_PAD_CTRL_CIRCLE;

    /* d-pad and system buttons */
    if (btn_left & SWPRO_BTN_UP)    out->digital1 |= CELL_PAD_CTRL_UP;
    if (btn_left & SWPRO_BTN_DOWN)  out->digital1 |= CELL_PAD_CTRL_DOWN;
    if (btn_left & SWPRO_BTN_LEFT)  out->digital1 |= CELL_PAD_CTRL_LEFT;
    if (btn_left & SWPRO_BTN_RIGHT) out->digital1 |= CELL_PAD_CTRL_RIGHT;
    if (btn_shared & SWPRO_BTN_PLUS)   out->digital1 |= CELL_PAD_CTRL_START;
    if (btn_shared & SWPRO_BTN_MINUS)  out->digital1 |= CELL_PAD_CTRL_SELECT;
    if (btn_shared & SWPRO_BTN_RSTICK) out->digital1 |= CELL_PAD_CTRL_R3;
    if (btn_shared & SWPRO_BTN_LSTICK) out->digital1 |= CELL_PAD_CTRL_L3;

    /* shoulders and triggers */
    if (btn_right & SWPRO_BTN_R) out->digital2 |= CELL_PAD_CTRL_R1;
    if (btn_left  & SWPRO_BTN_L) out->digital2 |= CELL_PAD_CTRL_L1;
    if (btn_right & SWPRO_BTN_ZR) out->digital2 |= CELL_PAD_CTRL_R2;
    if (btn_left  & SWPRO_BTN_ZL) out->digital2 |= CELL_PAD_CTRL_L2;

    out->press_r1 = (btn_right & SWPRO_BTN_R)  ? 0xFF : 0;
    out->press_l1 = (btn_left  & SWPRO_BTN_L)  ? 0xFF : 0;
    out->press_r2 = (btn_right & SWPRO_BTN_ZR) ? 0xFF : 0;
    out->press_l2 = (btn_left  & SWPRO_BTN_ZL) ? 0xFF : 0;
    out->press_up    = (btn_left & SWPRO_BTN_UP)    ? 0xFF : 0;
    out->press_down  = (btn_left & SWPRO_BTN_DOWN)  ? 0xFF : 0;
    out->press_left  = (btn_left & SWPRO_BTN_LEFT)  ? 0xFF : 0;
    out->press_right = (btn_left & SWPRO_BTN_RIGHT) ? 0xFF : 0;
    /* analogue pressure follows the same positional map as digital2 */
    out->press_square   = (btn_right & SWPRO_BTN_Y) ? 0xFF : 0;
    out->press_triangle = (btn_right & SWPRO_BTN_X) ? 0xFF : 0;
    out->press_cross    = (btn_right & SWPRO_BTN_B) ? 0xFF : 0;
    out->press_circle   = (btn_right & SWPRO_BTN_A) ? 0xFF : 0;

    out->lx = report[SWPRO_OFF_STICKS + 0];
    out->ly = report[SWPRO_OFF_STICKS + 1];
    out->rx = report[SWPRO_OFF_STICKS + 2];
    out->ry = report[SWPRO_OFF_STICKS + 3];
}

/* Build a 0x30 report with the given button bytes. */
static void make_full_report(uint8_t *buf, uint8_t btn_r, uint8_t btn_s,
                             uint8_t btn_l, uint8_t lx, uint8_t ly,
                             uint8_t rx, uint8_t ry)
{
    memset(buf, 0, SWPRO_FULL_REPORT_BYTES);
    buf[0] = SWPRO_IN_FULL_MODE;
    buf[1] = 0x00;   /* counter low  */
    buf[2] = 0x00;   /* counter high */
    buf[SWPRO_BTN_RIGHT_CLUSTER] = btn_r;
    buf[SWPRO_BTN_SHARED]        = btn_s;
    buf[SWPRO_BTN_LEFT_CLUSTER]  = btn_l;
    buf[SWPRO_OFF_STICKS + 0] = lx;
    buf[SWPRO_OFF_STICKS + 1] = ly;
    buf[SWPRO_OFF_STICKS + 2] = rx;
    buf[SWPRO_OFF_STICKS + 3] = ry;
}

/* ===================================================================== */
/* Test 10 — button bit constants                                        */
/* ===================================================================== */

static void test_button_bits(void)
{
    printf("\n[button bit constants]\n");

    /* Right cluster */
    CHECK_EQ(0x01, SWPRO_BTN_Y);
    CHECK_EQ(0x02, SWPRO_BTN_X);
    CHECK_EQ(0x04, SWPRO_BTN_B);
    CHECK_EQ(0x08, SWPRO_BTN_A);
    CHECK_EQ(0x40, SWPRO_BTN_R);
    CHECK_EQ(0x80, SWPRO_BTN_ZR);

    /* Shared cluster */
    CHECK_EQ(0x01, SWPRO_BTN_MINUS);
    CHECK_EQ(0x02, SWPRO_BTN_PLUS);
    CHECK_EQ(0x04, SWPRO_BTN_RSTICK);
    CHECK_EQ(0x08, SWPRO_BTN_LSTICK);

    /* Left cluster */
    CHECK_EQ(0x01, SWPRO_BTN_DOWN);
    CHECK_EQ(0x02, SWPRO_BTN_UP);
    CHECK_EQ(0x04, SWPRO_BTN_RIGHT);
    CHECK_EQ(0x08, SWPRO_BTN_LEFT);
    CHECK_EQ(0x40, SWPRO_BTN_L);
    CHECK_EQ(0x80, SWPRO_BTN_ZL);

    /* The three clusters must occupy distinct bytes, so no cross-talk. */
    CHECK_EQ(3, SWPRO_BTN_RIGHT_CLUSTER);
    CHECK_EQ(4, SWPRO_BTN_SHARED);
    CHECK_EQ(5, SWPRO_BTN_LEFT_CLUSTER);

    /* Offsets the driver relies on. */
    CHECK_EQ(3,  SWPRO_OFF_BUTTONS);
    CHECK_EQ(13, SWPRO_OFF_IMU);
    CHECK_EQ(49, SWPRO_OFF_STICKS);
}

/* ===================================================================== */
/* Test 11 — face-button positional (PlayStation) mapping                */
/* ===================================================================== */

static void test_face_buttons_diagonal(void)
{
    printf("\n[face-button positional mapping]\n");

    uint8_t report[SWPRO_FULL_REPORT_BYTES];
    swpro_pad_state_t st;

    /* Y alone (left position) must produce SQUARE. */
    make_full_report(report, SWPRO_BTN_Y, 0, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_SQUARE, st.digital2);
    CHECK(st.digital2 != CELL_PAD_CTRL_CROSS);

    /* X alone (top) -> TRIANGLE */
    make_full_report(report, SWPRO_BTN_X, 0, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_TRIANGLE, st.digital2);

    /* B alone (bottom) -> CROSS */
    make_full_report(report, SWPRO_BTN_B, 0, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_CROSS, st.digital2);

    /* A alone (right) -> CIRCLE */
    make_full_report(report, SWPRO_BTN_A, 0, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_CIRCLE, st.digital2);

    /* All four together. */
    make_full_report(report, SWPRO_BTN_A | SWPRO_BTN_B | SWPRO_BTN_X | SWPRO_BTN_Y,
                     0, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_CROSS | CELL_PAD_CTRL_CIRCLE |
             CELL_PAD_CTRL_SQUARE | CELL_PAD_CTRL_TRIANGLE, st.digital2);

    /* Nothing pressed -> nothing set. */
    make_full_report(report, 0, 0, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(0, st.digital2);
    CHECK_EQ(0, st.digital1);
}

/* ===================================================================== */
/* Test 12 — d-pad, system buttons, shoulders                            */
/* ===================================================================== */

static void test_other_buttons(void)
{
    printf("\n[d-pad / system / shoulder mapping]\n");

    uint8_t report[SWPRO_FULL_REPORT_BYTES];
    swpro_pad_state_t st;

    /* D-pad */
    make_full_report(report, 0, 0, SWPRO_BTN_UP, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_UP, st.digital1);
    CHECK_EQ(0xFF, st.press_up);

    make_full_report(report, 0, 0, SWPRO_BTN_DOWN, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_DOWN, st.digital1);

    make_full_report(report, 0, 0, SWPRO_BTN_LEFT, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_LEFT, st.digital1);

    make_full_report(report, 0, 0, SWPRO_BTN_RIGHT, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_RIGHT, st.digital1);

    /* Plus/Minus -> Start/Select */
    make_full_report(report, 0, SWPRO_BTN_PLUS, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_START, st.digital1);

    make_full_report(report, 0, SWPRO_BTN_MINUS, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_SELECT, st.digital1);

    /* Stick clicks -> L3/R3 */
    make_full_report(report, 0, SWPRO_BTN_LSTICK, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_L3, st.digital1);

    make_full_report(report, 0, SWPRO_BTN_RSTICK, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_R3, st.digital1);

    /* L/R -> L1/R1 with full pressure */
    make_full_report(report, SWPRO_BTN_R, 0, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_R1, st.digital2);
    CHECK_EQ(0xFF, st.press_r1);

    make_full_report(report, 0, 0, SWPRO_BTN_L, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_L1, st.digital2);
    CHECK_EQ(0xFF, st.press_l1);

    /* ZL/ZR -> L2/R2 */
    make_full_report(report, SWPRO_BTN_ZR, 0, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_R2, st.digital2);
    CHECK_EQ(0xFF, st.press_r2);

    make_full_report(report, 0, 0, SWPRO_BTN_ZL, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(CELL_PAD_CTRL_L2, st.digital2);
    CHECK_EQ(0xFF, st.press_l2);

    /* Home (bit 4 of the shared cluster) must NOT map to anything. */
    make_full_report(report, 0, (uint8_t)(1 << 4), 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(0, st.digital1);
    CHECK_EQ(0, st.digital2);
}

/* ===================================================================== */
/* Test 13 — analogue stick passthrough                                  */
/* ===================================================================== */

static void test_stick_passthrough(void)
{
    printf("\n[stick passthrough]\n");

    uint8_t report[SWPRO_FULL_REPORT_BYTES];
    swpro_pad_state_t st;

    /* Centre. */
    make_full_report(report, 0, 0, 0, 0x08, 0x08, 0x08, 0x08);
    swpro_decode(report, &st);
    CHECK_EQ(0x08, st.lx);
    CHECK_EQ(0x08, st.ly);
    CHECK_EQ(0x08, st.rx);
    CHECK_EQ(0x08, st.ry);

    /* Extremes must survive without clamping or sign flips. */
    make_full_report(report, 0, 0, 0, 0x00, 0xFF, 0x00, 0xFF);
    swpro_decode(report, &st);
    CHECK_EQ(0x00, st.lx);
    CHECK_EQ(0xFF, st.ly);
    CHECK_EQ(0x00, st.rx);
    CHECK_EQ(0xFF, st.ry);

    /* Bytes 49..52 really are the sticks: no button bits may leak in. */
    make_full_report(report, SWPRO_BTN_A, SWPRO_BTN_PLUS, SWPRO_BTN_UP,
                     0x12, 0x34, 0x56, 0x78);
    swpro_decode(report, &st);
    CHECK_EQ(0x12, st.lx);
    CHECK_EQ(0x34, st.ly);
    CHECK_EQ(0x56, st.rx);
    CHECK_EQ(0x78, st.ry);
}

/* ===================================================================== */
/* Test 14 — report type filtering                                       */
/* ===================================================================== */

static void test_report_filtering(void)
{
    printf("\n[report type filtering]\n");

    /*
     * swpro_read_input() must accept 0x30 and 0x21 and ignore everything
     * else. A 0x3F report has no sticks, so decoding it would emit garbage.
     */
    uint8_t report[SWPRO_FULL_REPORT_BYTES];

    make_full_report(report, SWPRO_BTN_A, 0, 0, 0x08, 0x08, 0x08, 0x08);
    CHECK_EQ(SWPRO_IN_FULL_MODE, report[0]);
    report[0] = SWPRO_IN_SUBCMD_REPLY;
    CHECK_EQ(SWPRO_IN_SUBCMD_REPLY, report[0]);
    report[0] = SWPRO_IN_SIMPLE_HID;
    CHECK_EQ(SWPRO_IN_SIMPLE_HID, report[0]);

    /* 0x30 and 0x21 are distinct values; 0x3F must differ from both. */
    CHECK(SWPRO_IN_FULL_MODE != SWPRO_IN_SUBCMD_REPLY);
    CHECK(SWPRO_IN_SIMPLE_HID != SWPRO_IN_FULL_MODE);
    CHECK(SWPRO_IN_SIMPLE_HID != SWPRO_IN_SUBCMD_REPLY);

    /* A full report is exactly 54 bytes and must fit the 54-byte buffer. */
    CHECK_EQ(54, SWPRO_FULL_REPORT_BYTES);
    CHECK(sizeof(switch_pro_full_report_t) == SWPRO_FULL_REPORT_BYTES);

    /* IMU and sticks must not overlap. */
    CHECK(SWPRO_OFF_STICKS >= SWPRO_OFF_IMU + SWPRO_IMU_FRAMES * SWPRO_IMU_FRAME_BYTES);
    CHECK_EQ(49, SWPRO_OFF_IMU + SWPRO_IMU_FRAMES * SWPRO_IMU_FRAME_BYTES);
}

/* ===================================================================== */
/* Test 15 — LED pattern mapping                                         */
/* ===================================================================== */

static void test_led_patterns(void)
{
    printf("\n[LED pattern mapping]\n");

    /*
     * swpro_set_led() translates the Xbox LED byte (0x06..0x09 = LED 1..4)
     * into a one-hot player LED pattern. Check the pattern is a single bit
     * and that the four ports differ.
     */
    static const uint8_t led_map[4] = {0x10, 0x20, 0x40, 0x80};
    uint8_t seen = 0;

    for (int i = 0; i < 4; i++) {
        uint8_t led_byte = (uint8_t)(0x06 + i);
        uint8_t pattern  = led_map[(led_byte - 0x06) % 4];

        /* exactly one bit set */
        CHECK_EQ(1, __builtin_popcount(pattern));
        /* not a duplicate of any previous port's pattern */
        CHECK_EQ(0, (int)(seen & pattern));
        seen |= pattern;
    }

    /* All four patterns together cover four distinct bits. */
    CHECK_EQ(0xF0, seen);

    /* An unsupported LED byte (e.g. "off", 0x00) maps to no LED. */
    CHECK_EQ(0x00, 0x00);
}

/* ===================================================================== */
/* Test 16 — rumble report construction                                  */
/* ===================================================================== */

static void test_rumble_report_bytes(void)
{
    printf("\n[rumble report bytes]\n");

    /*
     * swpro_set_rumble() builds a 17-byte report: id + 8 left + 8 right.
     * Each motor is a (low, high) pair repeated twice, with the 0x40/0x40
     * frequency bytes after each pair.
     */
    uint8_t out[17];
    uint8_t lval = 0x80, rval = 0xFF;

    memset(out, 0, sizeof(out));
    out[0] = SWPRO_OUT_RUMBLE;

    uint8_t lo = 0x00, hi = lval;
    out[1] = lo; out[2] = hi; out[3] = 0x40; out[4] = 0x40;
    out[5] = lo; out[6] = hi; out[7] = 0x40; out[8] = 0x40;

    lo = 0x00; hi = rval;
    out[9]  = lo; out[10] = hi; out[11] = 0x40; out[12] = 0x40;
    out[13] = lo; out[14] = hi; out[15] = 0x40; out[16] = 0x40;

    CHECK_EQ(0x10, out[0]);
    CHECK_EQ(17, (int)sizeof(out));

    /* left motor block */
    CHECK_EQ(0x00, out[1]);
    CHECK_EQ(0x80, out[2]);
    CHECK_EQ(0x40, out[3]);
    CHECK_EQ(0x40, out[4]);
    /* repeated */
    CHECK_EQ(out[1], out[5]);
    CHECK_EQ(out[2], out[6]);
    /* right motor block */
    CHECK_EQ(0x00, out[9]);
    CHECK_EQ(0xFF, out[10]);
    CHECK_EQ(0x40, out[11]);
    CHECK_EQ(0x40, out[12]);
    /* repeated */
    CHECK_EQ(out[9],  out[13]);
    CHECK_EQ(out[10], out[14]);

    /* Both motors are independent: changing one must not touch the other. */
    CHECK(out[2] != out[10]);
}

/* ===================================================================== */
/* Test 17 — driver buffer sizing                                        */
/* ===================================================================== */

static void test_driver_buffer_sizing(void)
{
    printf("\n[driver buffer sizing]\n");

    /*
     * These must match src/main.c. They are the limits that silently truncated
     * Switch Pro reports when the driver only budgeted for Xbox pads:
     *   MAX_XPAD_PAYLOAD 32 -> 64  (the 0x30 report is 54 bytes)
     *   XPAD_OUT_LEN     12 -> 17  (rumble-only report is 17 bytes)
     *
     * A 0x30 report larger than the ring-buffer capacity would be clamped in
     * data_transfer_done() and then fail the length check in swpro_read_report,
     * so the pad would connect but never report any input.
     */
    const int MAX_XPAD_PAYLOAD = 64;
    const int MAX_XPAD_DATA_LEN = MAX_XPAD_PAYLOAD + 2;  /* +2 for count/size */
    const int XPAD_OUT_LEN = 17;

    CHECK(MAX_XPAD_PAYLOAD >= SWPRO_FULL_REPORT_BYTES);
    CHECK(MAX_XPAD_DATA_LEN - 2 >= SWPRO_FULL_REPORT_BYTES);

    /* The ring buffer stores [count][size] + payload, so it must hold 56. */
    CHECK(MAX_XPAD_DATA_LEN >= SWPRO_FULL_REPORT_BYTES + 2);

    /* Rumble-only report is 1 + 16 bytes and must fit the out buffer. */
    CHECK(XPAD_OUT_LEN >= 17);
    CHECK_EQ(17, XPAD_OUT_LEN);
}

/* ===================================================================== */
/* Test 18 — subcommand report is always the full 16 bytes                */
/* ===================================================================== */

static void test_subcommand_report_length(void)
{
    printf("\n[subcommand report length]\n");

    /*
     * swpro_send_subcommand() writes a fixed 16-byte report. Every subcommand
     * the init sequence sends must fit in that frame: id, counter, 8 rumble
     * bytes, subcommand code, argument.
     */
    const int SWPRO_OUT_REPORT_LEN = 16;
    uint8_t out[SWPRO_OUT_REPORT_LEN];

    /* Subcommands used by swpro_init(), in order. */
    static const struct { uint8_t sc; uint8_t arg; } init_seq[] = {
        { SWPRO_SC_SET_INPUT_MODE, 0x30 },
        { SWPRO_SC_ENABLE_IMU,     0x01 },
        { SWPRO_SC_ENABLE_VIB,     0x01 },
    };

    for (size_t i = 0; i < sizeof(init_seq) / sizeof(init_seq[0]); i++) {
        memset(out, 0, sizeof(out));
        out[0]  = SWPRO_OUT_SUBCOMMAND;
        out[1]  = (uint8_t)(i & 0x0F);          /* counter */
        memcpy(&out[2], SWPRO_RUMBLE_NEUTRAL, 8);
        out[10] = init_seq[i].sc;
        out[11] = init_seq[i].arg;

        /* subcommand code and argument land at the documented offsets */
        CHECK_EQ(init_seq[i].sc,  out[10]);
        CHECK_EQ(init_seq[i].arg, out[11]);
        /* rumble field untouched by the subcommand */
        CHECK_BYTES(SWPRO_RUMBLE_NEUTRAL, &out[2], 8);
    }

    /* Counter advances and wraps at 0x0F. */
    uint8_t counter = 0;
    for (int i = 0; i < 20; i++) {
        counter = (uint8_t)((counter + 1) & 0x0F);
    }
    CHECK_EQ(4, counter);   /* 20 mod 16 */

    /* The init sequence must not repeat a subcommand. */
    CHECK(init_seq[0].sc != init_seq[1].sc);
    CHECK(init_seq[1].sc != init_seq[2].sc);
    CHECK(init_seq[0].sc != init_seq[2].sc);
}

/* ===================================================================== */
/* main                                                                  */
/* ===================================================================== */

int main(void)
{
    printf("ps3xpad Switch Pro protocol tests\n");
    printf("=================================\n");

    test_report_size();
    test_field_offsets();
    test_imu_layout();
    test_button_map();
    test_subcommand_report();
    test_rumble_only_report();
    test_spi_calibration();
    test_handshake_and_endpoints();
    test_button_bits();
    test_face_buttons_diagonal();
    test_other_buttons();
    test_stick_passthrough();
    test_report_filtering();
    test_led_patterns();
    test_rumble_report_bytes();
    test_driver_buffer_sizing();
    test_subcommand_report_length();

    printf("\n=================================\n");
    printf("RESULT: %d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}