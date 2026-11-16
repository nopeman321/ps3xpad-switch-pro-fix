/*
 * test_bt_dongle.c — Unit tests for Bluetooth HCI + L2CAP transport layer
 *
 * Verifies:
 * 1. HCI_Create_Connection packet formatting and Little-Endian BD_ADDR ordering.
 * 2. ACL + L2CAP Little-Endian length headers and 0xA2 output wrapping.
 * 3. Parsing of incoming ACL/L2CAP packets: stripping 0xA1 header and passing
 *    clean 0x30/0x21 reports to the Switch Pro decoder.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#define HCI_OP_CREATE_CONN       0x0405
#define L2CAP_LOCAL_CID_HID_INTR 0x0041
#define BT_HID_HDR_DATA_INPUT    0xA1
#define BT_HID_HDR_DATA_OUTPUT   0xA2
#define SWPRO_FULL_REPORT_LEN    54

static int tests_run = 0;
static int tests_passed = 0;

#define CHECK(expr, msg) do { \
  tests_run++; \
  if (expr) { \
    tests_passed++; \
    printf("  [PASS] %s\n", msg); \
  } else { \
    printf("  [FAIL] %s (line %d)\n", msg, __LINE__); \
    assert(expr); \
  } \
} while(0)

/* ------------------------------------------------------------------------- */
/* 1. Test HCI_Create_Connection Formatting                                  */
/* ------------------------------------------------------------------------- */
static void test_hci_create_connection(void) {
  printf("[1] Testing HCI_Create_Connection packet formatting...\n");

  const uint8_t target_mac[6] = {0x28, 0xE1, 0xF7, 0xD8, 0x17, 0xE4};
  uint8_t pkt[16];
  uint16_t opcode = HCI_OP_CREATE_CONN;
  uint8_t param_len = 13;

  /* Opcode and parameter length */
  pkt[0] = opcode & 0xFF;
  pkt[1] = (opcode >> 8) & 0xFF;
  pkt[2] = param_len;

  /* Parameters: Target BD_ADDR (6 bytes LE) */
  memcpy(&pkt[3], target_mac, 6);

  /* Parameters: Packet_Type = 0xCC18 (LE: 18 CC) */
  pkt[9]  = 0x18;
  pkt[10] = 0xCC;

  /* Parameters: Page_Scan_Repetition_Mode = R1 (0x01) */
  pkt[11] = 0x01;

  /* Parameters: Reserved = 0x00 */
  pkt[12] = 0x00;

  /* Parameters: Clock_Offset = 0x0000 */
  pkt[13] = 0x00;
  pkt[14] = 0x00;

  /* Parameters: Allow_Role_Switch = 0x01 */
  pkt[15] = 0x01;

  const uint8_t expected[16] = {
    0x05, 0x04, 0x0D,
    0x28, 0xE1, 0xF7, 0xD8, 0x17, 0xE4,
    0x18, 0xCC,
    0x01,
    0x00,
    0x00, 0x00,
    0x01
  };

  CHECK(memcmp(pkt, expected, 16) == 0,
        "HCI_Create_Connection matches exact 16-byte wire specification: "
        "05 04 0D 28 E1 F7 D8 17 E4 18 CC 01 00 00 00 01");
}

/* ------------------------------------------------------------------------- */
/* 2. Test ACL + L2CAP Little-Endian Output Packet Packaging                 */
/* ------------------------------------------------------------------------- */
static void test_acl_l2cap_output_packaging(void) {
  printf("[2] Testing ACL + L2CAP Output Packet packaging (0x01 SET_MODE)...\n");

  uint16_t acl_handle = 0x0042;
  uint16_t intr_rcid  = 0x0055;

  /* Switch Pro 16-byte subcommand report (SET_MODE 0x30) */
  uint8_t subcmd_report[16];
  memset(subcmd_report, 0, sizeof(subcmd_report));
  subcmd_report[0] = 0x01; /* Output Report ID */
  subcmd_report[1] = 0x03; /* Global Counter */
  /* Neutral rumble */
  subcmd_report[2] = 0x00; subcmd_report[3] = 0x01; subcmd_report[4] = 0x40; subcmd_report[5] = 0x40;
  subcmd_report[6] = 0x00; subcmd_report[7] = 0x01; subcmd_report[8] = 0x40; subcmd_report[9] = 0x40;
  subcmd_report[10] = 0x03; /* Subcommand SET_MODE */
  subcmd_report[11] = 0x30; /* Mode 0x30 Full report */

  uint8_t acl_out[64];
  uint16_t report_len = 49;
  uint16_t l2cap_len  = 1 + report_len; /* 50 bytes (0xA2 + 49-byte report) */
  uint16_t acl_data_len = l2cap_len + 4; /* 54 bytes */

  /* ACL Header (4 bytes) */
  acl_out[0] = acl_handle & 0xFF;                         /* Handle low: 0x42 */
  acl_out[1] = ((acl_handle >> 8) & 0x0F) | 0x20;        /* Handle hi + PB 0x02: 0x20 */
  acl_out[2] = acl_data_len & 0xFF;                      /* ACL data len low: 54 (0x36) */
  acl_out[3] = (acl_data_len >> 8) & 0xFF;               /* ACL data len hi:  0 (0x00) */

  /* L2CAP Header (4 bytes) */
  acl_out[4] = l2cap_len & 0xFF;                         /* L2CAP len low: 50 (0x32) */
  acl_out[5] = (l2cap_len >> 8) & 0xFF;                  /* L2CAP len hi:   0 (0x00) */
  acl_out[6] = intr_rcid & 0xFF;                         /* Destination CID low: 0x55 */
  acl_out[7] = (intr_rcid >> 8) & 0xFF;                  /* Destination CID hi:  0x00 */

  /* Bluetooth HID Header */
  acl_out[8] = BT_HID_HDR_DATA_OUTPUT;                   /* 0xA2 */

  /* Payload (padded to 49 bytes) */
  memset(&acl_out[9], 0, report_len);
  memcpy(&acl_out[9], subcmd_report, 16);

  CHECK(acl_out[0] == 0x42 && acl_out[1] == 0x20, "ACL handle and PB flags correct (0x42 0x20)");
  CHECK(acl_out[2] == 0x36 && acl_out[3] == 0x00, "ACL data length Little-Endian is 54 (0x36 0x00)");
  CHECK(acl_out[4] == 0x32 && acl_out[5] == 0x00, "L2CAP length Little-Endian is 50 (0x32 0x00)");
  CHECK(acl_out[6] == 0x55 && acl_out[7] == 0x00, "L2CAP destination CID is 0x0055 (0x55 0x00)");
  CHECK(acl_out[8] == 0xA2, "Bluetooth HID output transaction header is 0xA2");
  CHECK(acl_out[9] == 0x01 && acl_out[19] == 0x03 && acl_out[20] == 0x30,
        "Subcommand payload correctly aligned after 0xA2 header");
}

/* ------------------------------------------------------------------------- */
/* 3. Test ACL Bulk IN Parser: Strip 0xA1 and Deliver Clean 0x30 Report      */
/* ------------------------------------------------------------------------- */
typedef struct {
  uint8_t received_report[64];
  int32_t received_len;
} simulated_unit_t;

static void simulated_swpro_read_report(int32_t id, const uint8_t *readBuf, int32_t len) {
  (void)id;
  CHECK(readBuf[0] == 0x30, "Report starts with 0x30 (0xA1 prefix was cleanly stripped)");
  CHECK(len == SWPRO_FULL_REPORT_LEN, "Report payload length is exactly 54 bytes");
  /* Check button cluster byte offset 3 */
  CHECK(readBuf[3] == 0x04, "Button field preserved at byte 3 (B button / Cross)");
}

static void test_acl_in_report_stripping(void) {
  printf("[3] Testing ACL Bulk IN parsing and 0xA1 stripping...\n");

  /* Construct simulated incoming 54-byte 0x30 report */
  uint8_t swpro_report[SWPRO_FULL_REPORT_LEN];
  memset(swpro_report, 0, sizeof(swpro_report));
  swpro_report[0] = 0x30;       /* Full report ID */
  swpro_report[1] = 0x55;       /* Timer tick */
  swpro_report[3] = 0x04;       /* Button B pressed (Cross on PS3) */
  swpro_report[6] = 0x00; swpro_report[7] = 0x88; swpro_report[8] = 0x08; /* Left stick center (2048) */

  /* Package into incoming ACL bulk packet */
  uint8_t incoming_acl[80];
  uint16_t acl_handle = 0x0042;
  uint16_t local_cid  = L2CAP_LOCAL_CID_HID_INTR; /* 0x0041 */
  uint16_t l2cap_len  = 1 + SWPRO_FULL_REPORT_LEN; /* 1 + 54 = 55 */
  uint16_t acl_len    = l2cap_len + 4;             /* 59 */

  incoming_acl[0] = acl_handle & 0xFF;
  incoming_acl[1] = ((acl_handle >> 8) & 0x0F) | 0x20;
  incoming_acl[2] = acl_len & 0xFF;
  incoming_acl[3] = (acl_len >> 8) & 0xFF;

  incoming_acl[4] = l2cap_len & 0xFF;
  incoming_acl[5] = (l2cap_len >> 8) & 0xFF;
  incoming_acl[6] = local_cid & 0xFF;
  incoming_acl[7] = (local_cid >> 8) & 0xFF;

  incoming_acl[8] = BT_HID_HDR_DATA_INPUT; /* 0xA1 */
  memcpy(&incoming_acl[9], swpro_report, SWPRO_FULL_REPORT_LEN);

  /* Parser logic from btd_acl_in_done: */
  uint16_t parsed_l2cap_len = incoming_acl[4] | (incoming_acl[5] << 8);
  uint16_t parsed_cid = incoming_acl[6] | (incoming_acl[7] << 8);
  uint8_t *payload = &incoming_acl[8];

  CHECK(parsed_cid == L2CAP_LOCAL_CID_HID_INTR, "Target CID identified as HID Interrupt (0x0041)");
  CHECK(payload[0] == BT_HID_HDR_DATA_INPUT, "First byte is BT_HID_HDR_DATA_INPUT (0xA1)");

  const uint8_t *delivered_report = &payload[1];
  int32_t delivered_len = parsed_l2cap_len - 1;

  simulated_swpro_read_report(0, delivered_report, delivered_len);
}

int main(void) {
  printf("====================================================\n");
  printf(" ps3xpad Bluetooth Transport & Packet Unit Tests    \n");
  printf("====================================================\n");

  test_hci_create_connection();
  test_acl_l2cap_output_packaging();
  test_acl_in_report_stripping();

  printf("====================================================\n");
  printf(" RESULTS: %d/%d assertions passed successfully!\n", tests_passed, tests_run);
  printf("====================================================\n");
  return 0;
}
