#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ppu_thread.h>
#include <sys/synchronization.h>
#include <sys/timer.h>
#include <sys/types.h>
#include <cell/usbd.h>
#include <cell/pad.h>
#include <cell/cell_fs.h>
#include "bt_dongle.h"

/* External hooks implemented in src/main.c */
extern void xlog(const char *text);
extern void xlog_code(const char *what, int32_t code);
extern void xlog_dev(const char *what, uint16_t vid, uint16_t pid, int32_t code);
extern int32_t btd_is_custom_dongle(uint16_t vid, uint16_t pid);
extern void *btd_create_switch_unit(void);
extern void btd_destroy_switch_unit(void *u);
extern void btd_push_report(void *u, const uint8_t *report, int32_t len);
extern void btd_start_swpro_init(void *u);

#define SWAP16(x) (((x) >> 8) | (((x) & 0xff) << 8))

#define BTD_CMD_QUEUE_LEN 8

typedef struct {
  uint16_t opcode;
  uint8_t len;
  uint8_t data[24];
} btd_cmd_item_t;

typedef enum {
  BTD_STATE_IDLE = 0,
  BTD_STATE_ATTACHED,
  BTD_STATE_RESET,
  BTD_STATE_RTL_READ_ROM,
  BTD_STATE_RTL_DOWNLOAD,
  BTD_STATE_READ_BD_ADDR,
  BTD_STATE_WRITE_SSP,
  BTD_STATE_WRITE_COD,
  BTD_STATE_WRITE_SCAN_ENABLE,
  BTD_STATE_READY,
  BTD_STATE_CONNECTING,
  BTD_STATE_INQUIRY,
  BTD_STATE_INQUIRY_CANCEL,
  BTD_STATE_ACL_CONNECTED,
  BTD_STATE_L2CAP_OPENING_CTRL,
  BTD_STATE_L2CAP_CONFIG_CTRL,
  BTD_STATE_L2CAP_OPENING_INTR,
  BTD_STATE_L2CAP_CONFIG_INTR,
  BTD_STATE_HID_ACTIVE
} btd_fsm_state_t;

typedef struct {
  int32_t dev_id;
  int32_t c_pipe;
  int32_t i_pipe;
  int32_t bulk_in_pipe;
  int32_t bulk_out_pipe;

  uint16_t vid;
  uint16_t pid;

  uint8_t is_realtek;
  uint8_t rtl_rom_ver;
  uint8_t rtl_fw_loaded;

  btd_fsm_state_t state;
  int32_t timer;
  int32_t conn_timer;

  uint8_t dongle_bdaddr[6];
  uint8_t target_bdaddr[6];

  uint16_t acl_handle;
  uint16_t ctrl_rcid;
  uint16_t intr_rcid;

  uint8_t local_conf_ctrl;
  uint8_t remote_conf_ctrl;
  uint8_t local_conf_intr;
  uint8_t remote_conf_intr;
  uint8_t l2cap_ident;

  /* Transfer buffers and in-flight guards */
  UsbDeviceRequest cmd_req;
  uint8_t cmd_buf[HCI_CMD_BUF_SIZE];
  volatile uint8_t cmd_busy;
  volatile uint8_t cmd_complete;

  uint8_t evt_buf[HCI_EVT_BUF_SIZE];
  volatile uint8_t evt_busy;

  uint8_t acl_in_buf[HCI_ACL_BUF_SIZE];
  volatile uint8_t acl_in_busy;

  uint8_t acl_out_buf[HCI_ACL_BUF_SIZE];
  volatile uint8_t bulk_out_busy;

  /* Asynchronous HCI command queue */
  btd_cmd_item_t cmd_queue[BTD_CMD_QUEUE_LEN];
  uint8_t cmd_q_head;
  uint8_t cmd_q_tail;

  /* Attached virtual pad unit */
  void *swpro_unit;

  volatile uint8_t closing;
} btd_context_t;

static btd_context_t btd;

/* Realtek firmware download state */
static int32_t s_rtl_fd = -1;
static uint32_t s_rtl_fw_rem = 0;
static uint32_t s_rtl_frag_total = 0;
static uint32_t s_rtl_frag_curr = 0;
static uint8_t s_rtl_j = 0;

/* Default MAC from captured log (Little-Endian: 28 E1 F7 D8 17 E4 = E4:17:D8:F7:E1:28) */
static const uint8_t s_default_target_mac[6] = {0x28, 0xE1, 0xF7, 0xD8, 0x17, 0xE4};

/* Forward declarations */
static void btd_evt_done(int32_t result, int32_t count, void *arg);
static void btd_acl_in_done(int32_t result, int32_t count, void *arg);
static void btd_bulk_out_done(int32_t result, int32_t count, void *arg);
static void btd_cmd_done(int32_t result, int32_t count, void *arg);
static void btd_set_config_done(int32_t result, int32_t count, void *arg);
static int32_t btd_send_cmd_direct(uint16_t opcode, uint8_t param_len, const uint8_t *params);
static void btd_queue_cmd(uint16_t opcode, uint8_t len, const uint8_t *params);
static void btd_process_cmd_queue(void);
static void btd_rtl_init_firmware(void);
static void btd_rtl_send_next_chunk(void);
static void btd_send_l2cap_conn_req(uint16_t psm, uint16_t scid);
static void btd_send_l2cap_conf_req(uint16_t dcid);
static void btd_send_l2cap_conf_rsp(uint8_t ident, uint16_t scid);

static void btd_load_mac(void) {
  int32_t fd;
  uint64_t nread = 0;
  memcpy(btd.target_bdaddr, s_default_target_mac, 6);
  if (cellFsOpen("/dev_hdd0/plugins/xpad_bt_mac.bin", CELL_FS_O_RDONLY, &fd, NULL, 0) == CELL_FS_SUCCEEDED) {
    if (cellFsRead(fd, btd.target_bdaddr, 6, &nread) == CELL_FS_SUCCEEDED && nread == 6) {
      xlog("BTD: Loaded controller MAC from /dev_hdd0/plugins/xpad_bt_mac.bin");
    }
    cellFsClose(fd);
  }
}

void btd_learn_controller_mac(const uint8_t mac_le[6]) {
  int32_t fd;
  uint64_t nwritten = 0;
  if (mac_le == NULL) return;
  memcpy(btd.target_bdaddr, mac_le, 6);
  if (cellFsOpen("/dev_hdd0/plugins/xpad_bt_mac.bin", CELL_FS_O_WRONLY | CELL_FS_O_CREAT | CELL_FS_O_TRUNC, &fd, NULL, 0) == CELL_FS_SUCCEEDED) {
    cellFsWrite(fd, btd.target_bdaddr, 6, &nwritten);
    cellFsClose(fd);
    xlog("BTD: Saved learned controller MAC to xpad_bt_mac.bin");
  }
  /* Immediately attempt direct wireless connection with newly learned MAC */
  if (btd.state >= BTD_STATE_READY && btd.state < BTD_STATE_ACL_CONNECTED) {
    btd.state = BTD_STATE_READY;
    btd.timer = 2;
  }
}

int32_t btd_probe(int32_t dev_id) {
  UsbDeviceDescriptor *ddesc;
  uint16_t vid, pid;

  ddesc = (UsbDeviceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_DEVICE);
  if (ddesc == NULL) {
    xlog("BTD probe rejected: no device descriptor");
    return CELL_USBD_PROBE_FAILED;
  }

  vid = SWAP16(ddesc->idVendor);
  pid = SWAP16(ddesc->idProduct);
  xlog_dev("BTD probe entered for", vid, pid, 0);

  /* CRITICAL: Never claim Sony Bluetooth (0x054C:xxxx) to protect DualShock 3 */
  if (vid == 0x054C) {
    return CELL_USBD_PROBE_FAILED;
  }

  /* 1. Hardware Dongle detected in Phase 1 (Realtek RTL8761BU Bluetooth 5.3) */
  if (vid == 0x0BDA && pid == 0xA729) {
    xlog_dev("BTD probe matched (Realtek 5.3 dongle)", vid, pid, 0);
    return CELL_USBD_PROBE_SUCCEEDED;
  }

  /* 2. CSR8510 standard dongle */
  if (vid == 0x0A12 && pid == 0x0001) {
    xlog_dev("BTD probe matched (CSR8510)", vid, pid, 0);
    return CELL_USBD_PROBE_SUCCEEDED;
  }

  /* 3. Broadcom BCM20702 standard dongle */
  if (vid == 0x0A5C && pid == 0x21E8) {
    xlog_dev("BTD probe matched (Broadcom BCM20702)", vid, pid, 0);
    return CELL_USBD_PROBE_SUCCEEDED;
  }

  /* 4. Custom entries from xpad_devices.txt marked with XTYPE_BTD */
  if (btd_is_custom_dongle(vid, pid)) {
    xlog_dev("BTD probe matched (xpad_devices.txt)", vid, pid, 0);
    return CELL_USBD_PROBE_SUCCEEDED;
  }

  xlog_dev("BTD probe rejected: VID/PID not a recognized dongle", vid, pid, 0);
  return CELL_USBD_PROBE_FAILED;
}

int32_t btd_attach(int32_t dev_id) {
  uint8_t *desc = NULL;
  UsbDeviceDescriptor *ddesc;
  UsbConfigurationDescriptor *cdesc;
  UsbInterfaceDescriptor *ifd, *bt_if = NULL;
  UsbEndpointDescriptor *ed;
  UsbEndpointDescriptor *ep_event = NULL;
  UsbEndpointDescriptor *ep_bulk_in = NULL;
  UsbEndpointDescriptor *ep_bulk_out = NULL;
  uint16_t vid = 0, pid = 0;
  int32_t r;

  ddesc = (UsbDeviceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_DEVICE);
  if (ddesc != NULL) {
    vid = SWAP16(ddesc->idVendor);
    pid = SWAP16(ddesc->idProduct);
  }

  cdesc = (UsbConfigurationDescriptor *)cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_CONFIGURATION);
  if (cdesc == NULL) {
    xlog("BTD attach failed: no configuration descriptor");
    return CELL_USBD_ATTACH_FAILED;
  }

  /* Find Bluetooth interface (Class 0xE0, Subclass 0x01, Protocol 0x01) */
  while ((desc = (uint8_t *)cellUsbdScanStaticDescriptor(dev_id, desc, 0)) != NULL) {
    if (desc[1] == USB_DESCRIPTOR_TYPE_INTERFACE) {
      ifd = (UsbInterfaceDescriptor *)desc;
      if (ifd->bInterfaceClass == USB_CLASS_WIRELESS_CONTROLLER &&
          ifd->bInterfaceSubclass == USB_SUBCLASS_RF_CONTROLLER &&
          ifd->bInterfaceProtocol == USB_PROTOCOL_BLUETOOTH_PROGRAMMING) {
        bt_if = ifd;
      }
    } else if (desc[1] == USB_DESCRIPTOR_TYPE_ENDPOINT && bt_if != NULL) {
      ed = (UsbEndpointDescriptor *)desc;
      uint8_t attr = ed->bmAttributes & 0x03;
      uint8_t ep_addr = ed->bEndpointAddress;

      if (attr == USB_ENDPOINT_TRANSFER_TYPE_INTERRUPT && (ep_addr & 0x80)) {
        if (ep_event == NULL) ep_event = ed;
      } else if (attr == USB_ENDPOINT_TRANSFER_TYPE_BULK) {
        if ((ep_addr & 0x80) && ep_bulk_in == NULL) {
          ep_bulk_in = ed;
        } else if (!(ep_addr & 0x80) && ep_bulk_out == NULL) {
          ep_bulk_out = ed;
        }
      }
    }
  }

  if (ep_event == NULL || ep_bulk_in == NULL || ep_bulk_out == NULL) {
    xlog("BTD attach failed: missing HCI endpoints (interrupt/bulk)");
    return CELL_USBD_ATTACH_FAILED;
  }

  memset(&btd, 0, sizeof(btd));
  btd.dev_id = dev_id;
  btd.vid = vid;
  btd.pid = pid;
  btd.acl_handle = 0xFFFF;
  btd.l2cap_ident = 1;

  btd_load_mac();

  /* Open Pipes */
  btd.c_pipe = cellUsbdOpenPipe(dev_id, NULL);
  btd.i_pipe = cellUsbdOpenPipe(dev_id, ep_event);
  btd.bulk_in_pipe = cellUsbdOpenPipe(dev_id, ep_bulk_in);
  btd.bulk_out_pipe = cellUsbdOpenPipe(dev_id, ep_bulk_out);

  if (btd.c_pipe < 0 || btd.i_pipe < 0 || btd.bulk_in_pipe < 0 || btd.bulk_out_pipe < 0) {
    xlog("BTD attach failed: pipe opening error");
    btd_detach(dev_id);
    return CELL_USBD_ATTACH_FAILED;
  }

  cellUsbdSetPrivateData(dev_id, &btd);

  btd.vid = vid;
  btd.pid = pid;
  if (vid == 0x0BDA) {
    btd.is_realtek = 1;
  } else {
    btd.is_realtek = 0;
  }

  r = cellUsbdSetConfiguration(btd.c_pipe, cdesc->bConfigurationValue, btd_set_config_done, &btd);
  if (r != CELL_OK) {
    xlog_code("BTD attach: set_configuration submit failed", r);
    btd_detach(dev_id);
    return CELL_USBD_ATTACH_FAILED;
  }

  xlog_dev("BTD attach succeeded for", vid, pid, 0);
  return CELL_USBD_ATTACH_SUCCEEDED;
}

static void btd_set_config_done(int32_t result, int32_t count, void *arg) {
  if (result != CELL_OK || btd.closing) {
    xlog_code("BTD set_configuration callback failed", result);
    return;
  }
  btd.state = BTD_STATE_ATTACHED;

  /* Start continuous HCI event listening on i_pipe */
  btd.evt_busy = 1;
  cellUsbdInterruptTransfer(btd.i_pipe, btd.evt_buf, sizeof(btd.evt_buf), btd_evt_done, &btd);
}

int32_t btd_detach(int32_t dev_id) {
  btd.closing = 1;

  if (s_rtl_fd >= 0) {
    cellFsClose(s_rtl_fd);
    s_rtl_fd = -1;
  }
  s_rtl_fw_rem = 0;
  s_rtl_frag_total = 0;
  s_rtl_frag_curr = 0;
  s_rtl_j = 0;

  if (btd.swpro_unit != NULL) {
    btd_destroy_switch_unit(btd.swpro_unit);
    btd.swpro_unit = NULL;
  }

  if (btd.c_pipe >= 0) cellUsbdClosePipe(btd.c_pipe);
  if (btd.i_pipe >= 0) cellUsbdClosePipe(btd.i_pipe);
  if (btd.bulk_in_pipe >= 0) cellUsbdClosePipe(btd.bulk_in_pipe);
  if (btd.bulk_out_pipe >= 0) cellUsbdClosePipe(btd.bulk_out_pipe);

  btd.c_pipe = -1;
  btd.i_pipe = -1;
  btd.bulk_in_pipe = -1;
  btd.bulk_out_pipe = -1;
  btd.state = BTD_STATE_IDLE;
  btd.rtl_fw_loaded = 0;

  cellUsbdSetPrivateData(dev_id, NULL);
  xlog("BTD detached cleanly");
  return CELL_USBD_DETACH_SUCCEEDED;
}

int32_t btd_detach_all(void) {
  if (btd.dev_id > 0) {
    return btd_detach(btd.dev_id);
  }
  return CELL_USBD_DETACH_SUCCEEDED;
}

/* HCI Command transmission on Control Pipe EP0 */
static void btd_cmd_done(int32_t result, int32_t count, void *arg) {
  btd.cmd_busy = 0;
  if (result != CELL_OK) {
    xlog_code("BTD control command transfer error", result);
  }
  btd_process_cmd_queue();
}

static int32_t btd_send_cmd_direct(uint16_t opcode, uint8_t param_len, const uint8_t *params) {
  if (btd.cmd_busy || btd.c_pipe < 0) {
    return -1;
  }
  btd.cmd_buf[0] = opcode & 0xFF;
  btd.cmd_buf[1] = (opcode >> 8) & 0xFF;
  btd.cmd_buf[2] = param_len;
  if (param_len > 0 && params != NULL) {
    memcpy(&btd.cmd_buf[3], params, param_len);
  }

  btd.cmd_req.bmRequestType = HCI_USB_REQTYPE_CMD;
  btd.cmd_req.bRequest = HCI_USB_BREQUEST_CMD;
  btd.cmd_req.wValue = 0;
  btd.cmd_req.wIndex = 0;
  btd.cmd_req.wLength = 3 + param_len;

  btd.cmd_busy = 1;
  btd.cmd_complete = 0;

  int32_t r = cellUsbdControlTransfer(btd.c_pipe, &btd.cmd_req, btd.cmd_buf, btd_cmd_done, &btd);
  if (r != CELL_OK) {
    btd.cmd_busy = 0;
    xlog_code("BTD submit control transfer failed", r);
    return r;
  }
  return CELL_OK;
}

static void btd_queue_cmd(uint16_t opcode, uint8_t len, const uint8_t *params) {
  uint8_t next = (btd.cmd_q_head + 1) % BTD_CMD_QUEUE_LEN;
  if (next == btd.cmd_q_tail) {
    xlog("BTD cmd queue full, dropping command");
    return;
  }
  btd.cmd_queue[btd.cmd_q_head].opcode = opcode;
  btd.cmd_queue[btd.cmd_q_head].len = len;
  if (len > 0 && params != NULL) {
    memcpy(btd.cmd_queue[btd.cmd_q_head].data, params, len > 24 ? 24 : len);
  }
  btd.cmd_q_head = next;
}

static void btd_process_cmd_queue(void) {
  if (btd.cmd_busy || btd.c_pipe < 0) return;
  if (btd.cmd_q_head != btd.cmd_q_tail) {
    btd_cmd_item_t *item = &btd.cmd_queue[btd.cmd_q_tail];
    if (btd_send_cmd_direct(item->opcode, item->len, item->data) == CELL_OK) {
      btd.cmd_q_tail = (btd.cmd_q_tail + 1) % BTD_CMD_QUEUE_LEN;
    }
  }
}

#define RTL_FRAG_LEN 252

static void btd_rtl_send_next_chunk(void) {
  if (s_rtl_fd < 0) return;

  if (s_rtl_frag_curr >= s_rtl_frag_total) {
    /* Firmware download complete! */
    cellFsClose(s_rtl_fd);
    s_rtl_fd = -1;
    xlog("BTD: Realtek firmware patch successfully downloaded to dongle RAM!");
    btd.rtl_fw_loaded = 1;
    btd.cmd_complete = 1;
    btd.state = BTD_STATE_RESET;
    return;
  }

  uint8_t dl_buf[260];
  uint8_t index = s_rtl_j++;
  if (index == 0x7F) s_rtl_j = 1;

  uint16_t frag_len = RTL_FRAG_LEN;
  if (s_rtl_frag_curr == s_rtl_frag_total - 1) {
    index |= 0x80; /* Data End */
    frag_len = s_rtl_fw_rem;
  }

  dl_buf[0] = index;
  uint64_t nread = 0;
  cellFsRead(s_rtl_fd, &dl_buf[1], frag_len, &nread);
  s_rtl_fw_rem -= (uint32_t)nread;

  btd_send_cmd_direct(0xFC20, 1 + frag_len, dl_buf);
  s_rtl_frag_curr++;
}

static void btd_rtl_init_firmware(void) {
  uint64_t nread = 0;
  uint64_t pos = 0;
  uint8_t hdr[14];

  s_rtl_fd = -1;
  if (cellFsOpen("/dev_hdd0/plugins/rtl8761bu_fw.bin", CELL_FS_O_RDONLY, &s_rtl_fd, NULL, 0) != CELL_FS_SUCCEEDED) {
    xlog("BTD: Realtek firmware /dev_hdd0/plugins/rtl8761bu_fw.bin not found!");
    btd.rtl_fw_loaded = 1; /* Skip */
    btd.state = BTD_STATE_READ_BD_ADDR;
    return;
  }

  if (cellFsRead(s_rtl_fd, hdr, 14, &nread) != CELL_FS_SUCCEEDED || nread != 14) {
    xlog("BTD: Failed reading RTL firmware header");
    cellFsClose(s_rtl_fd);
    s_rtl_fd = -1;
    btd.rtl_fw_loaded = 1;
    btd.state = BTD_STATE_READ_BD_ADDR;
    return;
  }

  if (memcmp(hdr, "Realtech", 8) != 0) {
    xlog("BTD: Invalid Realtek EPATCH signature");
    cellFsClose(s_rtl_fd);
    s_rtl_fd = -1;
    btd.rtl_fw_loaded = 1;
    btd.state = BTD_STATE_READ_BD_ADDR;
    return;
  }

  uint16_t num_patches = hdr[12] | (hdr[13] << 8);
  xlog_code("BTD: RTL num_patches", (int32_t)num_patches);

  /* Read patch metadata */
  uint8_t meta[128];
  uint32_t meta_len = num_patches * 8;
  if (meta_len > sizeof(meta) || cellFsRead(s_rtl_fd, meta, meta_len, &nread) != CELL_FS_SUCCEEDED || nread != meta_len) {
    xlog("BTD: Failed reading RTL patch metadata");
    cellFsClose(s_rtl_fd);
    s_rtl_fd = -1;
    btd.rtl_fw_loaded = 1;
    btd.state = BTD_STATE_READ_BD_ADDR;
    return;
  }

  uint32_t patch_offset = 0;
  uint16_t patch_length = 0;
  uint16_t target_chip_id = btd.rtl_rom_ver + 1;

  for (int i = 0; i < num_patches; i++) {
    uint16_t cid = meta[i * 2] | (meta[i * 2 + 1] << 8);
    if (cid == target_chip_id) {
      patch_length = meta[num_patches * 2 + i * 2] | (meta[num_patches * 2 + i * 2 + 1] << 8);
      uint8_t *po = &meta[num_patches * 4 + i * 4];
      patch_offset = po[0] | (po[1] << 8) | (po[2] << 16) | (po[3] << 24);
      break;
    }
  }

  if (patch_offset == 0 || patch_length == 0) {
    xlog_code("BTD: No matching RTL patch for chip_id", (int32_t)target_chip_id);
    cellFsClose(s_rtl_fd);
    s_rtl_fd = -1;
    btd.rtl_fw_loaded = 1;
    btd.state = BTD_STATE_READ_BD_ADDR;
    return;
  }

  xlog_code("BTD: Loading RTL patch length", (int32_t)patch_length);
  cellFsLseek(s_rtl_fd, patch_offset, CELL_FS_SEEK_SET, &pos);

  s_rtl_fw_rem = patch_length;
  s_rtl_frag_total = (patch_length / RTL_FRAG_LEN) + 1;
  s_rtl_frag_curr = 0;
  s_rtl_j = 0;

  btd_rtl_send_next_chunk();
}

static void btd_log_mac_addr(const char *prefix, const uint8_t *mac) {
  static const char hex[] = "0123456789ABCDEF";
  char buf[80];
  char *m = buf;
  int i;
  for (i = 0; prefix[i]; i++) *m++ = prefix[i];
  for (i = 5; i >= 0; i--) {
    *m++ = hex[(mac[i] >> 4) & 0x0F];
    *m++ = hex[mac[i] & 0x0F];
    if (i > 0) *m++ = ':';
  }
  *m = 0;
  xlog(buf);
}

/* HCI Event Reception on Interrupt IN Pipe EP 0x81 */
static void btd_evt_done(int32_t result, int32_t count, void *arg) {
  btd.evt_busy = 0;
  if (result != CELL_OK || btd.closing) {
    return;
  }

  if (count >= 2) {
    uint8_t evt_code = btd.evt_buf[0];
    uint8_t *p = &btd.evt_buf[2];

    switch (evt_code) {
      case HCI_EVT_COMMAND_COMPLETE: {
        if (count >= 5) {
          uint16_t op = p[1] | (p[2] << 8);
          uint8_t status = p[3];

          if (op == HCI_OP_RESET && status == 0) {
            btd.cmd_complete = 1;
            if (btd.is_realtek && !btd.rtl_fw_loaded) {
              btd.state = BTD_STATE_RTL_READ_ROM;
            } else {
              btd.state = BTD_STATE_READ_BD_ADDR;
            }
          } else if (op == 0xFC6D && status == 0) {
            btd.rtl_rom_ver = p[4];
            xlog_code("BTD: Realtek ROM version", (int32_t)btd.rtl_rom_ver);
            btd.cmd_complete = 1;
            btd.state = BTD_STATE_RTL_DOWNLOAD;
            btd_rtl_init_firmware();
          } else if (op == 0xFC20 && status == 0) {
            btd.cmd_complete = 1;
            btd_rtl_send_next_chunk();
          } else if (op == HCI_OP_READ_BD_ADDR && status == 0) {
            memcpy(btd.dongle_bdaddr, &p[4], 6);
            btd_log_mac_addr("BTD: dongle BD_ADDR ", btd.dongle_bdaddr);
            btd.cmd_complete = 1;
            btd.state = BTD_STATE_WRITE_SSP;
          } else if (op == HCI_OP_WRITE_SSP_MODE && status == 0) {
            btd.cmd_complete = 1;
            btd.state = BTD_STATE_WRITE_COD;
          } else if (op == HCI_OP_WRITE_CLASS_OF_DEV && status == 0) {
            btd.cmd_complete = 1;
            btd.state = BTD_STATE_WRITE_SCAN_ENABLE;
          } else if (op == HCI_OP_WRITE_SCAN_ENABLE && status == 0) {
            btd.cmd_complete = 1;
            btd.state = BTD_STATE_READY;
            btd.timer = 50; /* 500ms before connecting */
            xlog("BTD: Dongle initialized and scanning (ready)");
          } else if (op == HCI_OP_INQUIRY_CANCEL) {
            btd.cmd_complete = 1;
          }
        }
        break;
      }

      case HCI_EVT_COMMAND_STATUS: {
        break;
      }

      case HCI_EVT_CONN_REQUEST: {
        if (count >= 8) {
          /* Incoming connection request from gamepad */
          uint8_t accept_params[7];
          memcpy(&accept_params[0], &p[0], 6);
          accept_params[6] = 0x00; /* Remain Master */
          btd_queue_cmd(HCI_OP_ACCEPT_CONN_REQ, 7, accept_params);
          xlog("BTD: Incoming connection request accepted");
        }
        break;
      }

      case HCI_EVT_CONN_COMPLETE: {
        uint8_t status = p[0];
        if (status == 0x00) {
          btd.acl_handle = p[1] | ((p[2] & 0x0F) << 8);
          btd.state = BTD_STATE_ACL_CONNECTED;
          xlog("BTD: ACL connected");

          /* Start continuous ACL bulk reading */
          btd.acl_in_busy = 1;
          cellUsbdBulkTransfer(btd.bulk_in_pipe, btd.acl_in_buf, sizeof(btd.acl_in_buf), btd_acl_in_done, &btd);

          /* Initiate L2CAP connection to HID Control channel (PSM 0x0011) */
          btd_send_l2cap_conn_req(L2CAP_PSM_HID_CTRL, L2CAP_LOCAL_CID_HID_CTRL);
          btd.state = BTD_STATE_L2CAP_OPENING_CTRL;
        } else {
          static uint8_t s_direct_try_count = 0;
          if (++s_direct_try_count >= 3) {
            s_direct_try_count = 0;
            btd.state = BTD_STATE_INQUIRY;
            btd.timer = 0;
          } else {
            btd.state = BTD_STATE_READY;
            btd.timer = 150; /* 1.5s listening window for incoming connection */
          }
        }
        break;
      }

      case HCI_EVT_DISCONN_COMPLETE: {
        uint8_t reason = (count >= 6) ? p[3] : 0;
        xlog_code("BTD: Disconnected, reason", (int32_t)reason);
        if (btd.swpro_unit != NULL) {
          btd_destroy_switch_unit(btd.swpro_unit);
          btd.swpro_unit = NULL;
        }
        btd.acl_handle = 0xFFFF;
        btd.ctrl_rcid = 0;
        btd.intr_rcid = 0;
        btd.state = BTD_STATE_READY;
        btd.timer = 100; /* Retry in 1 second */
        break;
      }

      case HCI_EVT_LINK_KEY_REQ: {
        if (count >= 8) {
          btd_queue_cmd(HCI_OP_LINK_KEY_NEG_REPLY, 6, &p[0]);
        }
        break;
      }

      case HCI_EVT_PIN_CODE_REQ: {
        if (count >= 8) {
          btd_queue_cmd(HCI_OP_PIN_CODE_NEG_REPLY, 6, &p[0]);
        }
        break;
      }

      case HCI_EVT_IO_CAP_REQ: {
        if (count >= 8) {
          uint8_t io_params[9];
          memcpy(&io_params[0], &p[0], 6);
          io_params[6] = 0x03; /* NoInputNoOutput */
          io_params[7] = 0x00; /* OOB Auth Data Not Present */
          io_params[8] = 0x00; /* MITM Not Required - General Bonding */
          btd_queue_cmd(HCI_OP_IO_CAP_REPLY, 9, io_params);
        }
        break;
      }

      case HCI_EVT_USER_CONFIRM_REQ: {
        if (count >= 8) {
          btd_queue_cmd(HCI_OP_USER_CONFIRM_REPLY, 6, &p[0]);
        }
        break;
      }

      case 0x13: /* HCI_EVT_NUM_COMPL_PKTS */
      case 0x1B: /* HCI_EVT_MAX_SLOTS_CHANGE */
        break;

      case HCI_EVT_INQUIRY_COMPLETE: {
        xlog("BTD: Inquiry complete, switching back to direct connect");
        btd.state = BTD_STATE_READY;
        btd.timer = 10;
        break;
      }

      case HCI_EVT_INQUIRY_RESULT:
      case HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
      case HCI_EVT_EXT_INQUIRY_RESULT: {
        if (count < 3) {
          break;
        }
        uint8_t num = p[0];
        int32_t min_len = (evt_code == HCI_EVT_INQUIRY_RESULT) ? 15 : 14;
        if (num > 0 && count >= min_len) {
          uint8_t *bd = &p[1];
          uint32_t cod = 0;
          if (evt_code == HCI_EVT_INQUIRY_RESULT) {
            cod = p[10] | (p[11] << 8) | (p[12] << 16);
          } else {
            cod = p[9] | (p[10] << 8) | (p[11] << 16);
          }
          btd_log_mac_addr("BTD: Found device in inquiry scan: ", bd);
          xlog_code("BTD: Device CoD", (int32_t)cod);
          xlog("BTD: Connecting to discovered device...");
          memcpy(btd.target_bdaddr, bd, 6);
          btd_learn_controller_mac(bd);
          btd_queue_cmd(HCI_OP_INQUIRY_CANCEL, 0, NULL);
          btd.state = BTD_STATE_INQUIRY_CANCEL;
        }
        break;
      }

      default:
        if (evt_code != HCI_EVT_COMMAND_STATUS && evt_code != HCI_EVT_COMMAND_COMPLETE) {
          xlog_code("BTD: Unhandled event code", (int32_t)evt_code);
        }
        break;
    }
  }

  /* Re-arm interrupt transfer */
  if (!btd.closing && btd.i_pipe >= 0) {
    btd.evt_busy = 1;
    cellUsbdInterruptTransfer(btd.i_pipe, btd.evt_buf, sizeof(btd.evt_buf), btd_evt_done, &btd);
  }
}

/* L2CAP Helpers and ACL Bulk Transmission */
static void btd_bulk_out_done(int32_t result, int32_t count, void *arg) {
  btd.bulk_out_busy = 0;
  if (result != CELL_OK) {
    xlog_code("BTD bulk out transfer failed", result);
  }
}

static int32_t btd_send_acl_l2cap(uint16_t dcid, const uint8_t *payload, uint16_t payload_len) {
  if (btd.bulk_out_busy || btd.bulk_out_pipe < 0 || btd.acl_handle == 0xFFFF) {
    return -1;
  }
  uint16_t l2cap_total = payload_len;
  uint16_t acl_data_len = l2cap_total + 4;

  /* ACL Header (4 bytes) */
  btd.acl_out_buf[0] = btd.acl_handle & 0xFF;
  btd.acl_out_buf[1] = ((btd.acl_handle >> 8) & 0x0F) | 0x20; /* PB = 0x02 First Auto-Flushable */
  btd.acl_out_buf[2] = acl_data_len & 0xFF;
  btd.acl_out_buf[3] = (acl_data_len >> 8) & 0xFF;

  /* L2CAP Header (4 bytes) */
  btd.acl_out_buf[4] = l2cap_total & 0xFF;
  btd.acl_out_buf[5] = (l2cap_total >> 8) & 0xFF;
  btd.acl_out_buf[6] = dcid & 0xFF;
  btd.acl_out_buf[7] = (dcid >> 8) & 0xFF;

  /* Payload */
  if (payload_len > 0 && payload != NULL) {
    memcpy(&btd.acl_out_buf[8], payload, payload_len);
  }

  btd.bulk_out_busy = 1;
  int32_t r = cellUsbdBulkTransfer(btd.bulk_out_pipe, btd.acl_out_buf, 8 + payload_len, btd_bulk_out_done, &btd);
  if (r != CELL_OK) {
    btd.bulk_out_busy = 0;
    return r;
  }
  return CELL_OK;
}

static void btd_send_l2cap_conn_req(uint16_t psm, uint16_t scid) {
  uint8_t pkt[8];
  pkt[0] = L2CAP_CMD_CONN_REQ;
  pkt[1] = btd.l2cap_ident++;
  pkt[2] = 0x04;
  pkt[3] = 0x00;
  pkt[4] = psm & 0xFF;
  pkt[5] = (psm >> 8) & 0xFF;
  pkt[6] = scid & 0xFF;
  pkt[7] = (scid >> 8) & 0xFF;
  btd_send_acl_l2cap(L2CAP_CID_SIGNALING, pkt, 8);
}

static void btd_send_l2cap_conn_rsp(uint8_t ident, uint16_t dcid, uint16_t scid, uint16_t result) {
  uint8_t pkt[12];
  pkt[0] = L2CAP_CMD_CONN_RSP;
  pkt[1] = ident;
  pkt[2] = 0x08;
  pkt[3] = 0x00;
  pkt[4] = dcid & 0xFF;
  pkt[5] = (dcid >> 8) & 0xFF;
  pkt[6] = scid & 0xFF;
  pkt[7] = (scid >> 8) & 0xFF;
  pkt[8] = result & 0xFF;
  pkt[9] = (result >> 8) & 0xFF;
  pkt[10] = 0x00;
  pkt[11] = 0x00;
  btd_send_acl_l2cap(L2CAP_CID_SIGNALING, pkt, 12);
}

static void btd_send_l2cap_conf_req(uint16_t dcid) {
  uint8_t pkt[8];
  pkt[0] = L2CAP_CMD_CONF_REQ;
  pkt[1] = btd.l2cap_ident++;
  pkt[2] = 0x04;
  pkt[3] = 0x00;
  pkt[4] = dcid & 0xFF;
  pkt[5] = (dcid >> 8) & 0xFF;
  pkt[6] = 0x00;
  pkt[7] = 0x00;
  btd_send_acl_l2cap(L2CAP_CID_SIGNALING, pkt, 8);
}

static void btd_send_l2cap_conf_rsp(uint8_t ident, uint16_t scid) {
  uint8_t pkt[10];
  pkt[0] = L2CAP_CMD_CONF_RSP;
  pkt[1] = ident;
  pkt[2] = 0x06;
  pkt[3] = 0x00;
  pkt[4] = scid & 0xFF;
  pkt[5] = (scid >> 8) & 0xFF;
  pkt[6] = 0x00;
  pkt[7] = 0x00;
  pkt[8] = 0x00; /* Result: Success */
  pkt[9] = 0x00;
  btd_send_acl_l2cap(L2CAP_CID_SIGNALING, pkt, 10);
}

/* ACL Bulk IN Reception (L2CAP Signaling + HID Input Data) */
static void btd_acl_in_done(int32_t result, int32_t count, void *arg) {
  btd.acl_in_busy = 0;
  if (result != CELL_OK) {
    xlog_code("BTD: ACL IN error", result);
  }
  if (btd.closing) {
    return;
  }

  if (count >= 8) {
    uint16_t l2cap_len = btd.acl_in_buf[4] | (btd.acl_in_buf[5] << 8);
    if ((int32_t)count >= 8 + (int32_t)l2cap_len) {
      uint16_t cid = btd.acl_in_buf[6] | (btd.acl_in_buf[7] << 8);
      uint8_t *payload = &btd.acl_in_buf[8];

      if (cid == L2CAP_CID_SIGNALING && l2cap_len >= 4) {
        uint8_t cmd = payload[0];
        uint8_t ident = payload[1];
        uint16_t clen = payload[2] | (payload[3] << 8);
        uint8_t *cdata = &payload[4];

        if (cmd == L2CAP_CMD_CONN_RSP && clen >= 8) {
          uint16_t dcid = cdata[0] | (cdata[1] << 8);
          uint16_t scid = cdata[2] | (cdata[3] << 8);
          uint16_t res  = cdata[4] | (cdata[5] << 8);

          if (res == L2CAP_CONN_SUCCESS) {
            if (scid == L2CAP_LOCAL_CID_HID_CTRL) {
              btd.ctrl_rcid = dcid;
              btd_send_l2cap_conf_req(dcid);
              btd.state = BTD_STATE_L2CAP_CONFIG_CTRL;
            } else if (scid == L2CAP_LOCAL_CID_HID_INTR) {
              btd.intr_rcid = dcid;
              btd_send_l2cap_conf_req(dcid);
              btd.state = BTD_STATE_L2CAP_CONFIG_INTR;
            }
          }
        } else if (cmd == L2CAP_CMD_CONN_REQ && clen >= 4) {
          uint16_t psm  = cdata[0] | (cdata[1] << 8);
          uint16_t scid = cdata[2] | (cdata[3] << 8);

          if (psm == L2CAP_PSM_HID_CTRL) {
            btd.ctrl_rcid = scid;
            btd_send_l2cap_conn_rsp(ident, L2CAP_LOCAL_CID_HID_CTRL, scid, L2CAP_CONN_SUCCESS);
            btd_send_l2cap_conf_req(scid);
          } else if (psm == L2CAP_PSM_HID_INTR) {
            btd.intr_rcid = scid;
            btd_send_l2cap_conn_rsp(ident, L2CAP_LOCAL_CID_HID_INTR, scid, L2CAP_CONN_SUCCESS);
            btd_send_l2cap_conf_req(scid);
          }
        } else if (cmd == L2CAP_CMD_CONF_REQ && clen >= 4) {
          uint16_t dcid = cdata[0] | (cdata[1] << 8);
          btd_send_l2cap_conf_rsp(ident, dcid == L2CAP_LOCAL_CID_HID_CTRL ? btd.ctrl_rcid : btd.intr_rcid);
          if (dcid == L2CAP_LOCAL_CID_HID_CTRL) btd.remote_conf_ctrl = 1;
          if (dcid == L2CAP_LOCAL_CID_HID_INTR) btd.remote_conf_intr = 1;
        } else if (cmd == L2CAP_CMD_CONF_RSP && clen >= 6) {
          uint16_t scid = cdata[0] | (cdata[1] << 8);
          uint16_t res  = cdata[4] | (cdata[5] << 8);

          if (res == L2CAP_CONF_SUCCESS) {
            if (scid == L2CAP_LOCAL_CID_HID_CTRL) {
              btd.local_conf_ctrl = 1;
              /* Control channel configured, now open Interrupt channel */
              btd_send_l2cap_conn_req(L2CAP_PSM_HID_INTR, L2CAP_LOCAL_CID_HID_INTR);
              btd.state = BTD_STATE_L2CAP_OPENING_INTR;
            } else if (scid == L2CAP_LOCAL_CID_HID_INTR) {
              btd.local_conf_intr = 1;
              /* Both channels open! Switch Pro Bluetooth is ACTIVE */
              btd.state = BTD_STATE_HID_ACTIVE;
              xlog("BTD: HID interrupt channel opened and active!");

              if (btd.swpro_unit == NULL) {
                btd.swpro_unit = btd_create_switch_unit();
                if (btd.swpro_unit != NULL) {
                  btd_start_swpro_init(btd.swpro_unit);
                }
              }
            }
          }
        }
      } else if ((cid == L2CAP_LOCAL_CID_HID_INTR || cid == btd.intr_rcid) && l2cap_len >= 2) {
        /* HID Report received over Bluetooth */
        const uint8_t *rep = NULL;
        int32_t rlen = 0;
        if (payload[0] == BT_HID_HDR_DATA_INPUT && l2cap_len > 1) {
          rep = &payload[1];
          rlen = l2cap_len - 1;
        } else if (payload[0] == 0x30 || payload[0] == 0x21 || payload[0] == 0x3F) {
          rep = &payload[0];
          rlen = l2cap_len;
        }

        if (rep != NULL) {
          static uint32_t s_in_count = 0;
          if (s_in_count++ == 0 || (s_in_count % 300) == 0) {
            xlog_code("BTD: HID input report ID", (int32_t)rep[0]);
            xlog_code("BTD: HID input report len", rlen);
          }
          if (btd.swpro_unit != NULL) {
            btd_push_report(btd.swpro_unit, rep, rlen);
          }
        }
      } else {
        static uint32_t s_other_cid_count = 0;
        if (s_other_cid_count++ < 3) {
          xlog_code("BTD: ACL IN other CID", (int32_t)cid);
          xlog_code("BTD: ACL IN other len", (int32_t)l2cap_len);
        }
      }
    }
  }

  /* Re-arm bulk IN transfer */
  if (!btd.closing && btd.bulk_in_pipe >= 0) {
    btd.acl_in_busy = 1;
    cellUsbdBulkTransfer(btd.bulk_in_pipe, btd.acl_in_buf, sizeof(btd.acl_in_buf), btd_acl_in_done, &btd);
  }
}

/* Public API for sending output reports (rumble and subcommands) over Bluetooth */
int32_t btd_send_hid_report(const uint8_t *report, int32_t len) {
  if (btd.state != BTD_STATE_HID_ACTIVE || btd.intr_rcid == 0 || len <= 0) {
    return -1;
  }
  /* Bluetooth HID Output reports to Nintendo Switch controllers must be 
   * exactly 50 bytes (0xA2 header + 49 bytes payload). Packets shorter 
   * than 50 bytes are rejected by controller firmware as TOO_SHORT. */
  uint8_t out_pkt[50];
  memset(out_pkt, 0, sizeof(out_pkt));
  out_pkt[0] = BT_HID_HDR_DATA_OUTPUT; /* 0xA2 */
  int32_t copy_len = (len < 49) ? len : 49;
  memcpy(&out_pkt[1], report, copy_len);
  return btd_send_acl_l2cap(btd.intr_rcid, out_pkt, 50);
}

int32_t btd_send_hid_output(void *unit, const uint8_t *data, int32_t len) {
  (void)unit;
  return btd_send_hid_report(data, len);
}

/* Polling tick driven from main.c xpadd_thread (every 10ms) */
void btd_poll_tick(void) {
  btd_process_cmd_queue();

  if (btd.cmd_busy) {
    return;
  }

  switch (btd.state) {
    case BTD_STATE_ATTACHED: {
      btd_send_cmd_direct(HCI_OP_RESET, 0, NULL);
      btd.state = BTD_STATE_RESET;
      break;
    }

    case BTD_STATE_RESET: {
      if (btd.cmd_complete) {
        btd_send_cmd_direct(HCI_OP_RESET, 0, NULL);
      }
      break;
    }

    case BTD_STATE_RTL_READ_ROM: {
      if (btd.cmd_complete) {
        btd_send_cmd_direct(0xFC6D, 0, NULL);
      }
      break;
    }

    case BTD_STATE_RTL_DOWNLOAD: {
      break;
    }

    case BTD_STATE_READ_BD_ADDR: {
      if (btd.cmd_complete) {
        btd_send_cmd_direct(HCI_OP_READ_BD_ADDR, 0, NULL);
      }
      break;
    }

    case BTD_STATE_WRITE_SSP: {
      if (btd.cmd_complete) {
        uint8_t ssp_mode = 0x01; /* Enable Secure Simple Pairing */
        btd_send_cmd_direct(HCI_OP_WRITE_SSP_MODE, 1, &ssp_mode);
      }
      break;
    }

    case BTD_STATE_WRITE_COD: {
      if (btd.cmd_complete) {
        uint8_t cod[3] = {0x04, 0x02, 0x00}; /* Desktop Computer (Host) */
        btd_send_cmd_direct(HCI_OP_WRITE_CLASS_OF_DEV, 3, cod);
      }
      break;
    }

    case BTD_STATE_WRITE_SCAN_ENABLE: {
      if (btd.cmd_complete) {
        uint8_t scan = HCI_SCAN_INQUIRY_AND_PAGE; /* 0x03 */
        btd_send_cmd_direct(HCI_OP_WRITE_SCAN_ENABLE, 1, &scan);
      }
      break;
    }

    case BTD_STATE_READY: {
      if (btd.timer > 0) {
        btd.timer--;
      } else {
        /* Direct Connection to stored controller BD_ADDR (wire little-endian format) */
        uint8_t conn_params[13];
        memcpy(&conn_params[0], btd.target_bdaddr, 6);

        conn_params[6] = 0x18; /* Packet Type 0xCC18 */
        conn_params[7] = 0xCC;
        conn_params[8] = 0x02; /* Page Scan Repetition Mode R2 (widest window) */
        conn_params[9] = 0x00; /* Reserved */
        conn_params[10] = 0x00; /* Clock Offset */
        conn_params[11] = 0x00;
        conn_params[12] = 0x00; /* Remain Master */

        btd_log_mac_addr("BTD: Direct connection to BD_ADDR: ", &conn_params[0]);
        btd_send_cmd_direct(HCI_OP_CREATE_CONN, 13, conn_params);
        btd.state = BTD_STATE_CONNECTING;
        btd.conn_timer = 600; /* 6.0 seconds timeout */
      }
      break;
    }

    case BTD_STATE_CONNECTING: {
      if (btd.conn_timer > 0) {
        btd.conn_timer--;
      } else {
        xlog("BTD: Direct connection timeout, switching to Inquiry...");
        btd_queue_cmd(HCI_OP_CREATE_CONN_CANCEL, 6, btd.target_bdaddr);
        btd.state = BTD_STATE_INQUIRY;
        btd.timer = 0;
      }
      break;
    }

    case BTD_STATE_INQUIRY: {
      if (btd.timer > 0) {
        btd.timer--;
        if (btd.timer == 0) {
          xlog("BTD: Inquiry scan timeout, switching to direct connect");
          btd_queue_cmd(HCI_OP_INQUIRY_CANCEL, 0, NULL);
          btd.state = BTD_STATE_READY;
          btd.timer = 10;
        }
      } else {
        uint8_t inq_params[5] = {0x33, 0x8B, 0x9E, 0x03, 0x00}; /* GIAC, 3.84s window */
        xlog("BTD: >>> STARTING INQUIRY SCAN FOR PAIRING CONTROLLERS <<<");
        btd_send_cmd_direct(HCI_OP_INQUIRY, 5, inq_params);
        btd.timer = 384;
      }
      break;
    }

    case BTD_STATE_INQUIRY_CANCEL: {
      if (btd.cmd_complete) {
        btd.state = BTD_STATE_READY;
        btd.timer = 10;
      }
      break;
    }

    default:
      break;
  }
}
