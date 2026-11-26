#ifndef __BT_DONGLE_H__
#define __BT_DONGLE_H__

#include <stdint.h>
#include <sys/types.h>
#include <cell/usbd.h>

/* USB Bluetooth HCI Class / SubClass / Protocol */
#define USB_CLASS_WIRELESS_CONTROLLER        0xE0
#define USB_SUBCLASS_RF_CONTROLLER           0x01
#define USB_PROTOCOL_BLUETOOTH_PROGRAMMING   0x01

/* USB HCI Transport Request Parameters (Endpoint 0 Control Pipe) */
#define HCI_USB_REQTYPE_CMD                  (USB_REQTYPE_DIR_TO_DEVICE | USB_REQTYPE_TYPE_CLASS | USB_REQTYPE_RECIP_DEVICE) /* 0x20 */
#define HCI_USB_BREQUEST_CMD                 0x00

/* Endpoints (Standard USB Bluetooth Dongle layout) */
#define HCI_EP_EVENT_IN                      0x81 /* Interrupt IN  (Events) */
#define HCI_EP_ACL_IN                        0x82 /* Bulk IN       (ACL Data) */
#define HCI_EP_ACL_OUT                       0x02 /* Bulk OUT      (ACL Data) */

/* Packet Maximum Buffer Sizes */
#define HCI_CMD_BUF_SIZE                     300
#define HCI_EVT_BUF_SIZE                     64
#define HCI_ACL_BUF_SIZE                     128

/* HCI Opcodes (OGF << 10 | OCF) */
/* OGF 0x01: Link Control Commands */
#define HCI_OP_INQUIRY                       0x0401
#define HCI_OP_INQUIRY_CANCEL                0x0402
#define HCI_OP_CREATE_CONN                   0x0405
#define HCI_OP_DISCONNECT                    0x0406
#define HCI_OP_CREATE_CONN_CANCEL             0x0408
#define HCI_OP_ACCEPT_CONN_REQ               0x0409
#define HCI_OP_LINK_KEY_NEG_REPLY            0x040C
#define HCI_OP_PIN_CODE_NEG_REPLY            0x040E
#define HCI_OP_AUTH_REQUESTED                0x0411
#define HCI_OP_IO_CAP_REPLY                  0x042B
#define HCI_OP_USER_CONFIRM_REPLY            0x042C

/* OGF 0x03: Controller & Baseband Commands */
#define HCI_OP_RESET                         0x0C03
#define HCI_OP_WRITE_SCAN_ENABLE             0x0C1A
#define HCI_OP_WRITE_CLASS_OF_DEV            0x0C24
#define HCI_OP_WRITE_SSP_MODE                0x0C56

/* OGF 0x04: Informational Parameters */
#define HCI_OP_READ_BD_ADDR                  0x1009

/* HCI Event Codes */
#define HCI_EVT_INQUIRY_COMPLETE             0x01
#define HCI_EVT_INQUIRY_RESULT               0x02
#define HCI_EVT_CONN_COMPLETE                0x03
#define HCI_EVT_CONN_REQUEST                 0x04
#define HCI_EVT_DISCONN_COMPLETE             0x05
#define HCI_EVT_AUTH_COMPLETE                0x06
#define HCI_EVT_COMMAND_COMPLETE             0x0E
#define HCI_EVT_COMMAND_STATUS               0x0F
#define HCI_EVT_ROLE_CHANGE                  0x12
#define HCI_EVT_PIN_CODE_REQ                 0x16
#define HCI_EVT_LINK_KEY_REQ                 0x17
#define HCI_EVT_LINK_KEY_NOTIF               0x18
#define HCI_EVT_IO_CAP_REQ                   0x31
#define HCI_EVT_USER_CONFIRM_REQ             0x33
#define HCI_EVT_SIMPLE_PAIRING_COMPLETE      0x36
#define HCI_EVT_INQUIRY_RESULT_WITH_RSSI     0x22
#define HCI_EVT_EXT_INQUIRY_RESULT           0x2F

/* HCI Scan Enable Flags */
#define HCI_SCAN_DISABLED                    0x00
#define HCI_SCAN_INQUIRY_ONLY                0x01
#define HCI_SCAN_PAGE_ONLY                   0x02
#define HCI_SCAN_INQUIRY_AND_PAGE            0x03

/* L2CAP Channels (CIDs) */
#define L2CAP_CID_SIGNALING                  0x0001
#define L2CAP_CID_CONNECTIONLESS             0x0002

/* L2CAP Protocol/Service Multiplexers (PSM) */
#define L2CAP_PSM_HID_CTRL                   0x0011
#define L2CAP_PSM_HID_INTR                   0x0013

/* Local CIDs assigned to Switch Pro HID channels */
#define L2CAP_LOCAL_CID_HID_CTRL             0x0040
#define L2CAP_LOCAL_CID_HID_INTR             0x0041

/* L2CAP Signaling Command Codes */
#define L2CAP_CMD_COMMAND_REJ                0x01
#define L2CAP_CMD_CONN_REQ                   0x02
#define L2CAP_CMD_CONN_RSP                   0x03
#define L2CAP_CMD_CONF_REQ                   0x04
#define L2CAP_CMD_CONF_RSP                   0x05
#define L2CAP_CMD_DISCONN_REQ                0x06
#define L2CAP_CMD_DISCONN_RSP                0x07

/* L2CAP Connection Response Result Codes */
#define L2CAP_CONN_SUCCESS                   0x0000
#define L2CAP_CONN_PENDING                   0x0001
#define L2CAP_CONN_REFUSED_PSM               0x0002
#define L2CAP_CONN_REFUSED_SECURITY          0x0003
#define L2CAP_CONN_REFUSED_NO_RESOURCES      0x0004

/* L2CAP Configuration Response Result Codes */
#define L2CAP_CONF_SUCCESS                   0x0000
#define L2CAP_CONF_UNACCEPTABLE_PARAMS       0x0001
#define L2CAP_CONF_REJECTED                  0x0002

/* Bluetooth HID Transaction Headers (PSM 0x0013 / 0x0011) */
#define BT_HID_HDR_DATA_INPUT                0xA1 /* 0xA0 (DATA) | 0x01 (INPUT) */
#define BT_HID_HDR_DATA_OUTPUT               0xA2 /* 0xA0 (DATA) | 0x02 (OUTPUT) */

#define SWPRO_OUT_REPORT_LEN                 16

/* Public API exported by bt_dongle to src/main.c */
int32_t btd_probe(int32_t dev_id);
int32_t btd_attach(int32_t dev_id);
int32_t btd_detach(int32_t dev_id);
int32_t btd_detach_all(void);
void    btd_poll_tick(void);
void    btd_learn_controller_mac(const uint8_t mac_le[6]);
int32_t btd_send_hid_report(const uint8_t *report, int32_t len);
int32_t btd_send_hid_output(void *unit, const uint8_t *data, int32_t len);
int32_t btd_is_dongle_present(void);
int32_t btd_is_connected(void);
const char *btd_get_status_str(void);
void    btd_set_paused(uint8_t paused);
void    btd_disconnect(void);
int32_t btd_rescan_dongle(void);

#endif /* __BT_DONGLE_H__ */
