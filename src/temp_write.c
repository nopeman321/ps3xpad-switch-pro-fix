static int32_t write_xpad(XPAD_UNIT_t *unit, uint8_t *data, int32_t len) {
  int32_t r;

  if (len > XPAD_OUT_LEN) {
    return(-1);
  }
