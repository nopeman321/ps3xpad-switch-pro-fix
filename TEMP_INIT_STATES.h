#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Define missing states
#define SWPRO_INIT_IDLE          0
#define SWPRO_INIT_HANDSHAKE_1   1
#define SWPRO_INIT_SET_BAUDRATE  2
#define SWPRO_INIT_HANDSHAKE_2   3
#define SWPRO_INIT_FORCE_HID     4
#define SWPRO_INIT_ENABLE_IMU    5
#define SWPRO_INIT_ENABLE_VIB    6
#define SWPRO_INIT_DONE          7

// ... el resto de la estructura se mantiene, pero actualizamos swpro_init en el código fuente
// Nota: Debido a problemas de permisos en la consola, ejecutaré el cambio vía bash usando sed
