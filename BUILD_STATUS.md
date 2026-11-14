# Build Status — ps3xpad con soporte Switch Pro

## Artefactos (build limpio, Nov 13 2026)

| Archivo | Tamaño | Estado |
|---------|--------|--------|
| `src/xpad.sprx`      | 19 773 bytes | ✅ compilado y firmado (Key Rev 0x000A, SELF, Data encrypted) |
| `game/xpad_game.sprx` | 2 873 bytes  | ✅ compilado y firmado |

## Qué se implementó

### Error 1 — Documento de protocolo (`SWITCH_PRO_PROTOCOL.md`)

Corregidas tres inconsistencias:

1. **IMU frames 16 → 12 bytes.** La versión anterior decía cada frame IMU era de
   16 bytes, pero `13 + 3×16 = 61 > 54`. Los frames reales (accel xyz + gyro xyz,
   LE int16) son 12 bytes. Offsets corregidos: IMU en 13, 25, 37; sticks en 49.

2. **Reporte 0x21 — subcmd en byte 11, no 14.** La tabla del subcmd reply tenía
   el campo de acknowledgement en el byte 14 (dentro de la porción de input),
   cuando en realidad está en el byte 11 (tras los 9 bytes del payload SPI).

3. **Tabla de botones añadida.** El documento no documentaba el layout de los 3
   bytes de botones. Se añadió la sección §5b con la tabla completa de bits para
   cada cluster (right/shared/left) y el mapeo PS3.

### Error 2 — Implementación Switch Pro en `src/main.c`

El enum `XTYPE_SWITCH_PRO = 3` existía pero era letra muerta. Ahora está completo:

#### Estructuras y constantes (`main.c` ~línea 158)
- `SWPRO_FULL_REPORT_t` — struct packed de 54 bytes para el reporte 0x30
- Constantes de protocolo: report IDs, subcommands, offsets, button bits
- `swpro_rumble_neutral[8]` — neutral `00 01 40 40 00 01 40 40`

#### Tabla de dispositivos
```c
static XPAD_INFO_t switch_info[] = {
  {0x057e, 0x2009, "Nintendo Switch Pro Controller"},
};
```
Registrada con su propio `swpro_ops` LDD. Los pads de terceros (8BitDo en Switch
mode) se añaden por `xpad_devices.txt` con `XTYPE_SWITCH_PRO`.

#### Métodos implementados
| Función | Qué hace |
|---------|----------|
| `swpro_probe()` | acepta VID/PID de `switch_info[]` y extras con `XTYPE_SWITCH_PRO` |
| `swpro_attach()` | busca interfaz HID (clase 0x03), abre pipes IN/OUT, registra LDD, llama `swpro_init()` |
| `swpro_detach()` | cierra unidad, desregistra LDD |
| `swpro_detach_all()` | desconecta todas las unidades Switch en unload |
| `swpro_send_subcommand()` | report 0x01 de 16 bytes con contador 0x00–0x0F wrapping |
| `swpro_init()` | secuencia: SET_MODE(0x30) → ENABLE_IMU(0x01) → ENABLE_VIB(0x01) |
| `swpro_read_report()` | parse de 0x30/0x21, mapeo diagonal A/B/X/Y→Cross/Circle/Square/Triangle |
| `swpro_read_input()` | drena ring buffer, llama `swpro_read_report()` |
| `swpro_set_led()` | convierte LED byte 0x06–0x09 a patrón one-hot 4-LEDs |
| `swpro_set_rumble()` | report 0x10 de 17 bytes, motores izq/der independientes |

#### Mapeo de botones
| Nintendo | PS3 |
|----------|-----|
| A | Cross ✕ |
| B | Circle ○ |
| X | Square □ |
| Y | Triangle △ |
| L / ZL | L1 / L2 |
| R / ZR | R1 / R2 |
| Minus / Plus | Select / Start |
| Stick L/R click | L3 / R3 |
| D-pad | D-pad |
| Home | *(sin mapeo — sin equivalente PS3)* |

**La diagonal importa:** A/B/X/Y en Nintendo están rotados 90° respecto a Xbox.
Mapear recto da los botones en posición física correcta; mapear Xbox-directo los
rota 180°.

#### Bugfixes críticos

1. **`MAX_XPAD_PAYLOAD` 32 → 64.** El reporte 0x30 tiene 54 bytes. Con el límite
   anterior, `data_transfer_done()` truncaba los reportes a 32 bytes, `swpro_read_report`
   fallaba el length check y la PS3 jamás recibía input.

2. **`XPAD_OUT_LEN` 12 → 17.** El report 0x10 de rumble es 1 + 16 = 17 bytes.
   Con 12, `write_xpad()` rechazaba el comando y el pad nunca vibraba.

3. **`xpad_devices.txt` acepta `XTYPE_SWITCH_PRO`.** El parser ahora reconoce
   `XTYPE_SWITCH_PRO`, `XTYPE_SWITCH` y `PTYPE_SWITCH` para facilitar la
   adición de pads de terceros sin recompilar.

## Tests unitarios

```
docker run --rm -v "E:/DSH/8btdoplging/ps3xpad_latest_switch/workspace_xpad:/work" \
  -w /work/tests --entrypoint sh ghcr.io/altps3/ps3dev:latest -c 'make test'
```

**232 passed, 0 failed.** 18 grupos de tests:

| # | Grupo |
|---|-------|
| 1–8 | Protocolo base: tamaños, offsets, IMU, botones, subcommands, rumble, SPI, handshake |
| 9 | Endpoints y report IDs |
| 10 | Bit constants de botones (derivados de `main.c`) |
| 11 | Mapeo diagonal cara A→Cross B→Circle X→Square Y→Triangle |
| 12 | D-pad, Plus/Minus→Start/Select, stick clicks, L/R/ZL/ZR |
| 13 | Passthrough de sticks analógicos sin escala ni inversión |
| 14 | Filtrado por tipo de reporte (0x30 y 0x21 pasan, 0x3F se rechaza) |
| 15 | Patrón LED one-hot, 4 puertos distintos, sum = 0xF0 |
| 16 | Construcción byte a byte del report 0x10 de rumble |
| 17 | Verificación de tamaños de buffer vs reporte real (el bug 32→64) |
| 18 | Report de subcommand siempre 16 bytes, counter wraps 0x0F→0x00 |

## Warnings pendientes (pre-existentes, no fatales)

| Archivo | Línea | Warning |
|---------|-------|---------|
| `main.c` | 56–57 | `CELL_USBD_ERROR_*` redefinidas (el SDK ya las define) |
| `main.c` | 596, 602 | integer-from-pointer en wrappers de syscall |
| `main.c` | 3348 | tipo de puntero en `sys_ppu_thread_create` |
| `libc.c` | 63 | `strstr` descarta `const` |
| `xpad_game.c` | 72 | `sys_event_port_connect_ipc` declaración implícita |

Ninguno afecta al comportamiento en PS3 (son todos pre-existentes desde antes
de este trabajo).
