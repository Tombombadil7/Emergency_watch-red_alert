#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GC9A01A_TFTWIDTH 240
#define GC9A01A_TFTHEIGHT 240
// Command definitions
#define GC9A01A_SWRESET 0x01
#define GC9A01A_SLPIN 0x10
#define GC9A01A_SLPOUT 0x11
#define GC9A01A_PTLON 0x12
#define GC9A01A_NORON 0x13
#define GC9A01A_INVOFF 0x20
#define GC9A01A_INVON 0x21
#define GC9A01A_DISPOFF 0x28
#define GC9A01A_DISPON 0x29
#define GC9A01A_CASET 0x2A
#define GC9A01A_RASET 0x2B
#define GC9A01A_RAMWR 0x2C
#define GC9A01A_MADCTL 0x36
#define GC9A01A_COLMOD 0x3A
#define GC9A01A_FRAMERATE 0xE8
#define GC9A01A_INREGEN2 0xEF
#define GC9A01A_GAMMA1 0xF0
#define GC9A01A_GAMMA2 0xF1
#define GC9A01A_GAMMA3 0xF2
#define GC9A01A_GAMMA4 0xF3
#define GC9A01A_INREGEN1 0xFE

typedef enum {
  MODE_COMMAND,
  MODE_DATA,
} chip_mode_t;

typedef struct {
  pin_t cs_pin;
  pin_t dc_pin;
  pin_t rst_pin;
  spi_dev_t spi;
  uint8_t spi_buffer[1024];

  buffer_t framebuffer;
  uint32_t width;
  uint32_t height;

  chip_mode_t mode;
  uint8_t command_code;
  uint8_t command_size;
  uint8_t command_index;
  uint8_t command_buf[16];
  bool ram_write;

  uint16_t x_start;
  uint16_t x_end;
  uint16_t y_start;
  uint16_t y_end;
  uint16_t x_cursor;
  uint16_t y_cursor;
  uint8_t madctl;

  bool display_on;
  bool sleep_mode;
  bool invert_display;
  uint8_t color_mode;
  uint8_t color_byte_count;  // New member to track color byte count
} chip_state_t;

static void chip_pin_change(void *user_data, pin_t pin, uint32_t value);
static void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count);

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));

  chip->width = GC9A01A_TFTWIDTH;
  chip->height = GC9A01A_TFTHEIGHT;
  chip->x_start = 0;
  chip->x_end = chip->width - 1;
  chip->y_start = 0;
  chip->y_end = chip->height - 1;
  chip->x_cursor = 0;
  chip->y_cursor = 0;
  chip->madctl = 0;
  chip->display_on = false;
  chip->sleep_mode = true;
  chip->invert_display = false;
  chip->color_mode = 0x06; // 18-bit color
  chip->color_byte_count = 0;

  const pin_watch_config_t watch_config = {
    .edge = BOTH,
    .pin_change = chip_pin_change,
    .user_data = chip,
  };

  chip->cs_pin = pin_init("CS", INPUT_PULLUP);
  pin_watch(chip->cs_pin, &watch_config);

  chip->dc_pin = pin_init("DC", INPUT);
  pin_watch(chip->dc_pin, &watch_config);

  chip->rst_pin = pin_init("RST", INPUT_PULLUP);
  pin_watch(chip->rst_pin, &watch_config);

  const spi_config_t spi_config = {
    .sck = pin_init("SCL", INPUT),
    .mosi = pin_init("SDA", INPUT),
    .miso = NO_PIN,
    .done = chip_spi_done,
    .user_data = chip,
  };
  chip->spi = spi_init(&spi_config);

  chip->framebuffer = framebuffer_init(&chip->width, &chip->height);
  printf("GC9A01A Driver Chip initialized!\n");
}

void chip_pin_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = (chip_state_t*)user_data;
  if (pin == chip->cs_pin) {
    if (value == LOW) {
      chip->command_size = 0;
      chip->command_index = 0;
      spi_start(chip->spi, chip->spi_buffer, sizeof(chip->spi_buffer));
    } else {
      spi_stop(chip->spi);
    }
  }
  if (pin == chip->dc_pin && chip->mode != value) {
    spi_stop(chip->spi);
    chip->mode = value;
    if (pin_read(chip->cs_pin) == LOW) {
      spi_start(chip->spi, chip->spi_buffer, sizeof(chip->spi_buffer));
    }
  }
  if (pin == chip->rst_pin && value == LOW) {
    spi_stop(chip->spi);
    chip->x_cursor = 0;
    chip->y_cursor = 0;
    chip->ram_write = false;
    chip->display_on = false;
    chip->sleep_mode = true;
    chip->invert_display = false;
    chip->madctl = 0;
    chip->color_mode = 0x06;
    chip->color_byte_count = 0;
  }
}

int command_args_size(uint8_t command_code) {
  switch (command_code) {
    case GC9A01A_MADCTL:
    case GC9A01A_COLMOD:
      return 1;
    case GC9A01A_CASET:
    case GC9A01A_RASET:
      return 4;
    case GC9A01A_FRAMERATE:
      return 1;
    case GC9A01A_GAMMA1:
    case GC9A01A_GAMMA2:
    case GC9A01A_GAMMA3:
    case GC9A01A_GAMMA4:
      return 6;
    default:
      return 0;
  }
}

void execute_command(chip_state_t *chip) {
  switch (chip->command_code) {
    case GC9A01A_SWRESET:
      chip->x_cursor = 0;
      chip->y_cursor = 0;
      chip->ram_write = false;
      chip->display_on = false;
      chip->sleep_mode = true;
      chip->invert_display = false;
      chip->madctl = 0;
      chip->color_mode = 0x06;
      chip->color_byte_count = 0;
      break;
    case GC9A01A_SLPIN:
      chip->sleep_mode = true;
      break;
    case GC9A01A_SLPOUT:
      chip->sleep_mode = false;
      break;
    case GC9A01A_INVOFF:
      chip->invert_display = false;
      break;
    case GC9A01A_INVON:
      chip->invert_display = true;
      break;
    case GC9A01A_DISPOFF:
      chip->display_on = false;
      break;
    case GC9A01A_DISPON:
      chip->display_on = true;
      break;
    case GC9A01A_CASET:
      chip->x_start = (chip->command_buf[0] << 8) | chip->command_buf[1];
      chip->x_end = (chip->command_buf[2] << 8) | chip->command_buf[3];
      chip->x_cursor = chip->x_start;
      break;
    case GC9A01A_RASET:
      chip->y_start = (chip->command_buf[0] << 8) | chip->command_buf[1];
      chip->y_end = (chip->command_buf[2] << 8) | chip->command_buf[3];
      chip->y_cursor = chip->y_start;
      break;
    case GC9A01A_RAMWR:
      chip->ram_write = true;
      break;
    case GC9A01A_MADCTL:
      chip->madctl = chip->command_buf[0];
      break;
    case GC9A01A_COLMOD:
      chip->color_mode = chip->command_buf[0];
      // (chip->color_mode == 0x06) {  // 18-bit color
      chip->color_byte_count = 0;
      //}
      break;
    default:
      //printf("Warning: unknown command 0x%02x\n", chip->command_code);
      break;
  }
}

void process_command(chip_state_t *chip, uint8_t *buffer, uint32_t buffer_size) {
  chip->ram_write = false;
  for (int i = 0; i < buffer_size; i++) {
    chip->command_code = buffer[i];
    chip->command_size = command_args_size(chip->command_code);
    chip->command_index = 0;
    if (!chip->command_size) {
      execute_command(chip);
    }
  }
}

void process_command_args(chip_state_t *chip, uint8_t *buffer, uint32_t buffer_size) {
  for (int i = 0; i < buffer_size; i++) {
    if (chip->command_index < chip->command_size) {
      chip->command_buf[chip->command_index++] = buffer[i];
      if (chip->command_size == chip->command_index) {
        execute_command(chip);
      }
    }
  }
}

uint32_t rgb565_to_rgba(uint16_t value) {
  uint8_t b5 = (value >> 11) & 0x1F;
  uint8_t g6 = (value >> 5)  & 0x3F;
  uint8_t r5 = value & 0x1F;
  uint8_t r8 = (r5 << 3) | (r5 >> 2);
  uint8_t g8 = (g6 << 2) | (g6 >> 4);
  uint8_t b8 = (b5 << 3) | (b5 >> 2);
  return (0xFFu << 24) | (r8 << 16) | (g8 << 8) | b8;
}

uint32_t rgb666_to_rgba(uint8_t r6, uint8_t g6, uint8_t b6) {
  uint8_t r8 = (r6 << 2) | (r6 >> 4);
  uint8_t g8 = (g6 << 2) | (g6 >> 4);
  uint8_t b8 = (b6 << 2) | (b6 >> 4);
  return (0xFFu << 24) | (r8 << 16) | (g8 << 8) | b8;
}

void process_data(chip_state_t *chip, const uint8_t *buffer, uint32_t buffer_size) {
  if (buffer_size % 2 != 0) return;
  for (uint32_t i = 0; i < buffer_size; i++) {
    if (!(chip->ram_write && chip->display_on && !chip->sleep_mode)) {
      continue;
    }
    // KONWERSJA KOLORU 
    uint32_t color = 0;
    uint16_t color16 = ((uint16_t)buffer[i] << 8) | buffer[i + 1];
    i++; // dwa bajty na pixel
    // ROZPAKOWANIE RGB565 (uwzględniając TFT_BGR)
    // czyli zamieniamy R <-> B
    uint8_t b5 = (color16 >> 11) & 0x1F;  // dawniej R
    uint8_t g6 = (color16 >> 5)  & 0x3F;
    uint8_t r5 = color16 & 0x1F;          // dawniej B
    // Konwersja 5/6-bit na 8-bit
    uint8_t r8 = (r5 << 3) | (r5 >> 2);
    uint8_t g8 = (g6 << 2) | (g6 >> 4);
    uint8_t b8 = (b5 << 3) | (b5 >> 2);
    // Składamy w ARGB8888 (pełne 32 bity)
    color = (0xFFu << 24) | (r8 << 16) | (g8 << 8) | b8;
    // --- DEBUG opcjonalny ---
    if (chip->x_cursor == 0 && chip->y_cursor < 5) {
      printf("[%03d] color=0x%08X (r=%d g=%d b=%d)\n", chip->y_cursor, color, r8, g8, b8);
    }
    // --- ZAPIS DO RAMKI ---
    int x = chip->x_cursor;
    int y = chip->y_cursor;
    bool swap_xy = chip->madctl & 0x20;   // MV
    bool mirror_x = chip->madctl & 0x40;  // MX
    bool mirror_y = chip->madctl & 0x80;  // MY

    if (swap_xy) {
      int tmp = x; x = y; y = tmp;
    }
    if (mirror_x) x = chip->width - 1 - x;
    if (mirror_y) y = chip->height - 1 - y;
    if (x < chip->width && y < chip->height) {
      uint32_t pix_index = (uint32_t)y * chip->width + x;
      buffer_write(chip->framebuffer, pix_index * 4, &color, 4);
    }
    chip->x_cursor++;
    if (chip->x_cursor > chip->x_end) {
      chip->x_cursor = chip->x_start;
      chip->y_cursor++;
      if (chip->y_cursor > chip->y_end) {
        chip->y_cursor = chip->y_start;
      }
    }
  }
}

void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  chip_state_t *chip = (chip_state_t*)user_data;
  if (!count) {
    return;
  }
  if (chip->mode == MODE_DATA) {
    if (chip->ram_write) {
      process_data(chip, buffer, count);
    } else {
      process_command_args(chip, buffer, count);
    }
  } else {
    process_command(chip, buffer, count);
  }

  if (pin_read(chip->cs_pin) == LOW) {
    spi_start(chip->spi, chip->spi_buffer, sizeof(chip->spi_buffer));
  }
}
// Main chip function (if needed)
void chip_main() {
  // This function can be used for any continuous operations or checks
  // that need to be performed by the chip. In this case, we don't need
  // any continuous operations, so we'll leave it empty.
}