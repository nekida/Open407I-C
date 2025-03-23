#ifndef AT45DBxx_H
#define AT45DBxx_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "spi.h"
#include "gpio.h"

#define AT45DBxx_PAGE_SIZE (512)
#define AT45DBxx_BLOCK_SIZE (AT45DBxx_PAGE_SIZE * 8)

#define AT45DBXX_OPCODE_GET_ID          (0x9F)
#define AT45DBXX_OPCODE_WR_PAGE_ON_BUF1 (0x82)
#define AT45DBXX_OPCODE_WR_PAGE_ON_BUF2 (0x85)
#define AT45DBXX_OPCODE_CONT_ARR_RD_HF  (0x0B)

typedef struct at45dbxx_config_s {
    uint8_t opcode;
    uint32_t addr;
    uint8_t *msg;
    size_t msg_len;
} at45dbxx_config_t;

void at45dbxx_init (const SPI_HandleTypeDef *spi, GPIO_TypeDef *port, uint32_t pin);
bool at45dbxx_rd_id (uint8_t *id, size_t id_size);
bool at45dbxx_wr_data (const at45dbxx_config_t * const config);
bool at45dbxx_rd_data (at45dbxx_config_t * const config);

#endif // AT45DBxx_H
