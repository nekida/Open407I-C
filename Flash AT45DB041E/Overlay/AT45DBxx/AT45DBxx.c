#include "AT45DBxx.h"

#include "string.h"

static SPI_HandleTypeDef at45dbxx_spi;
static GPIO_TypeDef *at45dbxx_spi_cs_port;
static uint32_t at45dbxx_spi_cs_pin;
static bool is_init = false;
static const size_t addr_size = 3;

void at45dbxx_init (const SPI_HandleTypeDef *spi, GPIO_TypeDef *port, uint32_t pin)
{
    at45dbxx_spi = *spi;
    at45dbxx_spi_cs_port = port;
    at45dbxx_spi_cs_pin = pin;
    is_init = true;
}

static bool at45dbxx_get_addr_packed (uint32_t addr_in, uint8_t *addr_out, size_t addr_out_size)
{
    const uint32_t mask_overflow = 0xFFE00000;

    if (addr_out == NULL || addr_out_size > addr_size || addr_in & mask_overflow)
        return false;

    // | X X P P P P P P | P P P P P P B B | B B B B B B B B |
    // P = Page Address Bit B = Byte/Buffer
    union {
        uint16_t addr_16;
        uint8_t addr_8[2];
    } addr;

    const uint8_t two_bits_offset = 2;
    addr.addr_16 = (addr_in << two_bits_offset) & 0x3FFC;
    memcpy (addr_out, &addr.addr_8[0], sizeof (addr));
    const uint8_t byte = 0;
    addr_out[2] = byte;

    return true;
}

bool at45dbxx_rd_id (uint8_t *id, size_t id_size)
{
    if (!is_init)
        return false;

    HAL_StatusTypeDef status = HAL_OK;

    const uint8_t opcode = AT45DBXX_OPCODE_GET_ID;
    HAL_GPIO_WritePin (at45dbxx_spi_cs_port, at45dbxx_spi_cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit (&at45dbxx_spi, &opcode, sizeof (opcode), HAL_MAX_DELAY);
    status = HAL_SPI_Receive (&at45dbxx_spi, id, id_size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin (at45dbxx_spi_cs_port, at45dbxx_spi_cs_pin, GPIO_PIN_SET);

    return (status == HAL_OK);
}

bool at45dbxx_wr_data (const at45dbxx_config_t * const config)
{
    if (!is_init)
        return false;

    uint8_t addr[addr_size];
    if (!at45dbxx_get_addr_packed (config->addr, addr, addr_size))
        return false;

    const size_t opcode__addr_len = addr_size + sizeof(config->opcode);
    uint8_t opcode__addr[opcode__addr_len];
    opcode__addr[0] = config->opcode;
    memcpy (&opcode__addr[1], addr, addr_size);

    HAL_StatusTypeDef status = HAL_OK;
    HAL_GPIO_WritePin (at45dbxx_spi_cs_port, at45dbxx_spi_cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit (&at45dbxx_spi, opcode__addr, (uint16_t)opcode__addr_len, HAL_MAX_DELAY);
    status = HAL_SPI_Transmit (&at45dbxx_spi, config->msg, (uint16_t)config->msg_len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin (at45dbxx_spi_cs_port, at45dbxx_spi_cs_pin, GPIO_PIN_SET);

    return (status == HAL_OK);
}

bool at45dbxx_rd_data (at45dbxx_config_t * const config)
{
    if (!is_init)
        return false;

    uint8_t addr[addr_size];
    if (!at45dbxx_get_addr_packed (config->addr, addr, addr_size))
        return false;

    const uint8_t dummy = 0x0;
    const size_t opcode__addr_len = addr_size + sizeof(config->opcode) + sizeof (dummy);
    uint8_t opcode__addr[opcode__addr_len];
    opcode__addr[0] = config->opcode;
    memcpy (&opcode__addr[1], addr, addr_size);
    opcode__addr[4] = dummy;

    HAL_StatusTypeDef status = HAL_OK;
    HAL_GPIO_WritePin (at45dbxx_spi_cs_port, at45dbxx_spi_cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit (&at45dbxx_spi, opcode__addr, (uint16_t)opcode__addr_len, HAL_MAX_DELAY);
    status = HAL_SPI_Receive (&at45dbxx_spi, config->msg, (uint16_t)config->msg_len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin (at45dbxx_spi_cs_port, at45dbxx_spi_cs_pin, GPIO_PIN_SET);

    return (status == HAL_OK);
}
