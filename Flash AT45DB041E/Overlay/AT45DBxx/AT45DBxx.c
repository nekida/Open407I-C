#include "AT45DBxx.h"

#define ADDR_SIZE (3)

const uint8_t at45dbxx_opcode_get_id = 0x9F;
const uint8_t at45dbxx_opcode_wr_page_on_buf1 = 0x82;
const uint8_t at45dbxx_opcode_wr_page_on_buf2 = 0x85;
const uint8_t at45dbxx_opcode_cont_arr_rd_hf = 0x0B;

bool at45dbxx_get_addr_packed (uint32_t addr_in, uint8_t *addr_out, size_t addr_out_size)
{
    const uint32_t mask_overflow = 0xFFE00000;
    if (addr_out == NULL || addr_out_size > ADDR_SIZE || addr_in & mask_overflow)
        return false;

    const uint8_t one_byte_offset = 8;
    const uint8_t two_byte_offset = 16;
    addr_out[0] = addr_in & 0xFF;
    addr_out[1] = (addr_in >> one_byte_offset) & 0xFF;
    addr_out[2] = (addr_in >> two_byte_offset) & 0xFF;

    return true;
}
