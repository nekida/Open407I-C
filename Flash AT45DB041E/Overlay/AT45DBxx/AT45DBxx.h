#ifndef AT45DBxx_H
#define AT45DBxx_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define AT45DBxx_PAGE_SIZE (512)
#define AT45DBxx_BLOCK_SIZE (AT45DBxx_PAGE_SIZE * 8)

extern const uint8_t at45dbxx_opcode_get_id;
extern const uint8_t at45dbxx_opcode_wr_page_on_buf1;
extern const uint8_t at45dbxx_opcode_wr_page_on_buf2;
extern const uint8_t at45dbxx_opcode_cont_arr_rd_hf;

bool at45dbxx_get_addr_packed (uint32_t addr_in, uint8_t *addr_out, size_t addr_out_size);

#endif // AT45DBxx_H
