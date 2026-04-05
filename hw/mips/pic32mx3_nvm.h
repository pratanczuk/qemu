/*
 * PIC32MX3 NVM (flash controller) — program/erase backed by QEMU flash RAM.
 */
#ifndef HW_MIPS_PIC32MX3_NVM_H
#define HW_MIPS_PIC32MX3_NVM_H

#include <stdint.h>

struct _pic32_t;

void pic32mx3_nvm_bind_flash(struct _pic32_t *s,
                             uint8_t *prog_host, uint8_t *boot_host,
                             uint32_t prog_sz, uint32_t boot_sz,
                             uint32_t prog_bus, uint32_t boot_bus);
void pic32mx3_nvm_reset(struct _pic32_t *s);
void pic32mx3_nvm_key_write(struct _pic32_t *s, uint32_t value);
uint32_t pic32mx3_nvm_nvmcon_write(struct _pic32_t *s, uint32_t old_con,
                                   uint32_t new_con);

void pic32mx3_flash_load_images(struct _pic32_t *s);
void pic32mx3_flash_register_exit_save(struct _pic32_t *s);

#endif
