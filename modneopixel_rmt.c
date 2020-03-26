/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "py/runtime.h"
#include "neopixel_rmt.h"

STATIC mp_obj_t init() {
    pixel_init();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(init_obj, init);

STATIC mp_obj_t write(mp_obj_t buf) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_READ);

    showPixels((uint8_t*)bufinfo.buf, 0, (uint16_t)bufinfo.len);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(write_obj, write);


STATIC const mp_rom_map_elem_t mp_module_neopixel_rmt_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_neopixel_rmt) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&init_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&write_obj) },
};
STATIC MP_DEFINE_CONST_DICT(mp_module_neopixel_rmt_globals, mp_module_neopixel_rmt_globals_table);

const mp_obj_module_t neopixel_rmt_module = {
    .base = { &mp_type_module },
    .globals = (void*)&mp_module_neopixel_rmt_globals,
};
MP_REGISTER_MODULE(MP_QSTR_neopixel_rmt, neopixel_rmt_module, NEOPIXEL_RMT_ENABLED);