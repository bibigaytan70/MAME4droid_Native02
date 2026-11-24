/* license:BSD-3-Clause */
/* copyright-holders:Aaron Giles */
/*************************************************************************

    Video hardware emulation for Midway Zeus (C99 / Classic MAME port)

**************************************************************************/

#include "emu.h"
#include "includes/midzeus.h"

#include "video/poly.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define LOG_ZEUS    (1U << 1)
#define LOG_PTR     (1U << 2)
#define LOG_REGS    (1U << 3)
#define LOG_CMD     (1U << 4)
#define LOG_MODEL   (1U << 5)
#define LOG_QUAD    (1U << 6)
#define LOG_FIFO    (1U << 7)
#define LOG_WAVE    (1U << 8)

#define LOG_ALL     (LOG_ZEUS | LOG_PTR | LOG_REGS | LOG_CMD | LOG_MODEL | LOG_QUAD | LOG_FIFO | LOG_WAVE)

#define VERBOSE     (0)
#define LOGMASKED(mask, ...) do { if (VERBOSE & (mask)) logerror(__VA_ARGS__); } while (0)

#define LOGZEUS(...)   LOGMASKED(LOG_ZEUS, __VA_ARGS__)
#define LOGPTR(...)    LOGMASKED(LOG_PTR, __VA_ARGS__)
#define LOGREGS(...)   LOGMASKED(LOG_REGS, __VA_ARGS__)
#define LOGCMD(...)    LOGMASKED(LOG_CMD, __VA_ARGS__)
#define LOGMODEL(...)  LOGMASKED(LOG_MODEL, __VA_ARGS__)
#define LOGQUAD(...)   LOGMASKED(LOG_QUAD, __VA_ARGS__)
#define LOGFIFO(...)   LOGMASKED(LOG_FIFO, __VA_ARGS__)
#define LOGWAVE(...)   LOGMASKED(LOG_WAVE, __VA_ARGS__)


/*************************************
 *  Constantes
 *************************************/

#define DUMP_WAVE_RAM       0
#define DEBUG_KEYS          0

#define WAVERAM0_WIDTH      512U
#define WAVERAM0_HEIGHT     2048U

#define WAVERAM1_WIDTH      512U
#define WAVERAM1_HEIGHT     512U

#define BLEND_OPAQUE1       0x00000000U
#define BLEND_OPAQUE2       0x4b23cb00U
#define BLEND_OPAQUE3       0x4b23dd00U
#define BLEND_OPAQUE4       0x00004800U
#define BLEND_OPAQUE5       0xdd23dd00U
#define BLEND_ADD1          0x40b68800U
#define BLEND_ADD2          0xc9b78800U
#define BLEND_MUL1          0x4093c800U

#ifndef ARRAY_LENGTH
#define ARRAY_LENGTH(x)     (sizeof(x) / sizeof((x)[0]))
#endif

static inline float max_float(float a, float b)
{
    return (a > b) ? a : b;
}

static inline uint8_t pal5bit_c(uint32_t bits)
{
    bits &= 0x1f;
    return (uint8_t)((bits << 3) | (bits >> 2));
}

static inline int rect_contains(const rectangle *r, int x, int y)
{
    return (x >= r->min_x && x <= r->max_x && y >= r->min_y && y <= r->max_y);
}


/*************************************
 *  Macros de acceso a WaveRAM
 *************************************/

#define WAVERAM_BLOCK0(state, blocknum)         ((void *)((uint8_t *)(state)->waveram[0] + 8 * (blocknum)))
#define WAVERAM_BLOCK1(state, blocknum)         ((void *)((uint8_t *)(state)->waveram[1] + 8 * (blocknum)))

#define WAVERAM_PTR8(base, bytenum)             ((uint8_t *)(base) + BYTE4_XOR_LE(bytenum))
#define WAVERAM_READ8(base, bytenum)            (*WAVERAM_PTR8(base, bytenum))
#define WAVERAM_WRITE8(base, bytenum, data)     do { *WAVERAM_PTR8(base, bytenum) = (data); } while (0)

#define WAVERAM_PTR16(base, wordnum)            ((uint16_t *)(base) + BYTE_XOR_LE(wordnum))
#define WAVERAM_READ16(base, wordnum)           (*WAVERAM_PTR16(base, wordnum))
#define WAVERAM_WRITE16(base, wordnum, data)    do { *WAVERAM_PTR16(base, wordnum) = (data); } while (0)

#define WAVERAM_PTR32(base, dwordnum)           ((uint32_t *)(base) + (dwordnum))
#define WAVERAM_READ32(base, dwordnum)          (*WAVERAM_PTR32(base, dwordnum))
#define WAVERAM_WRITE32(base, dwordnum, data)   do { *WAVERAM_PTR32(base, dwordnum) = (data); } while (0)

#define PIXYX_TO_WORDNUM(y, x)                  (((y) << 10) | (((x) & 0x1fe) << 1) | ((x) & 1))
#define DEPTHYX_TO_WORDNUM(y, x)                (PIXYX_TO_WORDNUM(y, x) | 2)

#define WAVERAM_PTRPIX(base, y, x)              WAVERAM_PTR16(base, PIXYX_TO_WORDNUM(y, x))
#define WAVERAM_READPIX(base, y, x)             (*WAVERAM_PTRPIX(base, y, x))
#define WAVERAM_WRITEPIX(base, y, x, color)     do { *WAVERAM_PTRPIX(base, y, x) = (color); } while (0)

#define WAVERAM_PTRDEPTH(base, y, x)            WAVERAM_PTR16(base, DEPTHYX_TO_WORDNUM(y, x))
#define WAVERAM_READDEPTH(base, y, x)           (*WAVERAM_PTRDEPTH(base, y, x))
#define WAVERAM_WRITEDEPTH(base, y, x, color)   do { *WAVERAM_PTRDEPTH(base, y, x) = (color); } while (0)


/*************************************
 *  Prototipos internos
 *************************************/

static void zeus_pointer_w(running_machine *machine, uint32_t which, uint32_t data, bool logit);
static void zeus_register16_w(running_machine *machine, offs_t offset, uint16_t data, bool logit);
static void zeus_register32_w(running_machine *machine, offs_t offset, uint32_t data, bool logit);
static void zeus_register_update(running_machine *machine, offs_t offset);
static int  zeus_fifo_process(running_machine *machine, const uint32_t *data, int numwords);
static void zeus_draw_model(running_machine *machine, uint32_t texdata, bool logit);
static void zeus_draw_quad(running_machine *machine, int long_fmt, const uint32_t *databuffer, uint32_t texdata, bool logit);
static void zeus_draw_solid_quad(running_machine *machine, const rectangle *rect, const poly_vertex *vert);
static void render_poly(void *dest, int32_t scanline, const poly_extent *extent, const void *extradata, int threadid);
static void render_poly_solid_fixedz(void *dest, int32_t scanline, const poly_extent *extent, const void *extradata, int threadid);
static void log_fifo_command(const uint32_t *data, int numwords, const char *suffix);
static void log_waveram(midzeus_state *state, uint32_t length_and_base);


/*************************************
 *  Funciones inline de direccionamiento
 *************************************/

static inline void *waveram0_ptr_from_block_addr(midzeus_state *state, uint32_t addr)
{
    uint32_t blocknum = (addr % WAVERAM0_WIDTH) + ((addr >> 12) % WAVERAM0_HEIGHT) * WAVERAM0_WIDTH;
    return WAVERAM_BLOCK0(state, blocknum);
}

static inline void *waveram0_ptr_from_expanded_addr(midzeus_state *state, uint32_t addr)
{
    uint32_t blocknum = (addr % WAVERAM0_WIDTH) + ((addr >> 16) % WAVERAM0_HEIGHT) * WAVERAM0_WIDTH;
    return WAVERAM_BLOCK0(state, blocknum);
}

static inline void *waveram1_ptr_from_expanded_addr(midzeus_state *state, uint32_t addr)
{
    uint32_t blocknum = (addr % WAVERAM1_WIDTH) + ((addr >> 16) % WAVERAM1_HEIGHT) * WAVERAM1_WIDTH;
    return WAVERAM_BLOCK1(state, blocknum);
}

static inline void *waveram0_ptr_from_texture_addr(midzeus_state *state, uint32_t addr, int width)
{
    uint32_t blocknum = (((addr & ~1U) * (uint32_t)width) / 8) % (WAVERAM0_WIDTH * WAVERAM0_HEIGHT);
    return WAVERAM_BLOCK0(state, blocknum);
}


/*************************************
 *  Funciones inline de renderizado y texels
 *************************************/

static inline void waveram_plot_depth(midzeus_state *state, int y, int x, uint16_t color, uint16_t depth)
{
    if (rect_contains(&state->zeus_cliprect, x, y))
    {
        WAVERAM_WRITEPIX(state->zeus_renderbase, y, x, color);
        WAVERAM_WRITEDEPTH(state->zeus_renderbase, y, x, depth);
    }
}

/* Bloques 4x2 */
static inline uint8_t get_texel_4bit(const void *base, int y, int x, int width)
{
    uint32_t byteoffs = (y / 2) * (width * 2) + ((x / 8) << 3) + ((y & 1) << 2) + ((x / 2) & 3);
    return (WAVERAM_READ8(base, byteoffs) >> (4 * (x & 1))) & 0x0f;
}

static inline uint8_t get_texel_8bit(const void *base, int y, int x, int width)
{
    uint32_t byteoffs = (y / 2) * (width * 2) + ((x / 4) << 3) + ((y & 1) << 2) + (x & 3);
    return WAVERAM_READ8(base, byteoffs);
}

/* Bloques 2x2 */
static inline uint8_t get_texel_alt_4bit(const void *base, int y, int x, int width)
{
    uint32_t byteoffs = (y / 4) * (width * 4) + ((x / 4) << 3) + ((y & 3) << 1) + ((x / 2) & 1);
    return (WAVERAM_READ8(base, byteoffs) >> (4 * (x & 1))) & 0x0f;
}

static inline uint8_t get_texel_alt_8bit(const void *base, int y, int x, int width)
{
    uint32_t byteoffs = (y / 4) * (width * 4) + ((x / 2) << 3) + ((y & 3) << 1) + (x & 1);
    return WAVERAM_READ8(base, byteoffs);
}


/*************************************
 *  Inicio y cierre de Video
 *************************************/

static void midzeus_exit_handler(running_machine *machine)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;

#if DUMP_WAVE_RAM
    FILE *f = fopen("waveram.dmp", "w");
    if (f != NULL)
    {
        uint32_t i;
        for (i = 0; i < WAVERAM0_WIDTH * WAVERAM0_HEIGHT; i++)
        {
            if (i % 4 == 0) fprintf(f, "%03X%03X: ", i / WAVERAM0_WIDTH, i % WAVERAM0_WIDTH);
            fprintf(f, " %08X %08X ",
                WAVERAM_READ32(state->waveram[0], i * 2 + 0),
                WAVERAM_READ32(state->waveram[0], i * 2 + 1));
            if (i % 4 == 3) fprintf(f, "\n");
        }
        fclose(f);
    }
#endif

    if (state->poly != NULL)
    {
        poly_free(state->poly);
        state->poly = NULL;
    }

    free(state->waveram[0]);
    free(state->waveram[1]);
    state->waveram[0] = NULL;
    state->waveram[1] = NULL;
}

VIDEO_START(midzeus)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;

    /* Reservar memoria para WaveRAM */
    state->waveram[0] = (uint32_t *)calloc(WAVERAM0_WIDTH * WAVERAM0_HEIGHT * 2, sizeof(uint32_t));
    state->waveram[1] = (uint32_t *)calloc(WAVERAM1_WIDTH * WAVERAM1_HEIGHT * 2, sizeof(uint32_t));

    /* Inicializar motor de polígonos (4 parámetros interpolados, máx 4096 polígonos en cola) */
    state->poly = poly_alloc(machine, 4096, sizeof(mz_poly_extra_data), 0);

    /* Registrar limpieza al salir */
    add_exit_callback(machine, midzeus_exit_handler);

    state->yoffs = 0;
    state->texel_width = 256;
    state->zeus_renderbase = state->waveram[1];

    /* Registro de estados (Save States) */
   

    /* HACK para revisión temprana de MK4 */
    state->is_mk4b = (strcmp(machine->gamedrv->name, "mk4b") == 0);
}


/*************************************
 *  Actualización de Pantalla
 *************************************/

VIDEO_UPDATE(midzeus)
{
    midzeus_state *state = (midzeus_state *)screen->machine->driver_data;
    int x, y;

    poly_wait(state->poly, "VIDEO_UPDATE");

    if (DEBUG_KEYS && input_code_pressed(screen->machine, KEYCODE_V))
    {
        const void *base;

        if (input_code_pressed(screen->machine, KEYCODE_DOWN))
            state->yoffs += input_code_pressed(screen->machine, KEYCODE_LSHIFT) ? 0x40 : 1;
        if (input_code_pressed(screen->machine, KEYCODE_UP))
            state->yoffs -= input_code_pressed(screen->machine, KEYCODE_LSHIFT) ? 0x40 : 1;
        if (input_code_pressed(screen->machine, KEYCODE_LEFT) && state->texel_width > 4)
        {
            state->texel_width >>= 1;
            while (input_code_pressed(screen->machine, KEYCODE_LEFT)) ;
        }
        if (input_code_pressed(screen->machine, KEYCODE_RIGHT) && state->texel_width < 512)
        {
            state->texel_width <<= 1;
            while (input_code_pressed(screen->machine, KEYCODE_RIGHT)) ;
        }

        if (state->yoffs < 0) state->yoffs = 0;
        base = waveram0_ptr_from_block_addr(state, state->yoffs << 12);

        for (y = cliprect->min_y; y <= cliprect->max_y; y++)
        {
            uint16_t *dest = BITMAP_ADDR16(bitmap, y, 0);
            for (x = cliprect->min_x; x <= cliprect->max_x; x++)
            {
                uint8_t tex = get_texel_8bit(base, y, x, state->texel_width);
                dest[x] = (tex << 8) | tex;
            }
        }

        popmessage("offs = %06X", state->yoffs << 12);
    }
    else
    {
        const void *base = waveram1_ptr_from_expanded_addr(state, state->zeusbase[0xcc]);
        int xoffs = video_screen_get_visible_area(screen)->min_x;

        for (y = cliprect->min_y; y <= cliprect->max_y; y++)
        {
            uint16_t *dest = BITMAP_ADDR16(bitmap, y, 0);
            for (x = cliprect->min_x; x <= cliprect->max_x; x++)
                dest[x] = WAVERAM_READPIX(base, y, x - xoffs) & 0x7fff;
        }
    }

    return 0;
}


/*************************************
 *  Manejadores principales de E/S
 *************************************/

READ32_HANDLER(zeus_r)
{
    midzeus_state *state = (midzeus_state *)space->machine->driver_data;
    bool logit = ((offset < 0xb0) || (offset > 0xb7));
    uint32_t result = state->zeusbase[offset & ~1];

    switch (offset & ~1)
    {
        case 0xf0:
            result = video_screen_get_hpos(state->screen);
            logit = false;
            break;

        case 0xf2:
            result = video_screen_get_vpos(state->screen);
            logit = false;
            break;

        case 0xf4:
            result = 6;
            if (video_screen_get_vblank(state->screen))
                result |= 0x800;
            logit = false;
            break;

        case 0xf6: /* Status: bit 0x9600 requerido para pasar test de sistema */
            result = 0x9600;
            if (state->zeusbase[0xb6] == 0x80040000)
                result |= 1;
            logit = false;
            break;
    }

    if (state->zeusbase[0x80] & 0x00020000)
    {
        /* Modo 32 bits */
        if (offset & 1)
            result >>= 16;
        if (logit)
        {
            if (offset & 1)
                LOGZEUS("%06X:zeus32_r(%02X) = %08X -- unexpected in 32-bit mode\n", cpu_get_pc(state->maincpu), offset, result);
            else
                LOGZEUS("%06X:zeus32_r(%02X) = %08X\n", cpu_get_pc(state->maincpu), offset, result);
        }
    }
    else
    {
        /* Modo 16 bits */
        if (offset & 1)
            result >>= 16;
        else
            result &= 0xffff;
        if (logit)
            LOGZEUS("%06X:zeus16_r(%02X) = %04X\n", cpu_get_pc(state->maincpu), offset, result);
    }
    return result;
}

WRITE32_HANDLER(zeus_w)
{
    midzeus_state *state = (midzeus_state *)space->machine->driver_data;
    bool logit = state->zeus_enable_logging || ((offset < 0xb0 || offset > 0xb7) && (offset < 0xe0 || offset > 0xe1));

    if (logit)
        LOGZEUS("%06X:zeus_w", cpu_get_pc(state->maincpu));

    if (state->zeusbase[0x80] & 0x00020000)
        zeus_register32_w(space->machine, offset, data, logit);
    else
        zeus_register16_w(space->machine, offset, (uint16_t)data, logit);
}


/*************************************
 *  Escritura de punteros y registros
 *************************************/

static void zeus_pointer_w(running_machine *machine, uint32_t which, uint32_t data, bool logit)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;

    switch (which & 0xffffff)
    {
        case 0x008000:
        case 0x018000:
            if (logit) LOGPTR(" -- setptr(objdata)\n");
            state->zeus_objdata = data;
            break;

        case 0x00c040:
            if (logit) LOGPTR(" -- setptr(palbase)\n");
            state->zeus_palbase = data;
            break;

        case 0x02c0f0:
            if (logit) LOGPTR(" -- setptr(unkbase)\n");
            state->zeus_unkbase = data;
            break;

        default:
            if (logit) LOGPTR(" -- setptr(%06X)\n", which & 0xffffff);
            break;
    }

    if (logit)
        log_waveram(state, data);
}

static void zeus_register16_w(running_machine *machine, offs_t offset, uint16_t data, bool logit)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;

    if ((offset & ~1) == 0xcc)
        video_screen_update_partial(state->screen, video_screen_get_vpos(state->screen));

    if (offset & 1)
        state->zeusbase[offset & ~1] = (state->zeusbase[offset & ~1] & 0x0000ffff) | ((uint32_t)data << 16);
    else
        state->zeusbase[offset & ~1] = (state->zeusbase[offset & ~1] & 0xffff0000) | (data & 0xffff);

    if (logit)
        LOGREGS("(%02X) = %04X [%08X]\n", offset, data & 0xffff, state->zeusbase[offset & ~1]);

    if ((offset & 1) == 0)
        zeus_register_update(machine, offset);
}

static void zeus_register32_w(running_machine *machine, offs_t offset, uint32_t data, bool logit)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;

    if ((offset & ~1) == 0xcc)
        video_screen_update_partial(state->screen, video_screen_get_vpos(state->screen));

    state->zeusbase[offset & ~1] = data;

    if (logit)
    {
        if (offset & 1)
            LOGREGS("(%02X) = %08X -- unexpected in 32-bit mode\n", offset, data);
        else
            LOGREGS("(%02X) = %08X\n", offset, data);
    }

    if ((offset & 1) == 0)
        zeus_register_update(machine, offset);
}


/*************************************
 *  Actualización de estado de registros
 *************************************/

static void zeus_register_update(running_machine *machine, offs_t offset)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;

    switch (offset)
    {
        case 0x52:
            state->zeusbase[0xb2] = state->zeusbase[0x52];
            break;

        case 0x60:
            if (state->zeusbase[0x60] & 1)
            {
                if ((state->zeusbase[0x80] & 0xffffff) == 0x22FCFF)
                {
                    poly_vertex vert[4];

                    vert[0].x = (int16_t)state->zeusbase[0x08];
                    vert[0].y = (int16_t)(state->zeusbase[0x08] >> 16);
                    vert[1].x = (int16_t)state->zeusbase[0x0a];
                    vert[1].y = (int16_t)(state->zeusbase[0x0a] >> 16);
                    vert[2].x = (int16_t)state->zeusbase[0x0c];
                    vert[2].y = (int16_t)(state->zeusbase[0x0c] >> 16);
                    vert[3].x = (int16_t)state->zeusbase[0x0e];
                    vert[3].y = (int16_t)(state->zeusbase[0x0e] >> 16);

                    state->solid_color_override = state->zeusbase[0x00];
                    state->solid_z_override = 0x7fff;

                    zeus_draw_solid_quad(machine, &state->zeus_cliprect, vert);
                    poly_wait(state->poly, "Normal");
                }
                else
                {
                    LOGCMD("Execute unknown command\n");
                }
            }
            break;

        case 0x70:
            state->zeus_point[0] = state->zeusbase[0x70] << 16;
            break;

        case 0x72:
            state->zeus_point[1] = state->zeusbase[0x72] << 16;
            break;

        case 0x74:
            state->zeus_point[2] = state->zeusbase[0x74] << 16;
            break;

        case 0x80:
            if (state->zeusbase[0x80] & 0x02000000)
                cpu_set_input_line(state->maincpu, 2, ASSERT_LINE);
            else
                cpu_set_input_line(state->maincpu, 2, CLEAR_LINE);
            break;

        case 0x84:
            state->zeus_renderbase = waveram1_ptr_from_expanded_addr(state, state->zeusbase[0x84] << 16);
            break;

        case 0xb0:
        case 0xb2:
            if ((state->zeusbase[0xb6] >> 16) != 0)
            {
                if ((offset == 0xb0 && (state->zeusbase[0xb6] & 0x02000000) == 0) ||
                    (offset == 0xb2 && (state->zeusbase[0xb6] & 0x02000000) != 0))
                {
                    void *dest;

                    if (state->zeusbase[0xb6] & 0x80000000)
                        dest = waveram1_ptr_from_expanded_addr(state, state->zeusbase[0xb4]);
                    else
                        dest = waveram0_ptr_from_expanded_addr(state, state->zeusbase[0xb4]);

                    if (state->zeusbase[0xb6] & 0x00100000)
                        WAVERAM_WRITE16(dest, 0, state->zeusbase[0xb0]);
                    if (state->zeusbase[0xb6] & 0x00200000)
                        WAVERAM_WRITE16(dest, 1, state->zeusbase[0xb0] >> 16);
                    if (state->zeusbase[0xb6] & 0x00400000)
                        WAVERAM_WRITE16(dest, 2, state->zeusbase[0xb2]);
                    if (state->zeusbase[0xb6] & 0x00800000)
                        WAVERAM_WRITE16(dest, 3, state->zeusbase[0xb2] >> 16);
                    if (state->zeusbase[0xb6] & 0x00020000)
                        state->zeusbase[0xb4]++;
                }
            }
            break;

        case 0xb4:
            if (state->zeusbase[0xb6] & 0x00010000)
            {
                const uint32_t *src;

                if (state->zeusbase[0xb6] & 0x80000000)
                    src = (const uint32_t *)waveram1_ptr_from_expanded_addr(state, state->zeusbase[0xb4]);
                else
                    src = (const uint32_t *)waveram0_ptr_from_expanded_addr(state, state->zeusbase[0xb4]);

                poly_wait(state->poly, "vram_read");
                state->zeusbase[0xb0] = WAVERAM_READ32(src, 0);
                state->zeusbase[0xb2] = WAVERAM_READ32(src, 1);
            }
            break;

        case 0xc0:
        case 0xc2:
        case 0xc4:
        case 0xc6:
        case 0xc8:
        case 0xca:
            video_screen_update_partial(state->screen, video_screen_get_vpos(state->screen));
            {
                int vtotal = state->zeusbase[0xca] >> 16;
                int htotal = state->zeusbase[0xc6] >> 16;
                rectangle visarea;

                visarea.min_x = state->zeusbase[0xc6] & 0xffff;
                visarea.max_x = htotal - 3;
                visarea.min_y = 0;
                visarea.max_y = state->zeusbase[0xc8] & 0xffff;

                if (htotal > 0 && vtotal > 0 && visarea.min_x < visarea.max_x && visarea.max_y < vtotal)
                {
                    video_screen_configure(state->screen, htotal, vtotal, &visarea,
                                           HZ_TO_ATTOSECONDS(MIDZEUS_VIDEO_CLOCK / 8) * htotal * vtotal);
                    state->zeus_cliprect = visarea;
                    state->zeus_cliprect.max_x -= state->zeus_cliprect.min_x;
                    state->zeus_cliprect.min_x = 0;
                }
            }
            midzeus_rearm_vsync(machine);
            break;

        case 0xcc:
            video_screen_update_partial(state->screen, video_screen_get_vpos(state->screen));
            if (DEBUG_KEYS)
                state->log_fifo = input_code_pressed(machine, KEYCODE_L);
            else
                state->log_fifo = 0;
            break;

        case 0xe0:
            state->zeus_fifo[state->zeus_fifo_words++] = state->zeusbase[0xe0];
            if (zeus_fifo_process(machine, state->zeus_fifo, state->zeus_fifo_words))
                state->zeus_fifo_words = 0;
            break;
    }
}


/*************************************
 *  Procesamiento de la cola FIFO
 *************************************/

static int zeus_fifo_process(running_machine *machine, const uint32_t *data, int numwords)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;

    switch (data[0] >> 24)
    {
        case 0x00:
        case 0x01:
            if (numwords < 2 && data[0] != 0)
                return 0;
            if (state->log_fifo)
                log_fifo_command(data, numwords, "");
            zeus_pointer_w(machine, data[0] & 0xffffff, data[1], state->log_fifo != 0);
            break;

        case 0x13: /* Renderizar modelo (Invasion) */
            if (state->log_fifo)
                log_fifo_command(data, numwords, "");
            zeus_draw_model(machine, (state->zeusbase[0x06] << 16), state->log_fifo != 0);
            break;

        case 0x17: /* Escribir 16 bits a registros bajos */
            if (state->log_fifo)
                log_fifo_command(data, numwords, " -- reg16");
            zeus_register16_w(machine, (data[0] >> 16) & 0x7f, (uint16_t)data[0], state->log_fifo != 0);
            break;

        case 0x18: /* Escribir 32 bits a registros bajos */
            if (numwords < 2)
                return 0;
            if (state->log_fifo)
                log_fifo_command(data, numwords, " -- reg32");
            zeus_register32_w(machine, (data[0] >> 16) & 0x7f, data[1], state->log_fifo != 0);
            break;

        case 0x1a:
        case 0x1b:
            if (state->log_fifo)
                log_fifo_command(data, numwords, " -- sync\n");
            break;

        case 0x1c:
        case 0x1e:
            if ((data[0] & 0xffff) != 0x7fff)
            {
                /* Matriz simple (8 palabras) */
                if (numwords < 8)
                    return 0;
                if (state->log_fifo)
                {
                    log_fifo_command(data, numwords, "");
                    LOGCMD("\n\t\
