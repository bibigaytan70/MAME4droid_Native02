/* license:BSD-3-Clause */
/* copyright-holders:Aaron Giles */
/*************************************************************************

    Driver for Midway Zeus / Zeus 2 games (C99 / Classic MAME style port)

    Games supported:
        * Invasion - The Abductors
        * Mortal Kombat 4
        * Cruis'n Exotica
        * The Grid

**************************************************************************/

#include "emu.h"
#include "includes/midzeus.h"
#include "dcs.h"
#include "cpu/adsp2100/adsp2100.h"
#include "cpu/pic16c5x/pic16c5x.h"
#include "cpu/tms32031/tms32031.h"
#include "machine/nvram.h"
#include "video/zeus2.h"
#include "speaker.h"
#include "layout/crusnexo.lay"

#include "cpu/tms34010/tms34010.h"
#include "machine/midwayic.h"
#include "machine/timekpr.h"


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Constantes de pistola óptica (reemplazo de constexpr) */
#define BEAM_XOFFS      40
#define BEAM_DY         3
#define BEAM_DX         3

/* Flags de depuración */
#define LOG_FIREWIRE    (1U << 1)
#define LOG_DISK        (1U << 2)
#define LOG_DISK_JR     (1U << 3)
#define LOG_TMS320C32   (1U << 4)
#define LOG_INPUT       (1U << 5)
#define LOG_CMOS        (1U << 6)
#define LOG_UNKNOWN     (1U << 7)

#define VERBOSE         (LOG_FIREWIRE)
#define LOGMASKED(mask, ...) do { if (VERBOSE & (mask)) logerror(__VA_ARGS__); } while (0)

#ifndef BIT
#define BIT(x, n) (((x) >> (n)) & 1U)
#endif

static inline int clamp_int(int val, int min_val, int max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static inline int max_int(int a, int b)
{
    return (a > b) ? a : b;
}

/*************************************
 *  Estructuras de Estado (Structs)
 *************************************/

/* Estructura derivada para Zeus 2 (embebe midzeus_state como base) */
typedef struct _midzeus2_state
{
    midzeus_state base;

    uint32_t    disk_asic[0x10];
    int         fw_int_enable;
    int         fw_int;

    device_t    *zeus;
    device_t    *m48t35;
    device_t    *fw_link;
    device_t    *fw_phy;
    ioport_port *io_keypad;
} midzeus2_state;

/* Estructura derivada para Cruis'n Exotica */
typedef struct _crusnexo_state
{
    midzeus2_state base;

    uint8_t     keypad_select;
    uint8_t     crusnexo_leds_select;

    ioport_port *io_analog[4];
} crusnexo_state;

/* Estructura derivada para The Grid */
typedef struct _thegrid_state
{
    midzeus2_state base;

    ioport_port *io_49way_x;
    ioport_port *io_49way_y;
    ioport_port *io_trackx;
    ioport_port *io_tracky;
} thegrid_state;


/*************************************
 *  Prototipos de funciones
 *************************************/

static void midzeus_rearm_vsync(running_machine *machine);
static void midzeus2_update_firewire_irq(running_machine *machine);
static void invasnab_update_gun_irq(running_machine *machine);


/*************************************
 *  Inicialización y Reset (Machine)
 *************************************/

static TIMER_CALLBACK(midzeus_vsync_start_cb)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;
    if (state->maincpu != NULL)
        cpu_set_input_line(state->maincpu, TMS320C3X_IRQ0, ASSERT_LINE);
    midzeus_rearm_vsync(machine);
}

static MACHINE_START(midzeus)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;

    state->timer[0] = timer_alloc(machine, NULL, NULL);
    state->timer[1] = timer_alloc(machine, NULL, NULL);
    state->vsync_start_timer = timer_alloc(machine, midzeus_vsync_start_cb, NULL);

    state_save_register_global_array(machine, state->disk_asic_jr);
    state_save_register_global(machine, state->cmos_protected);
}

static MACHINE_START(midzeus2)
{
    midzeus2_state *state = (midzeus2_state *)machine->driver_data;

    MACHINE_START_CALL(midzeus);

    memory_configure_bank(machine, "bank1", 0, 3, memory_region(machine, "bankeddata"), 0x400000 * 4);
    memory_set_bank(machine, "bank1", 0);

    state_save_register_global_array(machine, state->disk_asic);
    state_save_register_global(machine, state->fw_int_enable);
    state_save_register_global(machine, state->fw_int);
}

static MACHINE_START(crusnexo)
{
    crusnexo_state *state = (crusnexo_state *)machine->driver_data;

    MACHINE_START_CALL(midzeus2);

    state_save_register_global(machine, state->keypad_select);
    state_save_register_global(machine, state->crusnexo_leds_select);
}

static TIMER_CALLBACK(invasn_gun_callback);

static MACHINE_START(invasnab)
{
    invasnab_state *state = (invasnab_state *)machine->driver_data;

    MACHINE_START_CALL(midzeus);

    state->gun_timer[0] = timer_alloc(machine, invasn_gun_callback, NULL);
    state->gun_timer[1] = timer_alloc(machine, invasn_gun_callback, NULL);

    state_save_register_global(machine, state->gun_control);
    state_save_register_global(machine, state->gun_irq_state);
    state_save_register_global_array(machine, state->gun_x);
    state_save_register_global_array(machine, state->gun_y);
}

static MACHINE_RESET(midzeus)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;

    memcpy(state->ram_base, memory_region(machine, "maindata"), 0x40000 * 4);
    *state->ram_base <<= 1;
    cpu_reset(state->maincpu);

    state->cmos_protected = true;
    memset(state->disk_asic_jr, 0x0, 0x10 * 4);
    state->disk_asic_jr[6] = 0xa0; /* Rev3 Athens */

    midzeus_rearm_vsync(machine);
}

static MACHINE_RESET(midzeus2)
{
    midzeus2_state *state = (midzeus2_state *)machine->driver_data;

    MACHINE_RESET_CALL(midzeus);

    memset(state->disk_asic, 0x0, 0x10 * 4);
    state->fw_int_enable = 0;
    state->fw_int = 0;
}


/*************************************
 *  Sincronización Vertical (VSync)
 *************************************/

static attotime midzeus_time_until_vsync(running_machine *machine)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;
    const rectangle *visarea = video_screen_get_visible_area(state->screen);
    uint32_t line = state->zeusbase[0xc8] >> 16;

    if (line > (uint32_t)visarea->max_y && line < (uint32_t)video_screen_get_height(state->screen))
        return video_screen_get_time_until_pos(state->screen, line, 0);

    return video_screen_get_time_until_vblank_start(state->screen);
}

static void midzeus_vsync_changed(running_machine *machine, int irq_state)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;
    if (irq_state)
        cpu_pulse_input_line(state->maincpu, TMS320C3X_IRQ0, attotime_mul(ATTOTIME_IN_HZ(60000000), 2));
}

static void midzeus_rearm_vsync(running_machine *machine)
{
    midzeus_state *state = (midzeus_state *)machine->driver_data;
    /* En Zeus 2 el VSync lo maneja el propio dispositivo */
    if (!state->is_zeus2 && state->vsync_start_timer != NULL)
        timer_adjust_oneshot(state->vsync_start_timer, midzeus_time_until_vsync(machine), 0);
}

static void midzeus2_zeus_irq(device_t *device, int irq_state)
{
    midzeus_state *state = (midzeus_state *)device->machine->driver_data;
    cpu_set_input_line(state->maincpu, TMS320C3X_IRQ2, ASSERT_LINE);
}


/*************************************
 *  Acceso CMOS (Solo Zeus 1)
 *************************************/

static WRITE32_HANDLER(cmos_w)
{
    midzeus_state *state = (midzeus_state *)space->machine->driver_data;
    if (state->disk_asic_jr[2] && !state->cmos_protected)
        COMBINE_DATA(&state->nvram[offset]);
    else
        LOGMASKED(LOG_CMOS, "%06X:cmos_w with disk_asic_jr[2] = %d, cmos_protected = %d\n",
                  cpu_get_pc(state->maincpu), state->disk_asic_jr[2], state->cmos_protected);

    state->cmos_protected = true;
}

static READ32_HANDLER(cmos_r)
{
    midzeus_state *state = (midzeus_state *)space->machine->driver_data;
    return state->nvram[offset] | 0xffffff00;
}

static WRITE32_HANDLER(cmos_protect_w)
{
    midzeus_state *state = (midzeus_state *)space->machine->driver_data;
    state->cmos_protected = false;
}


/*************************************
 *  Timekeeper y ZPRAM (Solo Zeus 2)
 *************************************/

static READ32_HANDLER(timekeeper_r)
{
    midzeus2_state *state = (midzeus2_state *)space->machine->driver_data;
    return timekeeper_read(state->m48t35, offset) | 0xffffff00;
}

static WRITE32_HANDLER(timekeeper_w)
{
    midzeus2_state *state = (midzeus2_state *)space->machine->driver_data;
    if (state->base.disk_asic_jr[2] && !state->base.cmos_protected)
        timekeeper_write(state->m48t35, offset, data);
    else
        LOGMASKED(LOG_CMOS, "%s:timekeeper_w with disk_asic_jr[2] = %d, cmos_protected = %d\n",
                  cpuexec_describe_context(space->machine), state->base.disk_asic_jr[2], state->base.cmos_protected);

    state->base.cmos_protected = true;
}

static READ32_HANDLER(zpram_r)
{
    midzeus2_state *state = (midzeus2_state *)space->machine->driver_data;
    return state->base.nvram[offset] | 0xffffff00;
}

static WRITE32_HANDLER(zpram_w)
{
    midzeus2_state *state = (midzeus2_state *)space->machine->driver_data;
    if (state->base.disk_asic_jr[2])
        COMBINE_DATA(&state->base.nvram[offset]);
    else
        LOGMASKED(LOG_CMOS, "%06X:zpram_w with disk_asic_jr[2] = %d\n",
                  cpu_get_pc(state->base.maincpu), state->base.disk_asic_jr[2]);
}


/*************************************
 *  Registros Disk ASIC
 *************************************/

static READ32_HANDLER(disk_asic_r)
{
    midzeus2_state *state = (midzeus2_state *)space->machine->driver_data;
    uint32_t retVal = state->disk_asic[offset];

    switch (offset)
    {
        case 0: /* Sys Control: Bit 0 zeus reset, Bit 3 io reset, Bits 15:8 Version */
            retVal = (retVal & 0xffff00ff) | 0x3300;
            LOGMASKED(LOG_DISK, "%s: Disk ASIC System Control Read: %08x\n", cpuexec_describe_context(space->machine), retVal);
            break;

        case 1: /* Interrupt Status / Control */
            retVal = (state->fw_int << 9);
            LOGMASKED(LOG_DISK, "%s: Disk ASIC Interrupt Status/Ctrl Read: %08x\n", cpuexec_describe_context(space->machine), retVal);
            break;

        case 2:
        case 3:
        case 4:
        case 5:
            LOGMASKED(LOG_DISK, "%s: Disk ASIC Reg %d Read: %08x\n", cpuexec_describe_context(space->machine), offset, retVal);
            break;

        default:
            LOGMASKED(LOG_DISK | LOG_UNKNOWN, "%s: Disk ASIC Unknown Read: %08x\n", cpuexec_describe_context(space->machine), offset);
            break;
    }
    return retVal;
}

static WRITE32_HANDLER(disk_asic_w)
{
    midzeus2_state *state = (midzeus2_state *)space->machine->driver_data;
    state->disk_asic[offset] = data;

    switch (offset)
    {
        case 0:
            LOGMASKED(LOG_DISK, "%s: Disk ASIC System Control Write: %08x\n", cpuexec_describe_context(space->machine), data);
            break;

        case 1:
            LOGMASKED(LOG_DISK, "%s: Disk ASIC Interrupt Status/Ctrl Write: %08x\n", cpuexec_describe_context(space->machine), data);
            state->fw_int_enable = BIT(data, 9);
            midzeus2_update_firewire_irq(space->machine);
            break;

        case 2:
        case 3:
        case 4:
        case 5:
            LOGMASKED(LOG_DISK, "%s: Disk ASIC Reg %d Write: %08x\n", cpuexec_describe_context(space->machine), offset, data);
            break;

        default:
            LOGMASKED(LOG_DISK | LOG_UNKNOWN, "%s: Disk ASIC Unknown Write: %08x = %08x\n", cpuexec_describe_context(space->machine), offset, data);
            break;
    }
}


/*************************************
 *  Registros Disk ASIC JR
 *************************************/

static READ32_HANDLER(disk_asic_jr_r)
{
    midzeus_state *state = (midzeus_state *)space->machine->driver_data;
    uint32_t retVal = state->disk_asic_jr[offset];

    switch (offset)
    {
        case 1:
        case 2:
        case 3:
        case 5:
        case 6:
            break;

        default:
            LOGMASKED(LOG_DISK_JR, "%06X:disk_asic_jr_r(%X)\n", cpu_get_pc(state->maincpu), offset);
            break;
    }
    return retVal;
}

static WRITE32_HANDLER(disk_asic_jr_w)
{
    midzeus_state *state = (midzeus_state *)space->machine->driver_data;
    state->disk_asic_jr[offset] = data;

    switch (offset)
    {
        case 0:
            if (data != 0 && data != 1)
                LOGMASKED(LOG_DISK_JR, "%06X:disk_asic_jr_w(%X) = %X (unexpected)\n", cpu_get_pc(state->maincpu), offset, data);
            break;

        case 1:
            if (data != 1 && data != 3)
                LOGMASKED(LOG_DISK_JR, "%06X:disk_asic_jr_w(%X) = %X (unexpected)\n", cpu_get_pc(state->maincpu), offset, data);
            break;

        case 2:
        case 3:
        case 7:
            break;

        case 4:
            if (data != 2)
                LOGMASKED(LOG_DISK_JR, "%06X:disk_asic_jr_w(%X) = %X (unexpected)\n", cpu_get_pc(state->maincpu), offset, data);
            break;

        case 5: /* Selección de banco ROM en Zeus 2 */
            if (state->is_zeus2)
                memory_set_bank(space->machine, "bank1", state->disk_asic_jr[offset] & 3);
            break;

        case 8:
            if (data != 4 && data != 6)
                LOGMASKED(LOG_DISK_JR, "%06X:disk_asic_jr_w(%X) = %X (unexpected)\n", cpu_get_pc(state->maincpu), offset, data);
            break;

        case 9:
            if (data != 1)
                LOGMASKED(LOG_DISK_JR, "%06X:disk_asic_jr_w(%X) = %X (unexpected)\n", cpu_get_pc(state->maincpu), offset, data);
            break;

        default:
            LOGMASKED(LOG_DISK_JR, "%06X:disk_asic_jr_w(%X) = %X\n", cpu_get_pc(state->maincpu), offset, data);
            break;
    }
}


/*************************************
 *  Control LEDs 7 segmentos (Cruis'n)
 *************************************/

static READ32_HANDLER(crusnexo_leds_r)
{
    return ~0;
}

static WRITE32_HANDLER(crusnexo_leds_w)
{
    crusnexo_state *state = (crusnexo_state *)space->machine->driver_data;
    int bit, led;

    switch (offset)
    {
        case 0:
            break;

        case 1: /* Control de lámparas */
            for (bit = 0; bit < 8; bit++)
                output_set_lamp_value(bit, BIT(data, bit));
            break;

        case 2: /* Estado de LEDs seleccionados */
            for (bit = 4; bit < 7; bit++)
                if ((state->crusnexo_leds_select & (1 << bit)) == 0)
                    output_set_digit_value(bit, ~data & 0xff);

            for (bit = 0; bit < 3; bit++)
                if ((state->crusnexo_leds_select & (1 << bit)) == 0)
                    for (led = 0; led < 8; led++)
                        output_set_led_value(bit * 8 + led, BIT(~data, led));
            break;

        case 3: /* Selección de banco LED */
            state->crusnexo_leds_select = data;
            break;
    }
}


/*************************************
 *  FireWire / IEEE 1394 (Zeus 2)
 *************************************/

static void midzeus2_update_firewire_irq(running_machine *machine)
{
    midzeus2_state *state = (midzeus2_state *)machine->driver_data;
    int active = (state->fw_int_enable && state->fw_int);

    LOGMASKED(LOG_FIREWIRE, "%ssserting FW IRQ\n", active ? "A" : "Dea");
    cpu_set_input_line(state->base.maincpu, TMS320C3X_IRQ3, active ? ASSERT_LINE : CLEAR_LINE);
}

static void midzeus2_firewire_irq(device_t *device, int irq_state)
{
    midzeus2_state *state = (midzeus2_state *)device->machine->driver_data;
    state->fw_int = irq_state;
    midzeus2_update_firewire_irq(device->machine);
}


/*************************************
 *  Accesos E/S TMS320C31 / C32
 *************************************/

static READ32_HANDLER(tms320c32_control_r)
{
    midzeus_state *state = (midzeus_state *)space->machine->driver_data;

    if (offset == 0x24 || offset == 0x34)
    {
        int which = (offset >> 4) & 1;
        uint32_t rate = BIT(state->tms320c32_control[offset - 4], 9) ? (60000000 / 4) : 10000000;
        attotime elapsed = timer_timeelapsed(state->timer[which]);
        return attotime_to_ticks(elapsed, rate);
    }

    if (offset != 0x64)
        LOGMASKED(LOG_TMS320C32, "%06X:tms320c32_control_r(%02X)\n", cpu_get_pc(state->maincpu), offset);

    return state->tms320c32_control[offset];
}

static WRITE32_HANDLER(tms320c32_control_w)
{
    midzeus_state *state = (midzeus_state *)space->machine->driver_data;
    COMBINE_DATA(&state->tms320c32_control[offset]);

    if (offset == 0x64)
        return;

    if (offset == 0x20 || offset == 0x30)
    {
        int which = (offset >> 4) & 1;
        if (data & 0x40)
            timer_adjust_oneshot(state->timer[which], attotime_never, 0);
    }
    else
    {
        LOGMASKED(LOG_TMS320C32, "%06X:tms320c32_control_w(%02X) = %08X\n", cpu_get_pc(state->maincpu), offset, data);
    }
}


/*************************************
 *  Controles Especiales (Joystick, Keypad, Trackball)
 *************************************/

static CUSTOM_INPUT(custom_49way_r)
{
    static const uint8_t translate49[7] = { 0x8, 0xc, 0xe, 0xf, 0x3, 0x1, 0x0 };
    uint8_t x = input_port_read(field->port->machine, "49WAYX") >> 4;
    uint8_t y = input_port_read(field->port->machine, "49WAYY") >> 4;
    return (translate49[y] << 4) | translate49[x];
}

static WRITE32_HANDLER(keypad_select_w)
{
    crusnexo_state *state = (crusnexo_state *)space->machine->driver_data;
    if (offset == 1)
        state->keypad_select = data;
}

static CUSTOM_INPUT(keypad_r)
{
    crusnexo_state *state = (crusnexo_state *)field->port->machine->driver_data;
    uint32_t bits = input_port_read(field->port->machine, "KEYPAD");
    uint8_t select = state->keypad_select;

    while ((select & 1) != 0)
    {
        select >>= 1;
        bits >>= 4;
    }
    return bits;
}

static READ32_HANDLER(grid_keypad_r)
{
    uint32_t keypad = input_port_read(space->machine, "KEYPAD");
    return (keypad >> ((offset >> 1) << 2)) & 0xf;
}

static READ32_HANDLER(trackball_r)
{
    if (offset == 0)
        return input_port_read(space->machine, "TRACKY");
    else
        return input_port_read(space->machine, "TRACKX");
}

static READ32_HANDLER(analog_r)
{
    static const char *const analog_tags[4] = { "ANALOG0", "ANALOG1", "ANALOG2", "ANALOG3" };
    if (offset < 8 || offset > 11)
        LOGMASKED(LOG_INPUT, "%06X:analog_r(%X)\n", cpu_get_pc(space->cpu), offset);

    return input_port_read(space->machine, analog_tags[offset & 3]);
}

static WRITE32_HANDLER(analog_w)
{
    /* 16 escrituras previas a una lectura */
}


/*************************************
 *  Pistola Óptica (Invasion)
 *************************************/

static void invasnab_update_gun_irq(running_machine *machine)
{
    invasnab_state *state = (invasnab_state *)machine->driver_data;
    if (state->gun_irq_state & state->gun_control & 0x03)
        cpu_set_input_line(state->base.maincpu, TMS320C3X_IRQ3, ASSERT_LINE);
    else
        cpu_set_input_line(state->base.maincpu, TMS320C3X_IRQ3, CLEAR_LINE);
}

static TIMER_CALLBACK(invasn_gun_callback)
{
    invasnab_state *state = (invasnab_state *)machine->driver_data;
    int player = param;
    int beamy = video_screen_get_vpos(state->base.screen);
    const rectangle *visarea = video_screen_get_visible_area(state->base.screen);

    state->gun_irq_state |= (0x01 << player);
    invasnab_update_gun_irq(machine);

    beamy++;
    if (beamy <= visarea->max_y && beamy <= state->gun_y[player] + BEAM_DY)
    {
        int target_x = max_int(0, state->gun_x[player] - BEAM_DX);
        timer_adjust_oneshot(state->gun_timer[player],
                             video_screen_get_time_until_pos(state->base.screen, beamy, target_x),
                             player);
    }
}

static WRITE32_HANDLER(invasn_gun_w)
{
    invasnab_state *state = (invasnab_state *)space->machine->driver_data;
    uint32_t old_control = state->gun_control;
    int player;

    COMBINE_DATA(&state->gun_control);

    state->gun_irq_state &= ~((state->gun_control >> 2) & 3);
    invasnab_update_gun_irq(space->machine);

    for (player = 0; player < 2; player++)
    {
        uint8_t pmask = 0x04 << player;
        if (((old_control ^ state->gun_control) & pmask) != 0 && (state->gun_control & pmask) == 0)
        {
            const rectangle *visarea = video_screen_get_visible_area(state->base.screen);
            int screen_w = video_screen_get_width(state->base.screen);
            int vis_w = visarea->max_x - visarea->min_x + 1;
            int vis_h = visarea->max_y - visarea->min_y + 1;
            const char *gun_x_tag = (player == 0) ? "GUNX1" : "GUNX2";
            const char *gun_y_tag = (player == 0) ? "GUNY1" : "GUNY2";

            state->gun_x[player] = (input_port_read(space->machine, gun_x_tag) * vis_w / 0xff + visarea->min_x + BEAM_XOFFS + BEAM_DX * 2) % screen_w;
            state->gun_x[player] = clamp_int(state->gun_x[player], BEAM_DX, screen_w - BEAM_DX);
            state->gun_y[player] = input_port_read(space->machine, gun_y_tag) * vis_h / 0xff + visarea->min_y + BEAM_DY * 2;

            timer_adjust_oneshot(state->gun_timer[player],
                                 video_screen_get_time_until_pos(state->base.screen,
                                                                 max_int(0, state->gun_y[player] - BEAM_DY),
                                                                 max_int(0, state->gun_x[player] - BEAM_DX)),
                                 player);
        }
    }
}

static READ32_HANDLER(invasn_gun_r)
{
    invasnab_state *state = (invasnab_state *)space->machine->driver_data;
    uint32_t result = 0xffff;
    int beamx = video_screen_get_hpos(state->base.screen);
    int beamy = video_screen_get_vpos(state->base.screen);
    int player;

    result ^= input_port_read(space->machine, "TRIGGER") << 8;

    for (player = 0; player < 2; player++)
    {
        const char *gun_x_tag = (player == 0) ? "GUNX1" : "GUNX2";
        int io_gun_x = input_port_read(space->machine, gun_x_tag);

        if (io_gun_x > 0 && io_gun_x < 0xff)
        {
            int diffx = beamx - state->gun_x[player];
            int diffy = beamy - state->gun_y[player];

            if (diffx >= -BEAM_DX && diffx <= BEAM_DX && diffy >= -BEAM_DY && diffy <= BEAM_DY)
                result ^= (0x1000 << player);
        }
    }
    return result;
}


/*************************************
 *  Mapas de Memoria (Address Maps)
 *************************************/

static ADDRESS_MAP_START(zeus_map, ADDRESS_SPACE_PROGRAM, 32)
    AM_RANGE(0x000000, 0x03ffff) AM_RAM AM_BASE_MEMBER(midzeus_state, ram_base)
    AM_RANGE(0x400000, 0x41ffff) AM_RAM
    AM_RANGE(0x808000, 0x80807f) AM_READWRITE(tms320c32_control_r, tms320c32_control_w) AM_BASE_MEMBER(midzeus_state, tms320c32_control)
    AM_RANGE(0x880000, 0x8803ff) AM_READWRITE(zeus_r, zeus_w) AM_BASE_MEMBER(midzeus_state, zeusbase)
    AM_RANGE(0x8d0000, 0x8d0009) AM_READWRITE(disk_asic_jr_r, disk_asic_jr_w)
    AM_RANGE(0x990000, 0x99000f) AM_DEVREADWRITE("ioasic", midway_ioasic_r, midway_ioasic_w)
    AM_RANGE(0x9e0000, 0x9e0000) AM_NOP
    AM_RANGE(0x9f0000, 0x9f7fff) AM_READWRITE(cmos_r, cmos_w) AM_BASE_MEMBER(midzeus_state, nvram)
    AM_RANGE(0x9f8000, 0x9f8000) AM_WRITE(cmos_protect_w)
    AM_RANGE(0xa00000, 0xffffff) AM_ROM AM_REGION("maindata", 0)
ADDRESS_MAP_END

static ADDRESS_MAP_START(invasnab_map, ADDRESS_SPACE_PROGRAM, 32)
    AM_IMPORT_FROM(zeus_map)
    AM_RANGE(0x9c0000, 0x9c0000) AM_READWRITE(invasn_gun_r, invasn_gun_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START(zeus2_map, ADDRESS_SPACE_PROGRAM, 32)
    AM_RANGE(0x000000, 0x03ffff) AM_RAM AM_BASE_MEMBER(midzeus_state, ram_base)
    AM_RANGE(0x400000, 0x43ffff) AM_RAM
    AM_RANGE(0x808000, 0x80807f) AM_READWRITE(tms320c32_control_r, tms320c32_control_w) AM_BASE_MEMBER(midzeus_state, tms320c32_control)
    AM_RANGE(0x880000, 0x88007f) AM_DEVREADWRITE("zeus2", zeus2_r, zeus2_w)
    AM_RANGE(0x8a0000, 0x8a00cf) AM_DEVREADWRITE("fw_link", tsb12lv01a_r, tsb12lv01a_w)
    AM_RANGE(0x8d0000, 0x8d0009) AM_READWRITE(disk_asic_jr_r, disk_asic_jr_w)
    AM_RANGE(0x900000, 0x91ffff) AM_READWRITE(zpram_r, zpram_w) AM_BASE_MEMBER(midzeus_state, nvram) AM_MIRROR(0x020000)
    AM_RANGE(0x990000, 0x99000f) AM_DEVREADWRITE("ioasic", midway_ioasic_r, midway_ioasic_w)
    AM_RANGE(0x9d0000, 0x9d000f) AM_READWRITE(disk_asic_r, disk_asic_w)
    AM_RANGE(0x9e0000, 0x9e0000) AM_NOP
    AM_RANGE(0x9f0000, 0x9f7fff) AM_READWRITE(timekeeper_r, timekeeper_w)
    AM_RANGE(0x9f8000, 0x9f8000) AM_WRITE(cmos_protect_w)
    AM_RANGE(0xa00000, 0xbfffff) AM_ROM AM_REGION("maindata", 0)
    AM_RANGE(0xc00000, 0xffffff) AM_ROMBANK("bank1")
ADDRESS_MAP_END

static ADDRESS_MAP_START(crusnexo_map, ADDRESS_SPACE_PROGRAM, 32)
    AM_IMPORT_FROM(zeus2_map)
    AM_RANGE(0x8d0009, 0x8d000a) AM_WRITE(keypad_select_w)
    AM_RANGE(0x9b0004, 0x9b0007) AM_READWRITE(crusnexo_leds_r, crusnexo_leds_w)
    AM_RANGE(0x9c0000, 0x9c000f) AM_READWRITE(analog_r, analog_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START(thegrid_map, ADDRESS_SPACE_PROGRAM, 32)
    AM_IMPORT_FROM(zeus2_map)
    AM_RANGE(0x8c0000, 0x8c0001) AM_READ(trackball_r)
    AM_RANGE(0x9b0000, 0x9b0004) AM_READ(grid_keypad_r)
ADDRESS_MAP_END


/*************************************
 *  Machine Drivers (Configuración)
 *************************************/

static MACHINE_DRIVER_START(midzeus)
    MDRV_DRIVER_DATA(midzeus_state)

    MDRV_CPU_ADD("maincpu", TMS320C32, 60000000)
    MDRV_CPU_PROGRAM_MAP(zeus_map)

    MDRV_NVRAM_ADD_1FILL("nvram")

    MDRV_SCREEN_ADD("screen", RASTER)
    MDRV_SCREEN_FORMAT(BITMAP_FORMAT_RGB15)
    MDRV_SCREEN_RAW_PARAMS(MIDZEUS_VIDEO_CLOCK / 8, 529, 0, 400, 278, 0, 256)
    MDRV_SCREEN_UPDATE(midzeus)
    MDRV_PALETTE_LENGTH(32768)

    MDRV_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")
    MDRV_IMPORT_FROM(dcs2_audio_2104)

    MDRV_MACHINE_START(midzeus)
    MDRV_MACHINE_RESET(midzeus)
MACHINE_DRIVER_END

static MACHINE_DRIVER_START(mk4)
    MDRV_IMPORT_FROM(midzeus)
MACHINE_DRIVER_END

static MACHINE_DRIVER_START(invasn)
    MDRV_IMPORT_FROM(midzeus)
    MDRV_DRIVER_DATA(invasnab_state)
    MDRV_CPU_MODIFY("maincpu")
    MDRV_CPU_PROGRAM_MAP(invasnab_map)
    MDRV_CPU_ADD("pic", PIC16C57, 8000000)
    MDRV_MACHINE_START(invasnab)
MACHINE_DRIVER_END

static MACHINE_DRIVER_START(midzeus2)
    MDRV_DRIVER_DATA(midzeus2_state)

    MDRV_CPU_ADD("maincpu", TMS320C32, 60000000)
    MDRV_CPU_PROGRAM_MAP(zeus2_map)

    MDRV_NVRAM_ADD_1FILL("nvram")

    MDRV_SCREEN_ADD("screen", RASTER)
    MDRV_SCREEN_FORMAT(BITMAP_FORMAT_RGB32)
    MDRV_SCREEN_RAW_PARAMS(ZEUS2_VIDEO_CLOCK / 4, 666, 0, 512, 438, 0, 400)
    MDRV_SCREEN_UPDATE(zeus2)

    MDRV_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")
    MDRV_IMPORT_FROM(dcs2_audio_2104)

    MDRV_M48T35_ADD("m48t35", TIMEKEEPER_DEFAULT_YEAR_1999)

    MDRV_MACHINE_START(midzeus2)
    MDRV_MACHINE_RESET(midzeus2)
MACHINE_DRIVER_END

static MACHINE_DRIVER_START(crusnexo)
    MDRV_IMPORT_FROM(midzeus2)
    MDRV_DRIVER_DATA(crusnexo_state)
    MDRV_CPU_MODIFY("maincpu")
    MDRV_CPU_PROGRAM_MAP(crusnexo_map)
    MDRV_MACHINE_START(crusnexo)
MACHINE_DRIVER_END

static MACHINE_DRIVER_START(thegrid)
    MDRV_IMPORT_FROM(midzeus2)
    MDRV_DRIVER_DATA(thegrid_state)
    MDRV_CPU_MODIFY("maincpu")
    MDRV_CPU_PROGRAM_MAP(thegrid_map)
    MDRV_CPU_ADD("pic", PIC16C57, 8000000)
    MDRV_DEVICE_DISABLE()
MACHINE_DRIVER_END

/*
 * Nota: Las secciones INPUT_PORTS_START, ROM_START y GAME() son macros declarativas
 * basadas en tablas estáticas, por lo que permanecen sintácticamente compatibles en C.
 */
