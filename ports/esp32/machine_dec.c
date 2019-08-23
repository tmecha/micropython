/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
  * Copyright (c) 2018 B. Boser (https://github.com/loboris)
 *
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

#include <stdio.h>
#include "driver/pcnt.h"
#include "esp_err.h"

#include "py/nlr.h"
#include "py/runtime.h"
#include "modmachine.h"
#include "mphalport.h"

pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

/* Decoder structure
 * 
 */
typedef struct _esp32_dec_obj_t {
    mp_obj_base_t base;
    pcnt_config_t chan_a;
    pcnt_config_t chan_b;
} esp32_dec_obj_t;


/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}


//-------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t esp32_dec_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 2, 3, true);
    int unit = mp_obj_get_int(args[0]);
    gpio_num_t pin_a = machine_pin_get_id(args[1]);
    gpio_num_t pin_b = PCNT_PIN_NOT_USED;
    if (n_args == 3) pin_b = machine_pin_get_id(args[2]);

    if (unit < 0 || unit > PCNT_UNIT_MAX)
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Bad timer number %d", unit));

    // create dec object for the given unit
    esp32_dec_obj_t *self = m_new_obj(esp32_dec_obj_t);
    self->base.type = &machine_dec_type;

    // configure timer channel 0
    self->chan_a.channel = PCNT_CHANNEL_0;
    self->chan_a.pulse_gpio_num = pin_a;    // reverse from channel 1
    self->chan_a.ctrl_gpio_num = pin_b;
    self->chan_a.unit = unit;
    self->chan_a.pos_mode = PCNT_COUNT_DEC;
    self->chan_a.neg_mode = PCNT_COUNT_INC;
    self->chan_a.lctrl_mode = PCNT_MODE_KEEP;
    self->chan_a.hctrl_mode = PCNT_MODE_REVERSE;
    self->chan_a.counter_h_lim =  INT16_MAX;    // don't care if interrupt is not used?
    self->chan_a.counter_l_lim = -INT16_MAX;

    // configure timer channel 1
    self->chan_b.channel = PCNT_CHANNEL_1;
    self->chan_b.pulse_gpio_num = pin_b;    // reverse from channel 0
    self->chan_b.ctrl_gpio_num = pin_a;
    self->chan_b.unit = unit;
    self->chan_b.pos_mode = PCNT_COUNT_DEC;
    self->chan_b.neg_mode = PCNT_COUNT_INC;
    self->chan_b.lctrl_mode = PCNT_MODE_REVERSE;
    self->chan_b.hctrl_mode = PCNT_MODE_KEEP;
    self->chan_b.counter_h_lim =  INT16_MAX;    // don't care if interrupt is not used?
    self->chan_b.counter_l_lim = -INT16_MAX;

    if (n_args == 2) {
        // not sure if all this is required, but I've played long enough
        self->chan_a.pos_mode = PCNT_COUNT_INC;
        self->chan_a.hctrl_mode = PCNT_MODE_KEEP;

        self->chan_b.channel = PCNT_CHANNEL_1;
        self->chan_b.ctrl_gpio_num = pin_b;
        self->chan_b.pos_mode = PCNT_COUNT_DIS;
        self->chan_b.neg_mode = PCNT_COUNT_DIS;
        self->chan_b.hctrl_mode = PCNT_MODE_KEEP;
    }

    pcnt_unit_config(&(self->chan_a));
    pcnt_unit_config(&(self->chan_b));
    pcnt_filter_disable(unit);

    // init / start the counter
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);

    // not sure what this is for or if it's needed
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);

    return MP_OBJ_FROM_PTR(self);
}

//------------------------------------------------------------------------------------------
STATIC void esp32_dec_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    esp32_dec_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "DEC(%u, Pin(%u)", self->chan_a.unit, self->chan_a.pulse_gpio_num);
    if (self->chan_a.ctrl_gpio_num != PCNT_PIN_NOT_USED) mp_printf(print, ", Pin(%u)", self->chan_a.ctrl_gpio_num);
    mp_printf(print, ")");
}

//-----------------------------------------------------------------
STATIC mp_obj_t esp32_dec_count(mp_obj_t self_in)
{
    esp32_dec_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int16_t count;
    pcnt_get_counter_value(self->chan_a.unit, &count);
    return MP_OBJ_NEW_SMALL_INT(count);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp32_dec_count_obj, esp32_dec_count);

//-----------------------------------------------------------------
STATIC mp_obj_t esp32_dec_count_and_clear(mp_obj_t self_in)
{
    esp32_dec_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int16_t count;
    pcnt_get_counter_value(self->chan_a.unit, &count);
    pcnt_counter_clear(self->chan_a.unit);
    return MP_OBJ_NEW_SMALL_INT(count);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp32_dec_count_and_clear_obj, esp32_dec_count_and_clear);

//-----------------------------------------------------------------
STATIC mp_obj_t esp32_dec_clear(mp_obj_t self_in)
{
    esp32_dec_obj_t *self = MP_OBJ_TO_PTR(self_in);
    pcnt_counter_clear(self->chan_a.unit);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp32_dec_clear_obj, esp32_dec_clear);

//-----------------------------------------------------------------
STATIC mp_obj_t esp32_dec_pause(mp_obj_t self_in)
{
    esp32_dec_obj_t *self = MP_OBJ_TO_PTR(self_in);
    pcnt_counter_pause(self->chan_a.unit);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp32_dec_pause_obj, esp32_dec_pause);

//-----------------------------------------------------------------
STATIC mp_obj_t esp32_dec_resume(mp_obj_t self_in)
{
    esp32_dec_obj_t *self = MP_OBJ_TO_PTR(self_in);
    pcnt_counter_resume(self->chan_a.unit);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp32_dec_resume_obj, esp32_dec_resume);

//-----------------------------------------------------------------
// dec.irq(handler=None, trigger=IRQ_FALLING|IRQ_RISING)
STATIC mp_obj_t machine_dec_irq(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_handler, ARG_trigger, ARG_wake };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_handler, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_trigger, MP_ARG_INT, {.u_int = PCNT_EVT_L_LIM | PCNT_EVT_H_LIM | PCNT_EVT_THRES_0 | PCNT_EVT_THRES_1 | PCNT_EVT_ZERO} },
//      { MP_QSTR_wake, MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    machine_dec_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (n_args > 1 || kw_args->used != 0) {
        // configure irq
        mp_obj_t handler = args[ARG_handler].u_obj;
        uint32_t trigger = args[ARG_trigger].u_int;
//		mp_obj_t wake_obj = args[ARG_wake].u_obj;

		/*
        if ( wake_obj != mp_const_none) {
            mp_int_t wake;
            if (mp_obj_get_int_maybe(wake_obj, &wake)) {
                if (wake < 2 || wake > 7) {
                    mp_raise_ValueError("bad wake value");
                }
            } else {
                mp_raise_ValueError("bad wake value");
            }

            if (machine_rtc_config.wake_on_touch) { // not compatible
                mp_raise_ValueError("no resources");
            }

            if (!RTC_IS_VALID_EXT_PIN(self->id)) {
                mp_raise_ValueError("invalid pin for wake");
            }

        } 
		*/

		/* Set threshold 0 and 1 values and enable events to watch */
		/*
		pcnt_set_event_value(self->chan_a.unit, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
		pcnt_event_enable(self->chan_a.unit, PCNT_EVT_THRES_1);
		pcnt_set_event_value(self->chan_a.unit, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
		pcnt_event_enable(self->chan_a.unit, PCNT_EVT_THRES_0);
		*/
		/* Enable events on zero, maximum and minimum limit values */
		pcnt_event_enable(self->chan_a.unit, PCNT_EVT_H_LIM);
		pcnt_event_enable(self->chan_a.unit, PCNT_EVT_L_LIM);
		pcnt_event_enable(self->chan_a.unit, PCNT_EVT_ZERO);
		
		if (handler == mp_const_none) {
			handler = MP_OBJ_NULL;
			trigger = 0;
		}
		
		/* // This is how GPIO module does it.
		gpio_isr_handler_remove(self->id);
		MP_STATE_PORT(machine_pin_irq_handler)[self->id] = handler;
		gpio_set_intr_type(self->id, trigger);
		gpio_isr_handler_add(self->id, machine_pin_isr_handler, (void*)self);
		*/
		
		/* Register ISR handler and enable interrupts for PCNT unit */
		pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle); 
		//todo: Need to assign different handler for each different unit. 
			    // GPIO does this through some global variable.
		pcnt_intr_enable(self->chan_a.unit);
    }

    // return the irq object
    return MP_OBJ_FROM_PTR(&machine_pin_irq_object[self->id]);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_pin_irq_obj, 1, machine_pin_irq);

//==============================================================
STATIC const mp_rom_map_elem_t esp32_dec_locals_dict_table[] = {
    // instance methods
	{ MP_ROM_QSTR(MP_QSTR_count), MP_ROM_PTR(&esp32_dec_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_and_clear), MP_ROM_PTR(&esp32_dec_count_and_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&esp32_dec_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_pause), MP_ROM_PTR(&esp32_dec_pause_obj) },
    { MP_ROM_QSTR(MP_QSTR_resume), MP_ROM_PTR(&esp32_dec_resume_obj) },
	{ MP_ROM_QSTR(MP_QSTR_irq), MP_ROM_PTR(&machine_dec_irq_obj) },
	
	//class constants
	{ MP_ROM_QSTR(MP_QSTR_EVT_L_LIM), MP_ROM_INT(PCNT_EVT_L_LIM) },
	{ MP_ROM_QSTR(MP_QSTR_EVT_H_LIM), MP_ROM_INT(PCNT_EVT_H_LIM) },
	{ MP_ROM_QSTR(MP_QSTR_EVT_THRES_0), MP_ROM_INT(PCNT_EVT_THRES_0) },
	{ MP_ROM_QSTR(MP_QSTR_EVT_THRES_1), MP_ROM_INT(PCNT_EVT_THRES_1) },
	{ MP_ROM_QSTR(MP_QSTR_EVT_ZERO), MP_ROM_INT(PCNT_EVT_ZERO) },
	
};
STATIC MP_DEFINE_CONST_DICT(esp32_dec_locals_dict, esp32_dec_locals_dict_table);

//======================================
const mp_obj_type_t machine_dec_type = {
    { &mp_type_type },
    .name = MP_QSTR_DEC,
    .print = esp32_dec_print,
    .make_new = esp32_dec_make_new,
    .locals_dict = (mp_obj_dict_t*)&esp32_dec_locals_dict,
};