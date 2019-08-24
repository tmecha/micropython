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
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/pcnt.h"
#include "esp_err.h"

#include "py/nlr.h"
#include "py/runtime.h"
#include "modmachine.h"
#include "mphalport.h"

//pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle // got rid when switched to independent timer handlers

/* Counter structure
 * 
 */
typedef struct _esp32_dec_obj_t {
    mp_obj_base_t base;
    pcnt_unit_t unit;
    pcnt_config_t chan_0;
    pcnt_config_t chan_1;
    mp_obj_t handler;
    xQueueHandle event_queue;   //A queue to handle pulse counter interrupt events
} esp32_dec_obj_t;

/* A structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    // int unit;  // the PCNT unit that originated an interrupt
    uint32_t event; // information on the event type that caused the interrupt
    int16_t count;  // count captured in interrupt when event happened
} pcnt_evt_t;

//Forward declaration
STATIC const mp_obj_type_t mod_irq_event_Event_type;

/* A structure to return Python event objects
 *
 */
 /*
typedef struct {
	mp_obj_base_t base;
    uint32_t event; // information on the event type that caused the interrupt
    int16_t count;  // count captured in interrupt when event happened
} pcnt_evt_obj_t;
*/

// class Event(object):
typedef struct _mp_obj_Event_t {
    mp_obj_base_t base;
	uint32_t event; // information on the event type that caused the interrupt
    int16_t count;  // count captured in interrupt when event happened
} mp_obj_Event_t;

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR machine_cnt_isr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    esp32_dec_obj_t *self = arg;
    pcnt_evt_t event;

	if (intr_status & (BIT(self->unit))) {
		//event.unit = self->unit;
		/* Save the PCNT event type that caused an interrupt
		   to pass it to the main program */
		event.event = PCNT.status_unit[self->unit].val;
		pcnt_get_counter_value(self->unit, &(event.count));

		PCNT.int_clr.val = BIT(self->unit);

		xQueueSendFromISR(self->event_queue, &event, pdFALSE);
	}

    mp_sched_schedule(self->handler, MP_OBJ_FROM_PTR(self));
    mp_hal_wake_main_task_from_isr();

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
    self->unit = unit;

    //Set interrupt event queue to null for now
    self->event_queue = MP_OBJ_NULL;

    // configure timer channel 0
    self->chan_0.channel = PCNT_CHANNEL_0;
    self->chan_0.pulse_gpio_num = pin_a;    // reverse from channel 1
    self->chan_0.ctrl_gpio_num = pin_b;
    self->chan_0.unit = unit;
    self->chan_0.pos_mode = PCNT_COUNT_DEC;
    self->chan_0.neg_mode = PCNT_COUNT_INC;
    self->chan_0.lctrl_mode = PCNT_MODE_KEEP;
    self->chan_0.hctrl_mode = PCNT_MODE_REVERSE;
    self->chan_0.counter_h_lim =  4000; //INT16_MAX;    // don't care if interrupt is not used?
    self->chan_0.counter_l_lim =  INT16_MIN;

    // configure timer channel 1
    self->chan_1.channel = PCNT_CHANNEL_1;
    self->chan_1.pulse_gpio_num = pin_b;    // reverse from channel 0
    self->chan_1.ctrl_gpio_num = pin_a;
    self->chan_1.unit = unit;
    self->chan_1.pos_mode = PCNT_COUNT_DEC;
    self->chan_1.neg_mode = PCNT_COUNT_INC;
    self->chan_1.lctrl_mode = PCNT_MODE_REVERSE;
    self->chan_1.hctrl_mode = PCNT_MODE_KEEP;
    self->chan_1.counter_h_lim =  INT16_MAX;    // don't care if interrupt is not used?
    self->chan_1.counter_l_lim =  INT16_MIN;

    if (n_args == 2) {
        // not sure if all this is required, but I've played long enough
        self->chan_0.pos_mode = PCNT_COUNT_INC;
        self->chan_0.hctrl_mode = PCNT_MODE_KEEP;

        self->chan_1.channel = PCNT_CHANNEL_1;
        self->chan_1.ctrl_gpio_num = pin_b;
        self->chan_1.pos_mode = PCNT_COUNT_DIS;
        self->chan_1.neg_mode = PCNT_COUNT_DIS;
        self->chan_1.hctrl_mode = PCNT_MODE_KEEP;
    }

    pcnt_unit_config(&(self->chan_0));
    pcnt_unit_config(&(self->chan_1));
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
    mp_printf(print, "DEC(Unit(%u), Pin(%u)", self->unit, self->chan_0.pulse_gpio_num);
    if (self->chan_0.ctrl_gpio_num != PCNT_PIN_NOT_USED) mp_printf(print, ", Ctrl Pin(%u)", self->chan_0.ctrl_gpio_num);
    mp_printf(print, ")");
}

//-----------------------------------------------------------------
STATIC mp_obj_t esp32_dec_count(mp_obj_t self_in)
{
    esp32_dec_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int16_t count;
    pcnt_get_counter_value(self->chan_0.unit, &count);
    return MP_OBJ_NEW_SMALL_INT(count);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp32_dec_count_obj, esp32_dec_count);

//-----------------------------------------------------------------
STATIC mp_obj_t esp32_dec_count_and_clear(mp_obj_t self_in)
{
    esp32_dec_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int16_t count;
    pcnt_get_counter_value(self->chan_0.unit, &count);
    pcnt_counter_clear(self->chan_0.unit);
    return MP_OBJ_NEW_SMALL_INT(count);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp32_dec_count_and_clear_obj, esp32_dec_count_and_clear);

//-----------------------------------------------------------------
STATIC mp_obj_t esp32_dec_clear(mp_obj_t self_in)
{
    esp32_dec_obj_t *self = MP_OBJ_TO_PTR(self_in);
    pcnt_counter_clear(self->chan_0.unit);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp32_dec_clear_obj, esp32_dec_clear);

//-----------------------------------------------------------------
STATIC mp_obj_t esp32_dec_pause(mp_obj_t self_in)
{
    esp32_dec_obj_t *self = MP_OBJ_TO_PTR(self_in);
    pcnt_counter_pause(self->chan_0.unit);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp32_dec_pause_obj, esp32_dec_pause);

//-----------------------------------------------------------------
STATIC mp_obj_t esp32_dec_resume(mp_obj_t self_in)
{
    esp32_dec_obj_t *self = MP_OBJ_TO_PTR(self_in);
    pcnt_counter_resume(self->chan_0.unit);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp32_dec_resume_obj, esp32_dec_resume);

//-----------------------------------------------------------------
STATIC mp_obj_t esp32_dec_get_irq_event(mp_obj_t self_in)
{
	esp32_dec_obj_t *self = MP_OBJ_TO_PTR(self_in);
	portBASE_TYPE res;
	pcnt_evt_t evt;
	//pcnt_evt_obj_t *event_obj =  pvPortMalloc(sizeof(pcnt_evt_obj_t));

	mp_obj_Event_t *event_obj = m_new_obj(mp_obj_Event_t);
    event_obj->base.type = mod_irq_event_Event_type;
	
    /* Wait for the event information passed from PCNT's interrupt handler.
     * Once received, decode the event type and print it on the serial monitor.
     */
    res = xQueueReceive(self->event_queue, &evt, 0);
    if (res == pdTRUE) {
		event_obj->event = evt.event;
		event_obj->count = evt.count;
    	/*
        if (evt.event & PCNT_STATUS_THRES1_M) {
            printf("THRES1 EVT\n");
        }
        if (evt.event & PCNT_STATUS_THRES0_M) {
            printf("THRES0 EVT\n");
        }
        if (evt.event & PCNT_STATUS_L_LIM_M) {
            printf("L_LIM EVT\n");
        }
        if (evt.event & PCNT_STATUS_H_LIM_M) {
            printf("H_LIM EVT\n");
        }
        if (evt.event & PCNT_STATUS_ZERO_M) {
            printf("ZERO EVT\n");
        }
        */
    } else {
    	//No value in queue
    	return mp_const_none;
    }
    //return mp_const_none;
    return MP_OBJ_FROM_PTR(event_obj); //todo: do we need to free this later? Garbage collect?
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp32_dec_get_irq_event_obj, esp32_dec_get_irq_event);

//-----------------------------------------------------------------
// dec.irq(handler=None, trigger=IRQ_FALLING|IRQ_RISING)
STATIC mp_obj_t machine_dec_irq(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_handler, ARG_trigger, ARG_wake };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_handler, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_trigger, MP_ARG_INT, {.u_int = PCNT_EVT_L_LIM | PCNT_EVT_H_LIM | PCNT_EVT_THRES_0 | PCNT_EVT_THRES_1 | PCNT_EVT_ZERO} },
//      { MP_QSTR_wake, MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    esp32_dec_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    static bool did_install_isr = false;
    if (!did_install_isr) {
    	pcnt_isr_service_install(0);
        did_install_isr = true;
    }

    if (self->event_queue == MP_OBJ_NULL)
    {
        /* Initialize PCNT event queue */
        self->event_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    }
    else
    {
    	//todo: reset and clear event queue
    }

    if (n_args > 1 || kw_args->used != 0) {
        // configure irq
        self->handler = args[ARG_handler].u_obj;
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
		pcnt_set_event_value(self->unit, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
		pcnt_event_enable(self->unit, PCNT_EVT_THRES_1);
		pcnt_set_event_value(self->unit, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
		pcnt_event_enable(self->unit, PCNT_EVT_THRES_0);
		*/
		/* Enable events on zero, maximum and minimum limit values */
		pcnt_event_enable(self->unit, PCNT_EVT_H_LIM);
		pcnt_event_enable(self->unit, PCNT_EVT_L_LIM);
		pcnt_event_enable(self->unit, PCNT_EVT_ZERO);
		
		if (self->handler == mp_const_none) {
			self->handler = MP_OBJ_NULL;
			trigger = 0;
		}
		
		/* // This is how GPIO module does it.
		gpio_isr_handler_remove(self->id);
		MP_STATE_PORT(machine_pin_irq_handler)[self->id] = handler;
		gpio_set_intr_type(self->id, trigger);
		gpio_isr_handler_add(self->id, machine_pin_isr_handler, (void*)self);
		*/
		
		/* Register ISR handler and enable interrupts for PCNT unit */
		//pcnt_isr_register(machine_cnt_isr_handler, NULL, 0, &user_isr_handle);
		//todo: Need to assign different handler for each different unit. 
		pcnt_isr_handler_add(self->unit, machine_cnt_isr_handler, (void*)self);
		pcnt_intr_enable(self->unit);
    }

    // return the irq object
    // return MP_OBJ_FROM_PTR(&machine_dec_irq_object[self->id]);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_dec_irq_obj, 1, machine_dec_irq);

//=====Event Object=========================================================

// def Event.__init__(self, event, count)
STATIC mp_obj_t mod_irq_event_Event_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 2, 2, false);
    mp_obj_Event_t *self = m_new_obj(mp_obj_Event_t);
    self->base.type = &mod_irq_event_Event_type;
	self->event = mp_obj_get_int(args[0]);
	self->count = mp_obj_get_int(args[1]);
    return MP_OBJ_FROM_PTR(self);
}

// def Event.get_event(self) -> int
STATIC mp_obj_t mod_irq_event_Event_get_event(mp_obj_t self) {
    return mp_obj_new_int(self->event));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_irq_event_Event_get_event_obj, mod_irq_event_Event_get_event);

// def Event.get_event_count(self) -> int
STATIC mp_obj_t mod_irq_event_Event_get_event_count(mp_obj_t self) {
    return mp_obj_new_int(self->count));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_irq_event_Event_get_event_count_obj, mod_irq_event_Event_get_event_count);

// Event stuff

STATIC const mp_rom_map_elem_t mod_irq_event_Event_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_get_event), MP_ROM_PTR(&mod_irq_event_Event_get_event_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_event_count), MP_ROM_PTR(&mod_irq_event_Event_get_event_count_obj) },
};
STATIC MP_DEFINE_CONST_DICT(mod_irq_event_Event_locals_dict, mod_irq_event_Event_locals_dict_table);

STATIC const mp_obj_type_t mod_irq_event_Event_type = {
    { &mp_type_type },
    .name = MP_QSTR_Event,
    .make_new = mod_irq_event_Event_make_new,
    .locals_dict = (void*)&mod_irq_event_Event_locals_dict,
};

//=====MODULE=========================================================
/*
STATIC const mp_rom_map_elem_t esp32_dec_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_DEC) },
    { MP_ROM_QSTR(MP_QSTR_Event), MP_ROM_PTR(&mod_irq_event_Event_type) },
};
STATIC MP_DEFINE_CONST_DICT(esp32_dec_globals, esp32_dec_globals_table);
*/ 

STATIC const mp_rom_map_elem_t esp32_dec_locals_dict_table[] = {
    // instance methods
	{ MP_ROM_QSTR(MP_QSTR_count), MP_ROM_PTR(&esp32_dec_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_and_clear), MP_ROM_PTR(&esp32_dec_count_and_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear), MP_ROM_PTR(&esp32_dec_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_pause), MP_ROM_PTR(&esp32_dec_pause_obj) },
    { MP_ROM_QSTR(MP_QSTR_resume), MP_ROM_PTR(&esp32_dec_resume_obj) },
	{ MP_ROM_QSTR(MP_QSTR_irq), MP_ROM_PTR(&machine_dec_irq_obj) },
	{ MP_ROM_QSTR(MP_QSTR_get_irq_event), MP_ROM_PTR(&esp32_dec_get_irq_event_obj) },
	
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
	//.globals = (mp_obj_dict_t*)&esp32_dec_globals,
};
