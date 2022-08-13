/*
 * {{{

 }}}
Copyright 2022 aki27

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include QMK_KEYBOARD_H
#include <stdio.h>
#include "quantum.h"


// Defines names for use in layer keycodes and the keymap
enum layer_number {
    _BASE = 0,
    _LOWER = 1,
    _RAISE = 2,
    _TRACKBALL = 3
};


enum custom_keycodes {
    CK_PRN = COCOT_SAFE_RANGE,
    CK_BRC,
    CK_CLN
};


#define LW_SPC  LT(_LOWER, KC_SPC)
#define RS_ENT  LT(_RAISE, KC_ENT)
#define LW_LN2  LT(_LOWER, KC_LANG2)
#define RS_LN1  LT(_RAISE, KC_LANG1)
#define CT_TAB  LCTL_T(KC_TAB)
#define G_UP    G(KC_UP)
#define G_DOWN  G(KC_DOWN)
#define G_RGHT  G(KC_RGHT)
#define G_LEFT  G(KC_LEFT)
#define G_TAB   G(KC_TAB)
#define CG_RGHT C(G(KC_RGHT))
#define CG_LEFT C(G(KC_LEFT))
#define CG_D    C(G(KC_D))
#define CG_F4   C(G(KC_F4))


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [_BASE] = LAYOUT(
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      KC_GESC,    KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,                                          KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,  CK_PRN,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
       CT_TAB,    KC_A,    KC_S,    KC_D,    KC_F,    KC_G,                                          KC_H,    KC_J,    KC_K,    KC_L,  CK_CLN,  CK_BRC,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      KC_LSFT,    KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,                                          KC_N,    KC_M, KC_COMM,  KC_DOT, KC_SLSH, KC_QUOT,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
                        KC_TILD,KC_LANG1,  LW_SPC,  KC_DEL,   KC_MS_BTN3,             KC_MS_BTN2, KC_BSPC,  RS_ENT,KC_LANG2, KC_LGUI,
                                                                 KC_PGUP, KC_MS_BTN1,  KC_PGDOWN, XXXXXXX, XXXXXXX, XXXXXXX
                                                            //`--------------'  `--------------'
    ),
  [_LOWER] = LAYOUT(
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      _______, KC_EXLM,   KC_AT, KC_HASH,  KC_DLR, KC_PERC,                                       KC_CIRC, KC_AMPR, KC_ASTR, KC_LCBR, KC_RCBR, KC_RPRN,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      _______,    KC_1,    KC_2,    KC_3,    KC_4,    KC_5,                                       KC_PLUS,  KC_EQL, KC_MINS, KC_PIPE, _______, KC_RBRC,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      _______,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,                                       KC_BSLS, KC_UNDS, _______, _______, _______, KC_DQUO,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
                        _______, _______, _______, _______,   KC_MS_BTN4,             KC_MS_BTN5, _______, TT(3), _______, _______,
                                                                 KC_PGUP,    _______,  KC_PGDOWN, XXXXXXX, XXXXXXX, XXXXXXX
                                                            //`--------------'  `--------------'
    ),
  [_RAISE] = LAYOUT(
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      _______,   KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,                                         KC_F6,   KC_F7,   KC_F8,   KC_F9,  KC_F10,  KC_F11,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      _______, XXXXXXX,  G_LEFT,    G_UP,  G_DOWN,  G_RGHT,                                       KC_LEFT, KC_DOWN,   KC_UP, KC_RGHT, _______,  KC_F12,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      _______,   CG_F4,    CG_D, CG_LEFT, CG_RGHT,   G_TAB,                                       KC_HOME, KC_PGDN, KC_PGUP,  KC_END, _______, _______,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
                        _______, _______, TT(3), _______,   KC_MS_BTN4,             KC_MS_BTN5, _______, _______, _______, _______,
                                                                 KC_PGUP,    _______,  KC_PGDOWN, XXXXXXX, XXXXXXX, XXXXXXX
                                                            //`--------------'  `--------------'
    ),
  [_TRACKBALL] = LAYOUT(
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, RGB_TOG,                                       SCRL_TO,  CPI_SW, SCRL_SW, ROT_L15, ROT_R15, XXXXXXX,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      XXXXXXX, XXXXXXX, RGB_VAI, RGB_SAI, RGB_HUI, RGB_MOD,                                       SCRL_MO, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      XXXXXXX, XXXXXXX, RGB_VAD, RGB_SAD, RGB_HUD,RGB_RMOD,                                       SCRL_IN, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
                        _______, _______, _______, _______,      _______,                _______, _______, _______, _______, _______,
                                                                 KC_PGUP,    _______,  KC_PGDOWN, XXXXXXX, XXXXXXX, XXXXXXX
                                                            //`--------------'  `--------------'
    )
};

keyevent_t encoder1_ccw = {
    .key = (keypos_t){.row = 4, .col = 2},
    .pressed = false
};

keyevent_t encoder1_cw = {
    .key = (keypos_t){.row = 4, .col = 5},
    .pressed = false
};

bool encoder_update_user(uint8_t index, bool clockwise) {
    if (index == 0) { /* First encoder */
        if (clockwise) {
            encoder1_cw.pressed = true;
            encoder1_cw.time = (timer_read() | 1);
            action_exec(encoder1_cw);
        } else {
            encoder1_ccw.pressed = true;
            encoder1_ccw.time = (timer_read() | 1);
            action_exec(encoder1_ccw);
        }
    }

    return true;
}


int hue_fst = 50;
int sat_fst = 255;
int val_fst = 168;


layer_state_t layer_state_set_user(layer_state_t state) {
    hue_fst = rgblight_get_hue();
    sat_fst = rgblight_get_sat();
    val_fst = rgblight_get_val();

    switch (get_highest_layer(state)) {
    case _LOWER:
        rgblight_sethsv_range(HSV_BLUE, 0, 2);
        cocot_set_scroll_mode(true);
        break;
    case _RAISE:
        rgblight_sethsv_range(HSV_RED, 0, 2);
        cocot_set_scroll_mode(true);
        break;
    case _TRACKBALL:
        rgblight_sethsv_range(HSV_GREEN, 0, 2);
        cocot_set_scroll_mode(false);
        break;
    default:
        // rgblight_sethsv_range( 0, 0, 0, 0, 2);
        rgblight_sethsv_range(hue_fst, sat_fst, val_fst, 0, 2);
        cocot_set_scroll_mode(false);
        break;
    }
    rgblight_set_effect_range( 2, 10);
  return state;
};


#ifdef OLED_ENABLE
bool oled_task_user(void) {
    render_logo();
    oled_write_layer_state();
    return false;
}
#endif


/* static bool lower_pressed = false; */
/* static uint16_t lower_pressed_timer; */
/* static bool raise_pressed = false; */
/* static uint16_t raise_pressed_timer; */
static bool shift_pressed = false;
static bool alt_pressed = false;
static uint16_t alt_pressed_timer;
static bool alt_active = false;

bool process_record_user(uint16_t keycode, keyrecord_t *record) {

    if (record->event.pressed && keycode != CK_BRC && alt_pressed && !alt_active) {
        alt_active = true;
        register_code(KC_LALT);
    }

    switch (keycode) {
        case KC_LSFT:
            shift_pressed = record->event.pressed;
            break;

        case CK_PRN:
            if (record->event.pressed) {
                if (shift_pressed) {
                    tap_code16(KC_RPRN);
                } else {
                    tap_code16(KC_LPRN);
                }
            }
            return false;
            break;
        case CK_BRC:
            alt_pressed = record->event.pressed;
            if (record->event.pressed) {
                alt_pressed_timer = timer_read();
            } else {
                if (alt_active) {
                    unregister_code(KC_LALT);
                    alt_active = false;
                } else if (timer_elapsed(alt_pressed_timer) <= TAPPING_TERM) {
                    if (shift_pressed) {
                        unregister_code(KC_LSFT);
                        tap_code16(KC_RBRC);
                        register_code(KC_LSFT);
                    } else {
                        tap_code16(KC_LBRC);
                    }
                }
            }
            return false;
            break;
        case CK_CLN:
            if (record->event.pressed) {
                if (shift_pressed) {
                    unregister_code(KC_LSFT);
                    tap_code16(KC_SCLN);
                    register_code(KC_LSFT);
                } else {
                    tap_code16(KC_COLN);
                }
            }
            return false;
            break;
    }
    return true;
}


void matrix_scan_user(void) {

    if (alt_pressed && !alt_active) {
        if (timer_elapsed(alt_pressed_timer) > TAPPING_TERM) {
            register_code(KC_LALT);
            alt_active = true;
        }
    }

    if (IS_PRESSED(encoder1_ccw)) {
        encoder1_ccw.pressed = false;
        encoder1_ccw.time = (timer_read() | 1);
        action_exec(encoder1_ccw);
    }

    if (IS_PRESSED(encoder1_cw)) {
        encoder1_cw.pressed = false;
        encoder1_cw.time = (timer_read() | 1);
        action_exec(encoder1_cw);
    }

}