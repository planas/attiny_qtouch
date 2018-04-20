#include <avr/io.h>

#define tim_dly      r16 // __temp_reg__
#define detect_int   r17 // __zero_reg__
#define tempBL       r18
#define tempBH       r19
#define dht_counter  r20 // dht = drift hold time
#define state        r21
#define deltaL       r22
#define deltaH       r23
#define sw_timerL    r24
#define sw_timerH    r25
#define tempAL       r26 // XL
#define tempAH       r27 // XH
#define signal_valL  r28 // YL
#define signal_valH  r29 // YH
#define address_reg0 r30 // ZL
#define address_reg1 r31 // ZH

.data
reference_address: .byte 2
signal_address:    .byte 2
state_address:     .byte 2
di_address:        .byte 2 // detect integration
delta_address:     .byte 2
counter_address:   .byte 2
reference:         .byte 2
curr_signal:       .byte 1
aks_flag:          .byte 1
di_flag:           .byte 1 // detect integration flag
dht_counter_var:   .byte 1 // drift hold time counter

.text
aks_check:
  clr   tempBL
  sts   aks_flag, tempBL
  st    -Y, dht_counter
  st    -Y, detect_int
  st    -Y, tempAL
  st    -Y, tempAH
  st    -Y, sw_timerH
  st    -Y, sw_timerL
  st    -Y, tim_dly
  ldi   address_reg0, LOW(sensor_state)
  ldi   address_reg1, HIGH(sensor_state)
  ldi   tempAL, LOW(sensor_delta)
  ldi   tempAH, HIGH(sensor_delta)
  ldi   detect_int, DEF_QT_AKS_ENABLE
  tst   detect_int
  breq  restore

aks_loop:
  ld    dht_counter, Z+
  ld    sw_timerL, X+
  ld    sw_timerH, X+
  sbrc  sw_timerH, 7
  ldi   sw_timerL, 0x00
  ldi   tim_dly, DEF_QT_AKS_ENABLE
  cp    tempBH, tempBL
  breq  end_aks_check
  cp    detect_int, tim_dly
  brne  end_aks_check
  andi  dht_counter, 0x0F
  tst   dht_counter
  breq  aks_true
  cpi   sw_timerL, 0x00
  breq  chk_for_sensor_to_supress
  rcall substract_loop_delta_by_threshold
  rcall substract_curr_delta_by_threshold

chk_for_sensor_to_supress:
  cp    deltaL, sw_timerL
  brcs  aks_true
  rjmp  end_aks_check

aks_true:
  ldi   tim_dly, 0x01
  sts   aks_flag, tim_dly
  rjmp  restore

end_aks_check:
  inc   tempBL
  clr   tim_dly
  sts   aks_flag, tim_dly
  cpi   tempBL, DEF_QT_NUM_SENSORS_SYM
  brne  aks_loop

restore:
  ld    tim_dly, Y+
  ld    sw_timerL, Y+
  ld    sw_timerH, Y+
  ld    tempAH, Y+
  ld    tempAL, Y+
  ld    detect_int, Y+
  ld    dht_counter, Y+
  ret

start_cal:
  ldi   state, 0x03
  clr   tempBL
  sts   di_flag, tempBL
  ldi   sw_timerL, 0x0F
  clr   tempBH
  clr   sw_timerH
  ldi   address_reg0, LOW(sensor_state)
  ldi   address_reg1, HIGH(sensor_state)

cal_state:
  st    Z+, state
  inc   tempBH
  cpi   tempBH, DEF_QT_NUM_SENSORS_SYM
  brne  cal_state
  clr   tempBH
  clr   state
  ldi   address_reg0, LOW(sensor_general_counter)
  ldi   address_reg1, HIGH(sensor_general_counter)

cal_counter:
  st    Z+, sw_timerL
  st    Z+, sw_timerH
  inc   tempBH
  cpi   tempBH, DEF_QT_NUM_SENSORS_SYM
  brne  cal_counter
  ret

cal_key:
  ldi   state, 0x03
  ldi   sw_timerL, 0x0F
  ret

handle_key_touch:
  clr   tempBH
  ldi   tempAH, 0x00
  ldi   tempBL, 0x01
  ldi   address_reg0, LOW(sensor_state)
  ldi   address_reg1, HIGH(sensor_state)

touch_loop:
  ld    tempAL, Z+
  tst   tempAL
  brne  time_out
  or    tempAH, tempBL

time_out:
  inc   tempBH
  add   tempBL, tempBL
  cpi   tempBH, DEF_QT_NUM_SENSORS_SYM
  brne  touch_loop

  sts   sensor_states, tempAH // saves the sensor states
  ret

.global qt_init_sensing
qt_init_sensing:
  st    -Y, dht_counter
  st    -Y, state
  st    -Y, tempBL
  st    -Y, tempBH
  st    -Y, sw_timerL
  st    -Y, sw_timerH
  st    -Y, address_reg0
  st    -Y, address_reg1
  clr   dht_counter
  sts   dht_counter_var, dht_counter
  rcall start_cal
  ld    address_reg1, Y+
  ld    address_reg0, Y+
  ld    sw_timerH, Y+
  ld    sw_timerL, Y+
  ld    tempBH, Y+
  ld    tempBL, Y+
  ld    state, Y+
  ld    dht_counter, Y+
  ret

.global qt_measure_sensors
qt_measure_sensors:
  st    -Y, tim_dly
  st    -Y, detect_int
  st    -Y, tempBL
  st    -Y, tempBH
  st    -Y, dht_counter
  st    -Y, state
  st    -Y, deltaL
  st    -Y, deltaH
  st    -Y, sw_timerL
  st    -Y, sw_timerH
  st    -Y, tempAL
  st    -Y, tempAH
  st    -Y, address_reg0
  st    -Y, address_reg1
  lds   dht_counter, dht_counter_var

main_loop:
  clr   r16
  sts   sensor_states, r16

  ldi   tempAL, LOW(channel_signals)
  sts   signal_address + 0, tempAL
  ldi   tempAL, HIGH(channel_signals)
  sts   signal_address + 1, tempAL

  ldi   tempAL, LOW(channel_references)
  sts   reference_address + 0, tempAL
  ldi   tempAL, HIGH(channel_references)
  sts   reference_address + 1, tempAL

  ldi   tempAL, LOW(sensor_general_counter)
  sts   counter_address + 0, tempAL
  ldi   tempAL, HIGH(sensor_general_counter)
  sts   counter_address + 1, tempAL

  ldi   tempAL, LOW(sensor_delta)
  sts   delta_address + 0, tempAL
  ldi   tempAL, HIGH(sensor_delta)
  sts   delta_address + 1, tempAL

  ldi   tempAL, LOW(sensor_state)
  sts   state_address + 0, tempAL
  ldi   tempAL, HIGH(sensor_state)
  sts   state_address + 1, tempAL

  ldi   tempAL, LOW(sensor_ndil_counter)
  sts   di_address + 0, tempAL
  ldi   tempAL, HIGH(sensor_ndil_counter)
  sts   di_address + 1, tempAL

  lds   tim_dly, time_current_ms

  tst   dht_counter
  breq  dont_clear_tim_dly
  dec   dht_counter

dont_clear_tim_dly:
  cli
  push  signal_valL // why??
  push  signal_valH // why??

  lds   address_reg0, signal_address + 0 // Z
  lds   address_reg1, signal_address + 1 // Z+1

  lds   signal_valL, state_address + 0  // Y
  lds   signal_valH, state_address + 1  // Y+1

  ldi   tempBH, (1 + (DEF_QT_ADC_CHANNEL_START_INDEX-1))

acquire_loop:
  ld    state, Y+

  push  signal_valL     // sensor_state + 0
  push  signal_valH     // sensor_state + 1

  ld    signal_valL, Z+ // signal_address + 0
  ld    signal_valH, Z  // signal_address + 1

  ldi   tempAH, 0x01
  clr   tempAL

shift_loop:
  add   tempAH, tempAH
  inc   tempAL
  cpse  tempAL, tempBH
  rjmp  shift_loop
  clr   detect_int
  clr   deltaL
  clr   deltaH

burst_length_loop:
  ldi   tempAL, 0x08 // ADCMUX input = GND
  out   ADMUX, tempAL

  in    tempAL, PORTA
  or    tempAL, tempAH
  out   PORTA, tempAL

  in    tempAL, DDRA
  or    tempAL, tempAH
  out   DDRA, tempAL

  rcall delay

  com   tempAH
  and   tempAL, tempAH
  out   DDRA, tempAL
  out   ADMUX, tempBH

  rcall delay

  in    tempAL, ADCSRA
  ori   tempAL, 0x40 // set ADSC => start conversion
  out   ADCSRA, tempAL

loop2:
  in    tempAL, ADCSRA
  sbrs  tempAL, 4
  rjmp  loop2

  in    sw_timerL, ADCL
  in    sw_timerH, ADCH

  ldi   tempAL, 0x08 // ADCMUX input = GND
  out   ADMUX, tempAL

  in    tempAL, PORTA
  and   tempAL, tempAH
  out   PORTA, tempAL

  com   tempAH
  in    tempAL, DDRA
  or    tempAL, tempAH
  out   DDRA, tempAL

  in    tempAL, ADCSRA
  ori   tempAL, 0x10  // clear ADC interrupt flag
  out   ADCSRA, tempAL

  clr   tempAL
  out   ADMUX, tempAL

  com   tempAH
  in    tempAL, DDRA
  and   tempAL, tempAH
  out   DDRA, tempAL

  out   ADMUX, tempBH

  rcall delay
  in    tempAL, ADCSRA
  ori   tempAL, 0x40.  // set ADSC => start conversion
  out   ADCSRA, tempAL

loop3:
  in    tempAL, ADCSRA
  sbrs  tempAL, 4
  rjmp  loop3

  in    tempAL, PORTA
  and   tempAL, tempAH
  out   PORTA, tempAL

  com   tempAH
  in    tempAL, DDRA
  or    tempAL, tempAH
  out   DDRA, tempAL

  push  tempAH

  in    tempBL, ADCL
  in    tempAH, ADCH

  in    tempAL, ADCSRA
  ori   tempAL, 0x10 // clear ADC interrupt flag
  out   ADCSRA, tempAL

  subi  sw_timerL, 0x01
  sbci  sw_timerH, 0xFC // -4

  sub   sw_timerL, tempBL
  sbc   sw_timerH, tempAH

  pop   tempAH
  clc

  ror   sw_timerH
  ror   sw_timerL

  add   deltaL, sw_timerL
  adc   deltaH, sw_timerH

  inc   detect_int
  cpi   detect_int, DEF_QT_BURST_LENGTH
  breq  completed_burst_length
  rjmp  burst_length_loop

completed_burst_length:
  ldi   detect_int, DEF_QT_BURST_LENGTH // not needed -> can be removed

resolution_adjust_loop:
  cpi   detect_int, 0x04
  brcs  end_resolution_adjust // 0x04 > detect_int;
  clc

  ror   deltaH
  ror   deltaL

  lsr   detect_int
  lsr   detect_int

  rjmp  resolution_adjust_loop

end_resolution_adjust:
  cpi   state, 0x03
  brne  iir_filter

  mov   signal_valL, deltaL
  mov   signal_valH, deltaH
  rjmp  save_signal

iir_filter:
  mov   tempAH, signal_valH // signal_address
  mov   tempBL, signal_valL // signal_address
  clc
  rol   signal_valL
  rol   signal_valH
  clc
  rol   signal_valL
  rol   signal_valH

  sub   signal_valL, tempBL
  sbc   signal_valH, tempAH

  add   signal_valL, deltaL
  adc   signal_valH, deltaH
  clc
  ror   signal_valH
  ror   signal_valL
  clc
  ror   signal_valH
  ror   signal_valL

save_signal:
  st    Z, signal_valH
  st    -Z, signal_valL

  subi  address_reg0, 0xFE // -2

  pop   signal_valH // sensor_state + 0
  pop   signal_valL // sensor_state + 1

  inc   tempBH
  cpi   tempBH, (DEF_QT_ADC_CHANNEL_START_INDEX + DEF_QT_NUM_SENSORS_SYM)
  breq  exit_acquire

  // repeat loop for another channel
  mov   tempAH, signal_valH // sensor_state + 0
  mov   tempAL, signal_valL // sensor_state + 1
  pop   signal_valH // ????
  pop   signal_valL // ????
  sei
  rjmp  .+0 // let pending interrupt run before continue?
  cli
  push  signal_valL // ????
  push  signal_valH // ????
  mov   signal_valH, tempAH // sensor_state + 0
  mov   signal_valL, tempAL // sensor_state + 1
  rjmp  acquire_loop

exit_acquire:
  clr   tempBH
  clr   tempBL
  sts   di_flag, tempBL

  pop   signal_valH // ???? should not be required
  pop   signal_valL // ????

  sei

process:
  lds   address_reg0, signal_address + 0
  lds   address_reg1, signal_address + 1

  ld    tempAL, Z+
  sts   curr_signal + 0, tempAL
  ld    tempAL, Z
  sts   curr_signal + 1, tempAL

  lds   address_reg0, reference_address + 0
  lds   address_reg1, reference_address + 1
  ld    tempAL, Z+
  sts   reference + 0, tempAL
  ld    tempAL, Z
  sts   reference + 1, tempAL

  lds   address_reg0, counter_address + 0
  lds   address_reg1, counter_address + 1
  ld    sw_timerL, Z+
  ld    sw_timerH, Z

  lds   address_reg0, di_address + 0
  lds   address_reg1, di_address + 1
  ld    detect_int, Z

  lds   address_reg0, state_address + 0
  lds   address_reg1, state_address + 1
  ld    state, Z

  clr   tempAL // ??
  lds   deltaL, reference + 0
  inc   tempAL // ??
  lds   deltaH, reference + 1 // delta = reference

  com   deltaL
  com   deltaH // ~delta

  subi  deltaL, 0xFF // -1
  sbci  deltaH, 0xFF // -1 ; delta -= 0xFFFF => delta += 1

  st    -Y, address_reg0 // state_address
  st    -Y, address_reg1

  lds   address_reg0, curr_signal + 0
  lds   address_reg1, curr_signal + 1

  add   deltaL, address_reg0
  adc   deltaH, address_reg1 // delta += curr_signal

  ld    address_reg1, Y+ // state_address
  ld    address_reg0, Y+

  brmi  delta_neg
  tst   deltaH
  breq  deltaH_zero
  ldi   deltaL, 0xFF // deltaL = 0xFF if delta > 0

delta_neg:
deltaH_zero:
  cpi   state, 0x00
  brne  not_detect_state
  rjmp  detect_state // Long jump required

not_detect_state:
  cpi   state, 0x01
  breq  no_det_state
  cpi   state, 0x02
  brne  not_filter_state
  rjmp  filter_state

// ----------------------------------------------------------

not_filter_state:
calib_state:
  dec   sw_timerL

  ldi   tempBL, 0x80
  sts   di_flag, tempBL

  ldi   dht_counter, DEF_QT_DRIFT_HOLD_TIME

  cpi   sw_timerL, 0x0A
  brcc  sw_timerL_above_cal2 // sw_timerL >= 0x0A

  lds   tempAL, reference + 0
  lds   tempAH, reference + 1

  sbrs  deltaH, 7 //
  rjmp  posit // skip if deltaH is negative, jump if delta >= 0

  subi  tempAL, 0x01
  sbci  tempAH, 0x00 // tempA -= 1
  rjmp  end_drift

posit:
  tst   deltaL
  brne  drift_cal // deltaL != 0

  tst   deltaH
  breq  end_drift // deltaH == 0 // Signal = Reference.

drift_cal:
  subi  tempAL, 0xFF // -1
  sbci  tempAH, 0xFF // -1

end_drift:
  sts   reference + 0, tempAL
  sts   reference + 1, tempAH

end_drift2:
  tst   sw_timerL
  brne  dont_end_calibration

  rjmp  go_no_det // End calibration if sw_timeL is zero.

dont_end_calibration:
  rjmp  end_state

sw_timerL_above_cal2:
  st    -Y, address_reg0
  st    -Y, address_reg1

  lds   address_reg0, curr_signal + 0
  lds   address_reg1, curr_signal + 1
  sts   reference + 0, address_reg0
  sts   reference + 1, address_reg1 // reference = curr_signal

  ld    address_reg1, Y+
  ld    address_reg0, Y+
  rjmp  end_state // Jump to collection point for all states.


// ----------------------------------------------------------

no_det_state:
  sbrc  deltaH, 7
  rjmp  n_thres // jump if deltaH is negative

  rcall compare_delta_and_threshold // cp deltaL, sensor_threshold
  brcs  n_thres // branch if sensor_threshold > deltaL

  ldi   state, 0x02

  ldi   tempBL, 0x80
  sts   di_flag, tempBL

  ldi   detect_int, 0x01

  rjmp  end_thres_tst

n_thres:
  tst   tim_dly
  brne  no_track //Only drift if the time delay is zero

  sbrs  deltaH, 7
  rjmp  pos_delta // branch if deltaH >= 0

  cpi   deltaH, 0xFF
  brne  pos_tim // Skip ahead if high byte of delta was not 0xff, which means large negative delta

  rcall comp_delta_and_pos_recal // cp deltaL, sensor_recal_threshold
  brge  no_pos_tim // deltaL >= sensor_recal_threshold

pos_tim:
  inc   sw_timerH
  cpi   sw_timerH, (DEF_QT_POS_RECAL_DELAY + 1)
  brcc  skip_recal // branch if sw_timerH >= K
  rjmp  end_pos_tim

skip_recal:
  rcall cal_key
  rjmp  end_state

no_pos_tim:
  tst   dht_counter
  brne  no_track // Only drift when drift hold time is zero
  clr   sw_timerH

end_pos_tim:
  inc   sw_timerL
  cpi   sw_timerL, (129 + DEF_QT_POS_DRIFT_RATE)
  brcs  no_pos_drift_yet // branch if sw_timerL <= K; k > sw_timerL

  ldi   sw_timerL, 0x80

  lds   tempAL, reference + 0
  lds   tempAH, reference + 1
  subi  tempAL, 0x01
  sbci  tempAH, 0x00
  sts   reference + 0, tempAL
  sts   reference + 1, tempAH // Increment reference

no_pos_drift_yet:
  rjmp  end_drift2 //Praveesh?  TODO is this required?

pos_delta:
  tst   dht_counter
  brne  no_track // Only drift when drift hold time is zero

  clr   sw_timerH
  cp    deltaL, sw_timerH
  cpc   deltaH, sw_timerH // cp delta, 0
  breq  end_drift2 // is this required?

  dec   sw_timerL ; sw_timerL--

  cpi   sw_timerL, (128 - DEF_QT_NEG_DRIFT_RATE)
  brcc  no_neg_drift_yet // branch if sw_timerL >= k; k < sw_timerL

  ldi   sw_timerL, 0x80 // sw_timerL = 0x80

  // reference++;
  lds   tempAL, reference + 0
  lds   tempAH, reference + 1
  subi  tempAL, 0xFF // -1
  sbci  tempAH, 0xFF // -1
  sts   reference + 0, tempAL
  sts   reference + 1, tempAH

end_thres_tst:
no_track:
no_neg_drift_yet:
  rjmp  end_state

filter_state:
  ldi   tempBL, 0x80
  sts   di_flag, tempBL // di_flag = 0x80

  ldi   dht_counter, DEF_QT_DRIFT_HOLD_TIME

  sbrc  deltaH, 7
  rjmp  go_no_det // jump if delta < 0

  rcall compare_delta_and_threshold // cp deltaL, sensor_threshold
  brcc  delta_still_above_threshold // branch if sensor_threshold < deltaL; deltaL >= sensor_threshold

  rjmp  go_no_det // Go to no detect if signal not strong enough anymore.

delta_still_above_threshold:
  inc   detect_int

  rcall aks_check // AKS
  lds   tempAL, aks_flag // AKS
  sbrc  tempAL, 0 // AKS
  clr   detect_int // AKS

  cpi   detect_int, DEF_QT_DI
  brcs  no_detect_yet // branch if DEF_QT_DI > detect_int

  ldi   detect_int, DEF_QT_DI
  ldi   state, 0x00
  clr   sw_timerL
  clr   sw_timerH
  rjmp  end_state

no_detect_yet:
  clr   dht_counter
  rjmp  end_state

detect_state:
  ldi   dht_counter, DEF_QT_DRIFT_HOLD_TIME

  sbrc  deltaH, 7
  rjmp  go_no_det // Go to no detect if delta negative, ie signal larger than reference.

  rcall comp_delta_and_hyst // cp deltaL, sensor_hyst_threshold
  brcc  above_or_equal_hyst // branch if deltaL >= sensor_hyst_threshold. Skip ahead if delta above or equal to hysteresis value, ie still in detect.

  ldi   tempBL, 0x80
  sts   di_flag, tempBL // di_flag = 0x80

  tst   detect_int
  breq  go_no_det // Go to no detect if signal below hysteresis and detect int reached zero.

  dec   detect_int
  breq  go_no_det // Go to no detect if signal below hysteresis and detect int reached zero.

  rjmp  do_timeout // Skip past detect int increment, since we are below hysteresis.

above_or_equal_hyst:
  cpi   detect_int, DEF_QT_DI
  brcc  do_timeout // br if detect_int >= DEF_QT_DI.
  inc   detect_int // Increment detect_int only if its below DEF_QT_DI

do_timeout:
  tst   tim_dly
  brne  no_tim // br if tim_dly != 0; No timeout checking until time delay prescaler is zero.

  ldi   tempAL, LOW(DEF_QT_MAX_ON_DURATION)
  tst   tempAL
  brne  _orig_ // lo8(DEF_QT_MAX_ON_DURATION) != 0

  cpi   tempAL, HIGH(DEF_QT_MAX_ON_DURATION)
  breq  no_tim // hi8(DEF_QT_MAX_ON_DURATION) == 0

_orig_:
  subi  sw_timerL, 0xFF // -1
  sbci  sw_timerH, 0xFF // -1

  ldi   tempAL, HIGH(DEF_QT_MAX_ON_DURATION)

  cpi   sw_timerL, LOW(DEF_QT_MAX_ON_DURATION)
  cpc   sw_timerH, tempAL
  brcs  no_tim // branch if DEF_QT_MAX_ON_DURATION > sw_timer; sw_timer < DEF_QT_MAX_ON_DURATION

  rcall cal_key

no_tim:
  rjmp  end_state

go_no_det:
  ldi   state, 0x01
  clr   detect_int
  ldi   sw_timerL, 0x80
  clr   sw_timerH

end_state:
  lds   address_reg0, signal_address + 0
  lds   address_reg1, signal_address + 1
  st    -Y, tempAL
  st    -Y, tempAH

  lds   tempAL, curr_signal + 0
  lds   tempAH, curr_signal + 1
  st    Z+, tempAL
  st    Z+, tempAH

  ld    tempAH, Y+
  ld    tempAL, Y+
  sts   signal_address + 0, address_reg0
  sts   signal_address + 1, address_reg1

  lds   address_reg0, reference_address + 0
  lds   address_reg1, reference_address + 1
  lds   tempAL, reference + 0
  st    Z+, tempAL
  lds   tempAL, reference + 1
  st    Z+, tempAL
  sts   reference_address + 0, address_reg0
  sts   reference_address + 1, address_reg1

  lds   address_reg0, counter_address + 0
  lds   address_reg1, counter_address + 1
  st    Z+, sw_timerL
  st    Z+, sw_timerH

  sts   counter_address + 0, address_reg0
  sts   counter_address + 1, address_reg1

  lds   address_reg0, di_address + 0
  lds   address_reg1, di_address + 1
  st    Z+, detect_int

  sts   di_address + 0, address_reg0
  sts   di_address + 1, address_reg1
  lds   address_reg0, state_address + 0
  lds   address_reg1, state_address + 1
  st    Z+, state

  sts   state_address + 0, address_reg0
  sts   state_address + 1, address_reg1
  lds   address_reg0, delta_address + 0
  lds   address_reg1, delta_address + 1
  st    Z+, deltaL
  st    Z+, deltaH

  sts   delta_address + 0, address_reg0
  sts   delta_address + 1, address_reg1
  inc   tempBH
  cpi   tempBH, DEF_QT_NUM_SENSORS_SYM
  breq  process_over
  rjmp  process

process_over:
fast_di_check:
  lds   tempBL, di_flag
  tst   tempBL
  breq  di_resolved // br if di_flag == 0
  rjmp  main_loop // else jmp to main_loop

di_resolved: // qt_measure_sensors() end
  rcall handle_key_touch // Update Sensor states variable.
  sts   dht_counter_var, dht_counter

  ld    address_reg1, Y+
  ld    address_reg0, Y+
  ld    tempAH, Y+
  ld    tempAL, Y+
  ld    sw_timerH, Y+
  ld    sw_timerL, Y+
  ld    deltaH, Y+
  ld    deltaL, Y+
  ld    state, Y+
  ld    dht_counter, Y+
  ld    tempBH, Y+
  ld    tempBL, Y+
  ld    detect_int, Y+
  ld    tim_dly, Y+
  ret

push_util_regs:
  st    -Y, address_reg0
  st    -Y, address_reg1
  st    -Y, dht_counter
  ret

pop_util_regs:
  ld    dht_counter, Y+
  ld    address_reg1, Y+
  ld    address_reg0, Y+
  ret

delay:
  push  tempAL
  ldi   tempAL, DEF_CHARGE_SHARE_DELAY
  dec   tempAL
  brmi  return
  cpi   tempAL, 0x00
  brne  .-8 // $-6

return:
  pop   tempAL
  ret

/*
subtract_loop_delta_by_threshold:
  rcall push_util_regs
  ldi   address_reg0, LOW(sensor_threshold)
  ldi   address_reg1, HIGH(sensor_threshold)
  add   address_reg0, tempBL
  ld    dht_counter, Z
  sub   sw_timerL, dht_counter
  rcall pop_util_regs
  sbrc  sw_timerL, 7
  ldi   sw_timerL, 0x00
  ret

subtract_curr_delta_by_threshold:
  rcall push_util_regs
  ldi   address_reg0, LOW(sensor_threshold)
  ldi   address_reg1, HIGH(sensor_threshold)
  add   address_reg0, tempBH
  ld    dht_counter, Z
  sub   deltaL, dht_counter
  rcall pop_util_regs
  sbrc  deltaL, 7
  ldi   deltaL, 0x00
  ret
*/

compare_delta_and_threshold:
  rcall push_util_regs
  ldi   address_reg0, LOW(sensor_threshold)
  ldi   address_reg1, HIGH(sensor_threshold)
  rcall util_do_comp_and_pop
  ret

comp_delta_and_hyst:
  rcall push_util_regs
  ldi   address_reg0, LOW(sensor_hyst_threshold)
  ldi   address_reg1, HIGH(sensor_hyst_threshold)
  rcall util_do_comp_and_pop
  ret

comp_delta_and_pos_recal:
  rcall push_util_regs
  ldi   address_reg0, LOW(sensor_recal_threshold)
  ldi   address_reg1, HIGH(sensor_recal_threshold)
  rcall util_do_comp_and_pop
  ret

util_do_comp_and_pop:
  add   address_reg0, tempBH
  ld    dht_counter, Z
  cp    deltaL, dht_counter
  rcall pop_util_regs
  ret