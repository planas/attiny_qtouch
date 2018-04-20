aks_check: ; 0x22
  clr   r18
  sts   0x00, r18
  st    -Y, r20
  st    -Y, r17
  st    -Y, r26
  st    -Y, r27
  st    -Y, r25
  st    -Y, r24
  st    -Y, r16
  ldi   r30, LOW(sensor_state)
  ldi   r31, HIGH(sensor_state)
  ldi   r26, LOW(sensor_delta)
  ldi   r27, HIGH(sensor_delta)
  ldi   r17, DEF_QT_AKS_ENABLE
  tst   r17
  breq  restore

aks_loop: ; 0x42
  ld    r20, Z+
  ld    r24, X+
  ld    r25, X+
  sbrc  r25, 7
  ldi   r24, 0x00
  ldi   r16, DEF_QT_AKS_ENABLE
  cp    r19, r18
  breq  end_aks_check
  cp    r17, r16
  brne  end_aks_check
  andi  r20, 0x0F
  tst   r20
  breq  aks_true
  cpi   r24, 0x00
  breq  chk_for_sensor_to_supress
  rcall substract_loop_delta_by_threshold
  rcall substract_curr_delta_by_threshold

chk_for_sensor_to_supress: ; 0x64
  cp    r22, r24
  brcs  aks_true
  rjmp  end_aks_check

aks_true: ; 0x6a
  ldi   r16, 0x01
  sts   0x00, r16
  rjmp  restore

end_aks_check: ; 0x70
  inc   r18
  clr   r16
  sts   0x00, r16
  cpi   r18, DEF_QT_NUM_SENSORS_SYM
  brne  aks_loop

restore: ; 0x7a
  ld    r16, Y+
  ld    r24, Y+
  ld    r25, Y+
  ld    r27, Y+
  ld    r26, Y+
  ld    r17, Y+
  ld    r20, Y+
  ret

start_cal:
  ldi   r21, 0x03
  clr   r18
  sts   0x01, r18
  ldi   r24, 0x0F
  clr   r19
  clr   r25
  ldi   r30, LOW(sensor_state)
  ldi   r31, HIGH(sensor_state)

cal_state: ; 0x9a
  st    Z+, r21
  inc   r19
  cpi   r19, DEF_QT_NUM_SENSORS_SYM
  brne  cal_state
  clr   r19
  clr   r21
  ldi   r30, LOW(sensor_general_counter)
  ldi   r31, HIGH(sensor_general_counter)

cal_counter: ; 0xaa
  st    Z+, r24
  st    Z+, r25
  inc   r19
  cpi   r19, DEF_QT_NUM_SENSORS_SYM
  brne  cal_counter ; 0xaa
  ret

cal_key: ; 0xb6
  ldi   r21, 0x03
  ldi   r24, 0x0F
  ret

handle_key_touch: ; 0xbc
  clr   r19
  ldi   r27, 0x00
  ldi   r18, 0x01
  ldi   r30, LOW(sensor_state)
  ldi   r31, HIGH(sensor_state)

touch_loop: ; 0xc6
  ld    r26, Z+
  tst   r26
  brne  time_out
  or    r27, r18

time_out: ; 0xce
  inc   r19
  add   r18, r18
  cpi   r19, DEF_QT_NUM_SENSORS_SYM
  brne  touch_loop
  sts   0x30, r27
  ret

qt_init_sensing: ; 0xda
  st    -Y, r20
  st    -Y, r21
  st    -Y, r18
  st    -Y, r19
  st    -Y, r24
  st    -Y, r25
  st    -Y, r30
  st    -Y, r31
  clr   r20
  sts   0x02, r20
  rcall start_cal
  ld    r31, Y+
  ld    r30, Y+
  ld    r25, Y+
  ld    r24, Y+
  ld    r19, Y+
  ld    r18, Y+
  ld    r21, Y+
  ld    r20, Y+
  ret

qt_measure_sensors: ; 0x102
  st    -Y, r16
  st    -Y, r17
  st    -Y, r18
  st    -Y, r19
  st    -Y, r20
  st    -Y, r21
  st    -Y, r22
  st    -Y, r23
  st    -Y, r24
  st    -Y, r25
  st    -Y, r26
  st    -Y, r27
  st    -Y, r30
  st    -Y, r31
  lds   r20, 0x02

main_loop: ; 0x120
  clr   r16
  sts   0x30, r16
  ldi   r26, 0x83
  sts   0x72, r26
  ldi   r26, 0x00
  sts   0x73, r26
  ldi   r26, 0x8D
  sts   0x70, r26
  ldi   r26, 0x00
  sts   0x71, r26
  ldi   r26, 0xA1
  sts   0x7a, r26
  ldi   r26, 0x00
  sts   0x7b, r26
  ldi   r26, 0x97
  sts   0x78, r26
  ldi   r26, 0x00
  sts   0x79, r26
  ldi   r26, 0xAB
  sts   0x74, r26
  ldi   r26, 0x00
  sts   0x75, r26
  ldi   r26, 0xB1
  sts   0x76, r26
  ldi   r26, 0x00
  sts   0x77, r26
  lds   r16, 0x36
  tst   r20
  breq  dont_clear_tim_dly
  dec   r20

dont_clear_tim_dly: ; 0x15c
  cli
  push  r28
  push  r29
  lds   r30, 0x72
  lds   r31, 0x73
  lds   r28, 0x74
  lds   r29, 0x75
  ldi   r19, DEF_QT_ADC_CHANNEL_START_INDEX

acquire_loop: ; 0x16c
  ld    r21, Y+
  push  r28
  push  r29
  ld    r28, Z+
  ld    r29, Z
  ldi   r27, 0x01
  clr   r26

shift_loop: ; 0x17a
  add   r27, r27
  inc   r26
  cpse  r26, r19
  rjmp  shift_loop
  clr   r17
  clr   r22
  clr   r23

burst_length_loop: ; 0x188
  ldi   r26, 0x08
  out   0x10, r26
  in    r26, 0x02
  or    r26, r27
  out   0x02, r26
  in    r26, 0x01
  or    r26, r27
  out   0x01, r26
  rcall delay
  com   r27
  and   r26, r27
  out   0x01, r26
  out   0x10, r19
  rcall delay
  in    r26, 0x12
  ori   r26, 0x40
  out   0x12, r26

loop2: ; 0x1aa
  in    r26, 0x12
  sbrs  r26, 4
  rjmp  loop2
  in    r24, 0x0e
  in    r25, 0x0f
  ldi   r26, 0x08
  out   0x10, r26
  in    r26, 0x02
  and   r26, r27
  out   0x02, r26
  com   r27
  in    r26, 0x01
  or    r26, r27
  out   0x01, r26
  in    r26, 0x12
  ori   r26, 0x10
  out   0x12, r26
  clr   r26
  out   0x10, r26
  com   r27
  in    r26, 0x01
  and   r26, r27
  out   0x01, r26
  out   0x10, r19
  rcall delay
  in    r26, 0x12
  ori   r26, 0x40
  out   0x12, r26

loop3: ; 0x1e2
  in    r26, 0x12
  sbrs  r26, 4
  rjmp  loop3
  in    r26, 0x02
  and   r26, r27
  out   0x02, r26
  com   r27
  in    r26, 0x01
  or    r26, r27
  out   0x01, r26
  push  r27
  in    r18, 0x0e
  in    r27, 0x0f
  in    r26, 0x12
  ori   r26, 0x10
  out   0x12, r26
  subi  r24, 0x01
  sbci  r25, 0xFC
  sub   r24, r18
  sbc   r25, r27
  pop   r27
  clc
  ror   r25
  ror   r24
  add   r22, r24
  adc   r23, r25
  inc   r17
  cpi   r17, DEF_QT_BURST_LENGTH
  breq  completed_burst_length
  rjmp  burst_length_loop

completed_burst_length: ; 0x21e
  ldi   r17, DEF_QT_BURST_LENGTH

resolution_adjust_loop: ; 0x220
  cpi   r17, 0x04
  brcs  end_resolution_adjust
  clc
  ror   r23
  ror   r22
  lsr   r17
  lsr   r17
  rjmp  resolution_adjust_loop

end_resolution_adjust: ; 0x230
  cpi   r21, 0x03
  brne  iir_filter
  mov   r28, r22
  mov   r29, r23
  rjmp  save_signal

iir_filter: ; 0x23a
  mov   r27, r29
  mov   r18, r28
  clc
  rol   r28
  rol   r29
  clc
  rol   r28
  rol   r29
  sub   r28, r18
  sbc   r29, r27
  add   r28, r22
  adc   r29, r23
  clc
  ror   r29
  ror   r28
  clc
  ror   r29
  ror   r28

save_signal: ; 0x25e
  st    Z, r29
  st    -Z, r28
  subi  r30, 0xFE
  pop   r29
  pop   r28
  inc   r19
  cpi   r19, (DEF_QT_ADC_CHANNEL_START_INDEX + DEF_QT_NUM_SENSORS_SYM)
  breq  exit_acquire
  mov   r27, r29
  mov   r26, r28
  pop   r29
  pop   r28
  sei
  rjmp  .+0 ;  nop?
  cli
  push  r28
  push  r29
  mov   r29, r27
  mov   r28, r26
  rjmp  acquire_loop

exit_acquire: ; 0x286
  clr   r19
  clr   r18
  sts   0x01, r18
  pop   r29
  pop   r28
  sei

process: ; 0x292
  lds   r30, 0x72
  lds   r31, 0x73
  ld    r26, Z+
  sts   0x7e, r26
  ld    r26, Z
  sts   0x7f, r26
  lds   r30, 0x70
  lds   r31, 0x71
  ld    r26, Z+
  sts   0x7c, r26
  ld    r26, Z
  sts   0x7d, r26
  lds   r30, 0x7a
  lds   r31, 0x7b
  ld    r24, Z+
  ld    r25, Z
  lds   r30, 0x76
  lds   r31, 0x77
  ld    r17, Z
  lds   r30, 0x74
  lds   r31, 0x75
  ld    r21, Z
  clr   r26
  lds   r22, 0x7c
  inc   r26
  lds   r23, 0x7d
  com   r22
  com   r23
  subi  r22, 0xFF
  sbci  r23, 0xFF
  st    -Y, r30
  st    -Y, r31
  lds   r30, 0x7e
  lds   r31, 0x7f
  add   r22, r30
  adc   r23, r31
  ld    r31, Y+
  ld    r30, Y+
  brmi  delta_neg
  tst   r23
  breq  deltaH_zero
  ldi   r22, 0xFF

delta_neg: ; 0x2e6
deltaH_zero:
  cpi   r21, 0x00
  brne  not_detect_state
  rjmp  detect_state

not_detect_state: ; 0x2ec
  cpi   r21, 0x01
  breq  no_det_state
  cpi   r21, 0x02
  brne  not_filter_state
  rjmp  filter_state

not_filter_state: ; 0x2f6
calib_state:
  dec   r24
  ldi   r18, 0x80
  sts   0x01, r18
  ldi   r20, DEF_QT_DRIFT_HOLD_TIME
  cpi   r24, 0x0A
  brcc  sw_timerL_above_cal2
  lds   r26, 0x7c
  lds   r27, 0x7d
  sbrs  r23, 7
  rjmp  posit
  subi  r26, 0x01
  sbci  r27, 0x00
  rjmp  end_drift

posit: ; 0x310
  tst   r22
  brne  drift_cal
  tst   r23
  breq  end_drift

drift_cal: ; 0x318
  subi  r26, 0xFF
  sbci  r27, 0xFF

end_drift: ; 0x31c
  sts   0x7c, r26
  sts   0x7d, r27

end_drift2: ; 0x320
  tst   r24
  brne  dont_end_calibration
  rjmp  go_no_det

dont_end_calibration: ; 0x326
  rjmp  end_state

sw_timerL_above_cal2: ; 0x328
  st    -Y, r30
  st    -Y, r31
  lds   r30, 0x7e
  lds   r31, 0x7f
  sts   0x7c, r30
  sts   0x7d, r31
  ld    r31, Y+
  ld    r30, Y+
  rjmp  end_state

no_det_state: ; 0x33a
  sbrc  r23, 7
  rjmp  n_thres
  rcall compare_delta_and_threshold
  brcs  n_thres
  ldi   r21, 0x02
  ldi   r18, 0x80
  sts   0x01, r18
  ldi   r17, 0x01
  rjmp  end_thres_tst__no_track__no_neg_drift_yet

n_thres: ; 0x34c
  tst   r16
  brne  no_track
  sbrs  r23, 7
  rjmp  pos_delta
  cpi   r23, 0xFF
  brne  pos_tim
  rcall comp_delta_and_pos_recal
  brge  no_pos_tim

pos_tim: ; 0x35c
  inc   r25
  cpi   r25, (DEF_QT_POS_RECAL_DELAY + 1)
  brcc  skip_recal
  rjmp  end_pos_tim

skip_recal: ; 0x364
  rcall cal_key
  rjmp  end_state

no_pos_tim: ; 0x368
  tst   r20
  brne  no_track
  clr   r25

end_pos_tim: ; 0x36e
  inc   r24
  cpi   r24, (129 + DEF_QT_POS_DRIFT_RATE)
  brcs  no_pos_drift_yet
  ldi   r24, 0x80
  lds   r26, 0x7c
  lds   r27, 0x7d
  subi  r26, 0x01
  sbci  r27, 0x00
  sts   0x7c, r26
  sts   0x7d, r27

no_pos_drift_yet: ; 0x382
  rjmp  end_drift2

pos_delta: ; 0x384
  tst   r20
  brne  no_track
  clr   r25
  cp    r22, r25
  cpc   r23, r25
  breq  end_drift2
  dec   r24
  cpi   r24, (128 - DEF_QT_NEG_DRIFT_RATE)
  brcc  no_neg_drift_yet
  ldi   r24, 0x80
  lds   r26, 0x7c
  lds   r27, 0x7d
  subi  r26, 0xFF
  sbci  r27, 0xFF
  sts   0x7c, r26
  sts   0x7d, r27

end_thres_tst: ; 0x3a4
no_track:
no_neg_drift_yet:
  rjmp  end_state

filter_state: ; 0x3a6
  ldi   r18, 0x80
  sts   0x01, r18
  ldi   r20, DEF_QT_DRIFT_HOLD_TIME
  sbrc  r23, 7
  rjmp  go_no_det
  rcall compare_delta_and_threshold
  brcc  delta_still_above_threshold
  rjmp  go_no_det

delta_still_above_threshold: ; 0x3b6
  inc   r17
  rcall aks_check
  lds   r26, 0x00
  sbrc  r26, 0
  clr   r17
  cpi   r17, DEF_QT_DI
  brcs  no_detect_yet
  ldi   r17, DEF_QT_DI
  ldi   r21, 0x00
  clr   r24
  clr   r25
  rjmp  end_state

no_detect_yet: ; 0x3ce
  clr   r20
  rjmp  end_state

detect_state: ; 0x3d2
  ldi   r20, DEF_QT_DRIFT_HOLD_TIME
  sbrc  r23, 7
  rjmp  go_no_det
  rcall comp_delta_and_hyst
  brcc  above_or_equal_hyst
  ldi   r18, 0x80
  sts   0x01, r18
  tst   r17
  breq  go_no_det
  dec   r17
  breq  go_no_det
  rjmp  do_timeout

above_or_equal_hyst: ; 0x3ea
  cpi   r17, DEF_QT_DI
  brcc  do_timeout
  inc   r17

do_timeout: ; 0x3f0
  tst   r16
  brne  no_tim
  ldi   r26, LOW(DEF_QT_MAX_ON_DURATION)
  tst   r26
  brne  _orig_
  cpi   r26, HIGH(DEF_QT_MAX_ON_DURATION)
  breq  no_tim

_orig_: ; 0x3fe
  subi  r24, 0xFF
  sbci  r25, 0xFF
  ldi   r26, HIGH(DEF_QT_MAX_ON_DURATION)
  cpi   r24, LOW(DEF_QT_MAX_ON_DURATION)
  cpc   r25, r26
  brcs  no_tim
  rcall cal_key

no_tim: ; 0x40c
  rjmp  end_state

go_no_det: ; 0x40e
  ldi   r21, 0x01
  clr   r17
  ldi   r24, 0x80
  clr   r25

end_state: ; 0x416
  lds   r30, 0x72
  lds   r31, 0x73
  st    -Y, r26
  st    -Y, r27
  lds   r26, 0x7e
  lds   r27, 0x7f
  st    Z+, r26
  st    Z+, r27
  ld    r27, Y+
  ld    r26, Y+
  sts   0x72, r30
  sts   0x73, r31
  lds   r30, 0x70
  lds   r31, 0x71
  lds   r26, 0x7c
  st    Z+, r26
  lds   r26, 0x7d
  st    Z+, r26
  sts   0x70, r30
  sts   0x71, r31
  lds   r30, 0x7a
  lds   r31, 0x7b
  st    Z+, r24
  st    Z+, r25
  sts   0x7a, r30
  sts   0x7b, r31
  lds   r30, 0x76
  lds   r31, 0x77
  st    Z+, r17
  sts   0x76, r30
  sts   0x77, r31
  lds   r30, 0x74
  lds   r31, 0x75
  st    Z+, r21
  sts   0x74, r30
  sts   0x75, r31
  lds   r30, 0x78
  lds   r31, 0x79
  st    Z+, r22
  st    Z+, r23
  sts   0x78, r30
  sts   0x79, r31
  inc   r19
  cpi   r19, DEF_QT_NUM_SENSORS_SYM
  breq  process_over
  rjmp  process

process_over: ; 0x472
fast_di_check:
  lds   r18, 0x01
  tst   r18
  breq  di_resolved
  rjmp  main_loop

di_resolved: ; 0x47a
  rcall handle_key_touch
  sts   0x02, r20
  ld    r31, Y+
  ld    r30, Y+
  ld    r27, Y+
  ld    r26, Y+
  ld    r25, Y+
  ld    r24, Y+
  ld    r23, Y+
  ld    r22, Y+
  ld    r21, Y+
  ld    r20, Y+
  ld    r19, Y+
  ld    r18, Y+
  ld    r17, Y+
  ld    r16, Y+
  ret

push_util_regs: ; 0x49c
  st    -Y, r30
  st    -Y, r31
  st    -Y, r20
  ret

pop_util_regs: ; 0x4a4
  ld    r20, Y+
  ld    r31, Y+
  ld    r30, Y+
  ret

delay: ; 0x4ac
  push  r26
  ldi   r26, DEF_CHARGE_SHARE_DELAY
  dec   r26
  brmi  return
  cpi   r26, 0x00
  brne  .-8 ; $-6

return: ; 0x4b8
  pop   r26
  ret

subtract_loop_delta_by_threshold: ; 0x4bc
  rcall push_util_regs
  ldi   r30, LOW(sensor_threshold)
  ldi   r31, HIGH(sensor_threshold)
  add   r30, r18
  ld    r20, Z
  sub   r24, r20
  rcall pop_util_regs
  sbrc  r24, 7
  ldi   r24, 0x00
  ret

subtract_curr_delta_by_threshold: ; 0x4d0
  rcall push_util_regs
  ldi   r30, LOW(sensor_threshold)
  ldi   r31, HIGH(sensor_threshold)
  add   r30, r19
  ld    r20, Z
  sub   r22, r20
  rcall pop_util_regs
  sbrc  r22, 7
  ldi   r22, 0x00
  ret

compare_delta_and_threshold: ; 0x4e4
  rcall push_util_regs
  ldi   r30, LOW(sensor_threshold)
  ldi   r31, HIGH(sensor_threshold)
  rcall util_do_comp_and_pop
  ret

comp_delta_and_hyst: ; 0x4ee
  rcall push_util_regs
  ldi   r30, LOW(sensor_hyst_threshold)
  ldi   r31, HIGH(sensor_hyst_threshold)
  rcall util_do_comp_and_pop
  ret

comp_delta_and_pos_recal: ; 0x4f8
  rcall push_util_regs
  ldi   r30, LOW(sensor_recal_threshold)
  ldi   r31, HIGH(sensor_recal_threshold)
  rcall util_do_comp_and_pop
  ret

util_do_comp_and_pop: ; 0x502
  add   r30, r19
  ld    r20, Z
  cp    r22, r20
  rcall pop_util_regs
  ret