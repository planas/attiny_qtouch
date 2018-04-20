#include <stdbool.h>
#include <avr/io.h>

typedef enum {
  DETECT_STATE = 0,
  NO_DETECT_STATE,
  FILTER_STATE,
  CALIBRATION_STATE
} sensor_state_t;

/* Sensor Threshold level setting for each sensor. */
static const uint8_t sensor_threshold[DEF_QT_NUM_SENSORS] = {
  #if DEF_QT_NUM_SENSORS > 0u
    DEF_QT_SENSOR_0_THRESHOLD,
  #endif
  #if DEF_QT_NUM_SENSORS > 1u
    DEF_QT_SENSOR_1_THRESHOLD,
  #endif
  #if DEF_QT_NUM_SENSORS > 2u
    DEF_QT_SENSOR_2_THRESHOLD,
  #endif
  #if DEF_QT_NUM_SENSORS > 3u
    DEF_QT_SENSOR_3_THRESHOLD,
  #endif
  #if DEF_QT_NUM_SENSORS > 4u
    DEF_QT_SENSOR_4_THRESHOLD
  #endif
};

/* Sensor Hysteresis Threshold level for each sensor. */
static const uint8_t sensor_hyst_threshold[DEF_QT_NUM_SENSORS] = {
  #if DEF_QT_NUM_SENSORS > 0u
    SENSOR_0_HYST_THRESHOLD,
  #endif
  #if DEF_QT_NUM_SENSORS > 1u
    SENSOR_1_HYST_THRESHOLD,
  #endif
  #if DEF_QT_NUM_SENSORS > 2u
    SENSOR_2_HYST_THRESHOLD,
  #endif
  #if DEF_QT_NUM_SENSORS > 3u
    SENSOR_3_HYST_THRESHOLD,
  #endif
  #if DEF_QT_NUM_SENSORS > 4u
    SENSOR_4_HYST_THRESHOLD
  #endif
};

/* Sensor Recalibration Threshold level for each sensor. */
static const uint8_t sensor_recal_threshold[DEF_QT_NUM_SENSORS] = {
  #if DEF_QT_NUM_SENSORS > 0u
    SENSOR_0_RECAL_THRESHOLD,
  #endif
  #if DEF_QT_NUM_SENSORS > 1u
    SENSOR_1_RECAL_THRESHOLD,
  #endif
  #if DEF_QT_NUM_SENSORS > 2u
    SENSOR_2_RECAL_THRESHOLD,
  #endif
  #if DEF_QT_NUM_SENSORS > 3u
    SENSOR_3_RECAL_THRESHOLD,
  #endif
  #if DEF_QT_NUM_SENSORS > 4u
    SENSOR_4_RECAL_THRESHOLD,
  #endif
};

/* General counter for each sensor, required for QTouch Library internal use. */
static uint16_t sensor_general_counter[DEF_QT_NUM_SENSORS] = {
  #if DEF_QT_NUM_SENSORS > 0u
    0x000F,
  #endif
  #if DEF_QT_NUM_SENSORS > 1u
    0x000F,
  #endif
  #if DEF_QT_NUM_SENSORS > 2u
    0x000F,
  #endif
  #if DEF_QT_NUM_SENSORS > 3u
    0x000F,
  #endif
  #if DEF_QT_NUM_SENSORS > 4u
    0x000F,
  #endif
};

/* Sensor state for each sensor.
   0 - Sensor in Detect state.
   1 - Sensor in no Detect state.
   2 - Sensor in Filter state.
   3 - Sensor in Calibration state. */
static sensor_state_t sensor_state[DEF_QT_NUM_SENSORS] = {
  #if DEF_QT_NUM_SENSORS > 0u
    CALIBRATION_STATE,
  #endif
  #if DEF_QT_NUM_SENSORS > 1u
    CALIBRATION_STATE,
  #endif
  #if DEF_QT_NUM_SENSORS > 2u
    CALIBRATION_STATE,
  #endif
  #if DEF_QT_NUM_SENSORS > 3u
    CALIBRATION_STATE,
  #endif
  #if DEF_QT_NUM_SENSORS > 4u
    CALIBRATION_STATE,
  #endif
};

/* Individual Sensor ON/OFF Status, indicated by bit position. */
static uint8_t sensor_states;

/* Sensor ndil counter for each sensor, required for QTouch Library internal use. */
static uint8_t sensor_ndil_counter[DEF_QT_NUM_SENSORS];

/* Reference signal for each sensor. */
static uint16_t channel_references[DEF_QT_NUM_SENSORS];

/* Measured signal on each sensor. */
static uint16_t channel_signals[DEF_QT_NUM_SENSORS];

/* Sensor delta for each sensor. */
static int16_t  sensor_delta[DEF_QT_NUM_SENSORS];

//static uint8_t aks_flag;
static uint8_t di_flag;
static uint8_t dht_counter;

static void handle_key_touch() {
  for(uint8_t i = 0; i < DEF_QT_NUM_SENSORS; i++) {
    if(sensor_state[i] == 0)
      sensor_states |= (1 << i);
  }
}

static void delay() {
  uint8_t i = DEF_CHARGE_SHARE_DELAY;
  while(--i > 0) {
    ;
  }
}

static uint16_t acquire(uint8_t channel)
{
  uint8_t mask = _BV(channel);

  ADMUX = 0x08;  // ADCMUX input = GND
  PORTA |= mask; // OUT HIGH
  DDRA  |= mask; // OUTPUT PIN

  delay();

  DDRA &= ~mask;   // INPUT PIN
  ADMUX = channel; // SAMPLE CHANNEL

  delay();

  ADCSRA |= 0x40;  // set ADSC => start conversion

  // wait for ADIF flag (conversion ended)
  while((ADCSRA & 0x04) == 0);

  uint16_t adc1 = (ADCH << 8) | ADCL; // read result

  //------------------------------------------------------------

  ADMUX = 0x08;    // ADCMUX input = GND
  PORTA &= ~mask;  // OUT LOW
  DDRA |= mask;    // OUTPUT PIN

  ADCSRA |= 0x10;  // clear ADC interrupt flag
  ADMUX = 0;       // ADCMUX input = PA0

  DDRA &= ~mask;   // INPUT PIN
  ADMUX = channel; // SAMPLE CHANNEL

  delay();

  ADCSRA |= 0x40; // set ADSC => start conversion

  // wait for ADIF flag (conversion ended)
  while((ADCSRA & 0x04) == 0);

  PORTA &= ~mask; // OUT LOW
  DDRA |= mask;   // OUTPUT PIN

  uint16_t adc2 = (ADCH << 8) | ADCL; // read result

  ADCSRA |= 0x10; // clear ADC interrupt flag

  return ((adc1 + 1023 - adc2) >> 1);
}

void process(uint8_t i)
{
  uint16_t curr_signal = channel_signals[i];
  uint16_t reference   = channel_references[i];
  uint8_t detect_int   = sensor_ndil_counter[i];
  sensor_state_t state = sensor_state[i];

  uint8_t sw_timerH = hi8(sensor_general_counter[i]);
  uint8_t sw_timerL = lo8(sensor_general_counter[i]);

  int16_t delta = ((int16_t)reference) + curr_signal;

  if(delta > 0) {
    delta |= 0x00FF; // deltaL = 0xFF
  }

  uint8_t go_no_det = FALSE;

  switch(state) {
    case DETECT_STATE: // VERIFIED
      dht_counter = DEF_QT_DRIFT_HOLD_TIME;

      // Go to no detect if delta negative, ie signal larger than reference.
      if(delta < 0) {
        go_no_det = TRUE;
        break;
      }

      // Skip ahead if delta above or equal to hysteresis value, ie still in detect.
      if(lo8(delta) >= sensor_hyst_threshold[i]) {
        //above_or_equal_hyst:
        if(detect_int < DEF_QT_DI) {
          detect_int++; // Increment detect_int only if its below DEF_QT_DI
        }
        //goto do_timeout;
      }
      else
      {
        di_flag = 0x80;

        // Go to no detect if signal below hysteresis
        // and detect int reached zero.
        if((detect_int == 0) || (--detect_int == 0)) {
          go_no_det = TRUE;
          break;
        }
        // Skip past detect int increment, since we are below hysteresis.
        //goto do_timeout;
      }

      //do_timeout:
      // No timeout checking until time delay prescaler is zero.
#if DEF_QT_MAX_ON_DURATION > 0
      if(time_current_ms == 0) {
        //_orig_:
        sw_timer++; // -0xFFFF

        if(((sw_timerH << 8) | sw_timerL)  >= DEF_QT_MAX_ON_DURATION) {
          //cal_key() :
          state = CALIBRATION_STATE;
          sw_timerL = 0x0F;
        }
      }
#endif
      break;

    case NO_DETECT_STATE:
      // delta negative or below threshold
      if((delta < 0) || (sensor_threshold[i] > lo8(delta))) {
        //Only drift if the time delay is zero
        if(time_current_ms != 0) {
          break; //goto end_state; // no_track
        }

        if(delta >= 0) {
          //pos_delta:

          // Only drift when drift hold time is zero
          if(dht_counter != 0) {
            break;
          }

          sw_timerH = 0;

          // is this required? (ATMEL)
          if(delta == 0) {
            //goto end_drift2;
            if(sw_timerL == 0) {
              go_no_det = TRUE;
            }
          }
          else
          {
            sw_timerL--;

            if(sw_timerL < (128 - DEF_QT_NEG_DRIFT_RATE)) {
              sw_timerL = 0x80;
              reference++;
            }
          }
          break; //goto end_state;
        }

        // Skip ahead if high byte of delta was not 0xff,
        // which means large negative delta
        if((hi8(delta) != 0xFF) || (lo8(delta) < sensor_recal_threshold[i])) {
          //pos_tim:
          sw_timerH++;

          if(sw_timerH >= (DEF_QT_POS_RECAL_DELAY + 1)) {
            //skip_recal:
              //cal_key():
            state = CALIBRATION_STATE;
            sw_timerL = 0x0F;
            break; //goto end_state;
          }
          //else
            //goto end_pos_tim;
        }
        else// if(lo8(delta) >= sensor_recal_threshold[i])
        {
          //no_pos_tim:
          if(dht_counter != 0) {
            // Only drift when drift hold time is zero
            break; //goto end_state; // no_track
          }

          sw_timerH = 0;
          //goto end_pos_tim;
        }

        //end_pos_tim:
        sw_timerL++;

        if(sw_timerL > (129 + DEF_QT_POS_DRIFT_RATE)) {
          sw_timerL = 0x80;
          reference--;
        }

        //goto end_drift2;
        if(sw_timerL == 0) {
          go_no_det = TRUE;
        }
      }
      else
      {
        state = FILTER_STATE;
        di_flag = 0x80;
        detect_int = 0x01;
        //goto end_state; // end_thres_tst
      }
      break;

    case FILTER_STATE: // VERIFIED
      di_flag = 0x80;
      dht_counter = DEF_QT_DRIFT_HOLD_TIME;

      //if(delta < 0) {
      //  go_no_det = TRUE;
      //  break;
      //}

      // Delta is positive and above the threshold
      if((delta >= 0) && (lo8(delta) >= sensor_threshold[i])) {
        //delta_still_above_threshold:
        detect_int++;

        /*
        aks_check();
        if((aks_flag & 0x01) != 0) {
          detect_int = 0;
        }
        */

        if(DEF_QT_DI > detect_int) {
          //not_detect_yet:
          dht_counter = 0;
        }
        else {
          detect_int = DEF_QT_DI;
          state = DETECT_STATE;
          sw_timerL = 0;
          sw_timerH = 0;
        }
      }
      else {
        go_no_det = TRUE;
      }
      break;

    case CALIBRATION_STATE: // VERIFIED!
      sw_timerL--;
      di_flag = 0x80;
      dht_counter = DEF_QT_DRIFT_HOLD_TIME;

      if(sw_timerL >= 0x0A) {
        // sw_timerL_above_cal2:
        reference = curr_signal;
      }
      else
      {
        if(delta > 0) {
          reference++;
        }
        else if(delta < 0) {
          reference--;
        }

        if(sw_timerL == 0) {
          go_no_det = TRUE;
        }
      }
      break;
  }

  if(go_no_det) {
    state = NO_DETECT_STATE;
    detect_int = 0;
    sw_timerL = 0x80;
    sw_timerH = 0x00;
  }

  //end_state:
  channel_signals[i] = curr_signal;
  channel_references[i] = reference;
  sensor_general_counter[i] = (sw_timerH << 8) | sw_timerL;
  sensor_ndil_counter[i] = detect_int;
  sensor_state[i] = state;
  sensor_delta[i] = delta;
}

void qt_init_sensing()
{
  //dht_counter = 0;

  for(uint8_t i = 0; i < DEF_QT_NUM_SENSORS; i++)
  {
    sensor_state[i] = CALIBRATION_STATE;
    sensor_general_counter[i] = 0x000F;
  }
}

void qt_measure_sensors(uint8_t time_current_ms)
{
  if(dht_counter != 0) // dht_counter > 0
    dht_counter--;

  for(uint8_t i = 0; i < DEF_QT_NUM_SENSORS; i++)
  {
    uint8_t detect_int = 0;
    uint16_t delta = 0;

    cli();

    // burst_length_loop
    while(detect_int < DEF_QT_BURST_LENGTH) {
      delta += acquire(DEF_QT_ADC_CHANNEL_START_INDEX + i);
      detect_int++;
    }

    // resolution_adjust_loop
    while(detect_int >= 0x04) {
      delta >>= 1;
      detect_int >>= 2;
    }

    if(state != CALIBRATION_STATE)
    {
      // IIR filter
      uint16_t signal = channel_signals[i];

      channel_signals[i] <<= 2; // signal * 4
      channel_signals[i] -= signal;
      channel_signals[i] += delta;
      channel_signals[i] >>= 2; // signal / 4
    }
    else
    {
      channel_signals[i] = delta;
    }

    sei();

    di_flag = 0;
    process(i);
  }

  if(di_flag == 0) {
    // di_resolved
    handle_key_touch();
  }
  else
  {
    qt_measure_sensors(); // uuhh...??
  }
}
