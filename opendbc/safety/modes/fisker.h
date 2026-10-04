#pragma once

#include "opendbc/safety/declarations.h"

// Fisker Ocean — angle steering control, AUTOSAR "Short SecOC" on the
// actuator commands. Panda enforces signal-level bounds and rate limits; it does
// NOT validate the SecOC MAC (the car gateway does that, and the key lives in
// userspace). Panda validates counters + frequency + actuator limits.
//
// Buses: ADASBUS = bus 0.
//   TX  0x1D0  ADAS_LatCtrl_SteerAnReq    steering angle (SecOC)
//   TX  0x1C0  ADAS_LatCtrl activation    plain E2E, lateral activation/status
//   TX  0x121  ADAS_LgtCtrl_AccelReq      accel (SecOC, longitudinal only)
//   TX  0x5FE  relay release request      consumed and rejected here, never transmitted — see below
//   TX  0x52A  ICC feature settings       bus 2 (to the ADAS module), plain E2E — see below
//   TX  0x35B  ICC SVS/BSD/APA settings   bus 2 (to the ADAS module), no E2E — optional relay
//   RX  0x115  wheel speeds -> vehicle_moving
//   RX  0x318  vehicle speed + brake pedal
//   RX  0x1C2  EPS steering angle
//   RX  0x1C4  EPS driver torque
//   RX  0x313  ADAS ACC state (bus 2) -> cruise engaged

static bool fisker_longitudinal = false;

// ICC_0x52A (ICC feature settings, 200 ms): openpilot reads the ICC's frame on bus 0 and
// re-sends it to the ADAS module on bus 2 with settings overridden (enables ACC), keeping the
// ICC's own AliveCounter. The ICC's original is blocked from bus 0 -> 2 only while openpilot's
// copies are flowing; if openpilot stops sending, forwarding resumes so the ADAS module never
// loses 0x52A (and, since the counter is the ICC's, sees no counter jump at the handoff).
// openpilot is already sending when this safety mode starts, so the window opens at init —
// otherwise the ICC's first frame would be forwarded alongside our copy with the same counter.
#define FISKER_ICC_RELAY_TIMEOUT_US 500000U  // 2.5 missed 200 ms cycles
static uint32_t fisker_icc_settings_tx_last = 0U;

// ICC_0x35B (SVS view requests + BSD/DOW/APA settings, 200 ms): same relay, but optional —
// openpilot only sends it when settings overrides are configured. No counter on this message,
// so the ICC's original is blocked only once openpilot's copies are actually arriving.
static bool fisker_icc_0x35b_tx_seen = false;
static uint32_t fisker_icc_0x35b_tx_last = 0U;

static bool fisker_icc_relay_active(uint32_t tx_last) {
  return safety_get_ts_elapsed(microsecond_timer_get(), tx_last) < FISKER_ICC_RELAY_TIMEOUT_US;
}

// Steering: raw CAN at 0.0625 deg/LSB, offset -780 deg. Raw 12480 == 0 deg.
#define FISKER_ANGLE_ZERO_CAN 12480

// Accel: raw CAN at 0.0004882 m/s^2/LSB, offset -16 m/s^2. raw = (accel + 16) / 0.0004882
#define FISKER_ACCEL_INACTIVE 32773  //  0.0 m/s^2
#define FISKER_ACCEL_MAX      34822  // +1.0 m/s^2 (mirrors CarControllerParams.ACCEL_MAX; see values.py for why)
#define FISKER_ACCEL_MIN      25604  // -3.5 m/s^2

// Most Ocean messages carry a 4-bit AliveCounter in byte 1 bits 8..11.
// The AliveCounter is the low nibble of byte 1 (DBC: start bit 11, 4 bits), 0..14.
static uint8_t fisker_get_counter(const CANPacket_t *msg) {
  return msg->data[1] & 0x0FU;
}

// ---- Relay arbiter -----------------------------------------------------------------------------
// openpilot replaces three stock ADAS messages while it drives: 0x1D0 (steering angle), 0x1C0
// (lateral activation) and, with the long flag, 0x121 (accel). For each one panda decides who
// reaches the vehicle bus from what it actually observes (our TX arriving, the stock RX), never
// from a flag that lags:
//   PASS       stock frames are forwarded.
//   TAKEN      our copies are flowing; stock frames are blocked. If ours stop for
//              FISKER_RELAY_FAIL_US the stock stream returns by itself (fail-safe).
//   RELEASING  openpilot is handing back (release frame, or cruise dropped for 0x121). Ours are
//              rejected so our last counter freezes, and the stock stream resumes at the first frame
//              whose counter continues ours (or is at most 3 ahead), so the receivers see a strictly
//              +1 counter sequence with, at worst, a short timing gap.
// Counters are the shared 0..14 alive counter. The forward decision runs before the RX hook for the
// same frame (panda/board/drivers/fdcan.h) and sees no data, so it works from the previous stock
// frame's counter: the frame being decided is oem_ctr + 1.
#define FISKER_RELAY_FAIL_US     50000U
#define FISKER_RELAY_CTRL_ADDR   0x5FEU   // pseudo address, no car message; byte 0 = bitmask of relays to release
#define FISKER_RELAY_TAKE_WINDOW 3U       // first frame of ours may be at most this far ahead of the stock counter
#define FISKER_RELAY_RESUME_JUMP 3U       // stock resumes at most this far ahead of our last counter

typedef enum { FISKER_RELAY_PASS = 0, FISKER_RELAY_TAKEN, FISKER_RELAY_RELEASING } fisker_relay_state_t;

typedef struct {
  uint32_t addr;
  uint8_t release_bit;          // bit in the release frame's first byte
  fisker_relay_state_t state;
  bool oem_seen;
  uint8_t oem_ctr;              // last stock counter received on bus 2
  uint8_t ours_ctr;             // counter of our last accepted frame
  uint32_t ours_ts;             // when it was accepted
} fisker_relay_t;

#define FISKER_NUM_RELAYS 3
static fisker_relay_t fisker_relays[FISKER_NUM_RELAYS] = {
  {.addr = 0x1D0U, .release_bit = 0x01U},
  {.addr = 0x1C0U, .release_bit = 0x02U},
  {.addr = 0x121U, .release_bit = 0x04U},
};

static void fisker_relays_reset(void) {
  for (int i = 0; i < FISKER_NUM_RELAYS; i++) {
    fisker_relays[i].state = FISKER_RELAY_PASS;
    fisker_relays[i].oem_seen = false;
    fisker_relays[i].oem_ctr = 0U;
    fisker_relays[i].ours_ctr = 0U;
    fisker_relays[i].ours_ts = 0U;
  }
}

static fisker_relay_t *fisker_relay_get(uint32_t addr) {
  fisker_relay_t *ret = NULL;
  for (int i = 0; i < FISKER_NUM_RELAYS; i++) {
    if (fisker_relays[i].addr == addr) {
      ret = &fisker_relays[i];
    }
  }
  return ret;
}

// a - b on the 0..14 alive counter
static uint8_t fisker_ctr_diff(uint8_t a, uint8_t b) {
  return (uint8_t)(((uint32_t)a + 15U - (uint32_t)b) % 15U);
}

// Applied lazily so every path that drops controls_allowed (brake, cruise state, heartbeat, lag) hands 0x121 back.
static void fisker_relay_check_long(fisker_relay_t *r) {
  if ((r->addr == 0x121U) && (r->state == FISKER_RELAY_TAKEN) && !controls_allowed) {
    r->state = FISKER_RELAY_RELEASING;
  }
}

// Fail-safe: ours stopped without a release request.
static void fisker_relay_check_failsafe(fisker_relay_t *r) {
  if ((r->state != FISKER_RELAY_PASS) && (safety_get_ts_elapsed(microsecond_timer_get(), r->ours_ts) > FISKER_RELAY_FAIL_US)) {
    r->state = FISKER_RELAY_PASS;
  }
}

// True if the stock frame being decided (counter oem_ctr + 1) is blocked.
static bool fisker_relay_blocks_stock(fisker_relay_t *r) {
  fisker_relay_check_long(r);
  fisker_relay_check_failsafe(r);
  if ((r->state == FISKER_RELAY_RELEASING) && r->oem_seen &&
      (fisker_ctr_diff(r->oem_ctr, r->ours_ctr) <= FISKER_RELAY_RESUME_JUMP)) {
    r->state = FISKER_RELAY_PASS;
  }
  return r->state != FISKER_RELAY_PASS;
}

// True if our frame with counter n is accepted; updates the relay state.
static bool fisker_relay_accept_ours(fisker_relay_t *r, uint8_t n) {
  fisker_relay_check_long(r);
  fisker_relay_check_failsafe(r);
  bool ok = false;
  if (n < 15U) {
    if (r->state == FISKER_RELAY_PASS) {
      // taking over: must continue the stock counter (a repeat of the last stock frame or up to 3 ahead)
      // Only with permission: cruise for accel, cruise or MADS for steering. Without it the stock
      // frames keep flowing even if something sends.
      bool may_take = (r->addr == 0x121U) ? controls_allowed : (controls_allowed || controls_allowed_lateral);
      if (may_take && (!r->oem_seen || (fisker_ctr_diff(n, r->oem_ctr) <= FISKER_RELAY_TAKE_WINDOW))) {
        r->state = FISKER_RELAY_TAKEN;
        ok = true;
      }
    } else if (r->state == FISKER_RELAY_TAKEN) {
      // strictly forward, never a repeat or a step back
      uint8_t d = fisker_ctr_diff(n, r->ours_ctr);
      ok = (d >= 1U) && (d <= 3U);
    } else {
      // RELEASING: frozen
    }
  }
  if (ok) {
    r->ours_ctr = n;
    r->ours_ts = microsecond_timer_get();
  }
  return ok;
}


static void fisker_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // EPS_0x1C2: measured steering wheel angle (BE 16-bit @ start 23 -> bytes 2,3).
    if (msg->addr == 0x1C2U) {
      int raw = (msg->data[2] << 8) | msg->data[3];
      update_sample(&angle_meas, raw - FISKER_ANGLE_ZERO_CAN);
    }

    // EPS_0x1C4: driver steering torque magnitude (BE 12-bit @ start 23 -> bytes 2, hi-nibble 3).
    if (msg->addr == 0x1C4U) {
      int torque = (msg->data[2] << 4) | (msg->data[3] >> 4);
      update_sample(&torque_driver, torque);
    }

    // ESP_0x318: vehicle speed (BE 16-bit @ start 47 -> bytes 5,6, 0.1 km/h) + brake pedal.
    if (msg->addr == 0x318U) {
      float speed = ((msg->data[5] << 8) | msg->data[6]) * 0.1f * KPH_TO_MS;
      UPDATE_VEHICLE_SPEED(speed);
      brake_pressed = ((msg->data[1] >> 6) & 0x1U) != 0U;  // ESP_BrkPedlSts @ bit 14
    }

    // ESP_0x115: front-right wheel speed (BE 14-bit @ start 21) -> vehicle_moving.
    if (msg->addr == 0x115U) {
      int whl_rf = ((msg->data[1] & 0x3FU) << 8) | msg->data[2];
      vehicle_moving = whl_rf > 0;
    }

    // MFS_0x514: MFSS steering-wheel buttons. MFS_RiBtnSouth (2-bit @ start 33, big-endian
    // -> byte 4 bits 0,1) is sunnypilot's MADS engage/disengage toggle. Any non-zero state
    // counts as pressed (1=short_press, 2=long_press, 3=reserved). Openpilot's carstate
    // emits a matching ButtonType.lkas edge on the same signal so the MADS state machine
    // observes both sides. MADS is what enables lateral without cruise engaged.
    if (msg->addr == 0x514U) {
      int rbs = msg->data[4] & 0x03U;
      mads_button_press = (rbs != 0) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }
  }

  if (msg->bus == 2U) {
    // Stock counters of the relayed messages (see the relay arbiter above).
    fisker_relay_t *relay = fisker_relay_get(msg->addr);
    if (relay != NULL) {
      relay->oem_ctr = fisker_get_counter(msg);
      relay->oem_seen = true;
    }

    // ADAS_0x313: ACC state (ADAS_Sts_ACC_ICC, 4-bit @ start 35 -> data[4] low nibble),
    // authored by the ADAS module on the cam side. ACC is enabled via the 0x52A ICC-settings
    // override, so this is the cruise source. Engaged = 3=Active, 4=Override,
    // 5=Standstill_active, 6=Standstill_wait, 11=Standstill_GoNotification (must match
    // carstate.py). Others: 0=ACC_Off 1=Initialization 2=Standby 7=Deactivation_brake
    // 8=Deactivation_other 9/10=Failure.
    //
    // NOTE: acc_main_on is intentionally NOT set from acc_state on fisker. Panda's MADS
    // state machine treats acc_main rising as an implicit MADS engagement trigger and
    // acc_main falling as a forced MADS disengagement. On the Ocean's 2-step ACC UX:
    //   - RiBtnNorth press: ACC Off->Standby. This is only "ready cruise", NOT a MADS
    //     engagement moment. Firing acc_main-rising would open lat here and block OEM's
    //     0x1D0 => LKA fault the moment the user preps cruise.
    //   - Brake during cruise: Active->Deactivation_brake->Standby. Sunnypilot default
    //     "steering_mode_on_brake = Remain Active" wants MADS to stay engaged. If acc_main
    //     followed the engaged state, brake would falling-edge and forcibly disengage MADS
    //     regardless of the user's steering-mode setting.
    // Leaving acc_main_on at its default (false) sidesteps both. MADS is engaged on this
    // port only via mads_button (RiBtnSouth on MFS_0x514 above) or via op_controls_allowed
    // rising when cruise actually engages (pcm_cruise_check below).
    if (msg->addr == 0x313U) {
      int acc_state = msg->data[4] & 0x0FU;
      bool cruise_engaged = (acc_state == 3) || (acc_state == 4) || (acc_state == 5) ||
                            (acc_state == 6) || (acc_state == 11);
      pcm_cruise_check(cruise_engaged);
    }
  }
}


// Steering angle limits (mirror CarControllerParams.ANGLE_LIMITS in values.py). The
// upstream AngleSteeringLimits struct in this openpilot release is minimal — the
// tizi-era fields (max_angle_error, angle_error_min_speed, angle_is_curvature,
// enforce_angle_error, inactive_angle_is_zero) were dropped from the C struct when
// angle-error enforcement moved into the VM-based limits path. Fisker's carcontroller
// still uses the non-VM apply_std_steer_angle_limits, so the 5 fields below are all
// that apply.
static const AngleSteeringLimits FISKER_STEERING_LIMITS = {
  .max_angle = 9600,             // 600 deg * 16 CAN/deg
  .angle_deg_to_can = 16.0f,     // 1 / 0.0625
  .angle_rate_up_lookup = {
    {0., 5., 25.},
    {2.5, 1.5, 0.2}
  },
  .angle_rate_down_lookup = {
    {0., 5., 25.},
    {5.0, 2.0, 0.3}
  },
};

static const LongitudinalLimits FISKER_LONG_LIMITS = {
  .max_accel = FISKER_ACCEL_MAX,
  .min_accel = FISKER_ACCEL_MIN,
  .inactive_accel = FISKER_ACCEL_INACTIVE,
};


static bool fisker_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  // Steering angle command (0x1D0). 0x1D0 carries no explicit enable bit, so the
  // steer-active state is derived from (controls_allowed || controls_allowed_lateral) —
  // openpilot only transmits this frame when it intends to steer, and the MADS gate
  // opens the lateral path even without cruise engaged.
  if (msg->addr == 0x1D0U) {
    int raw_angle = (msg->data[2] << 8) | msg->data[3];
    int desired_angle = raw_angle - FISKER_ANGLE_ZERO_CAN;

    if (steer_angle_cmd_checks(desired_angle, controls_allowed || controls_allowed_lateral, FISKER_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // Lateral-control activation (0x1C0). Angle actuation is gated by 0x1D0's checks; also
  // only permit declaring the lateral request ACTIVE while lateral is authorized (either
  // by cruise or by MADS).
  // ADAS_LatCtrl_Req @ 31|2 -> data[3] bits 7..6 (0=Not_active, 1=Angle_request_active).
  if (msg->addr == 0x1C0U) {
    int lat_req = (msg->data[3] >> 6) & 0x03U;
    if ((lat_req != 0) && !(controls_allowed || controls_allowed_lateral)) {
      tx = false;
    }
  }

  // Accel command (0x121)
  if (msg->addr == 0x121U) {
    int raw_accel = (msg->data[2] << 8) | msg->data[3];

    if (fisker_longitudinal) {
      if (longitudinal_accel_checks(raw_accel, FISKER_LONG_LIMITS)) {
        tx = false;
      }
    } else {
      // Not controlling longitudinal: only the inactive value may be sent.
      if (raw_accel != FISKER_ACCEL_INACTIVE) {
        tx = false;
      }
    }
  }

  // Relay arbiter: our 0x1D0/0x1C0 (and 0x121 with the long flag) must continue the stock counter
  // sequence; a frame that would repeat, step back or jump is rejected here, so the vehicle bus
  // can never see our counter go wrong. Only checked once the signal-level checks above passed.
  if (tx) {
    fisker_relay_t *relay = fisker_relay_get(msg->addr);
    if ((relay != NULL) && ((msg->addr != 0x121U) || fisker_longitudinal)) {
      tx = fisker_relay_accept_ours(relay, fisker_get_counter(msg));
    }
  }

  // Release request: the Comma is handing these relays back (byte 0 = bitmask of release bits). It
  // is consumed here and rejected, so it never reaches the car.
  if (msg->addr == FISKER_RELAY_CTRL_ADDR) {
    for (int i = 0; i < FISKER_NUM_RELAYS; i++) {
      if (((msg->data[0] & fisker_relays[i].release_bit) != 0U) && (fisker_relays[i].state == FISKER_RELAY_TAKEN)) {
        fisker_relays[i].state = FISKER_RELAY_RELEASING;
      }
    }
    tx = false;
  }

  // ICC feature settings (0x52A, bus 2): remember when openpilot last sent one so the fwd hook
  // knows whether to keep blocking the ICC's original.
  if ((msg->addr == 0x52AU) && (msg->bus == 2U)) {
    fisker_icc_settings_tx_last = microsecond_timer_get();
  }
  if ((msg->addr == 0x35BU) && (msg->bus == 2U)) {
    fisker_icc_0x35b_tx_seen = true;
    fisker_icc_0x35b_tx_last = microsecond_timer_get();
  }

  return tx;
}


static safety_config fisker_init(uint16_t param) {
  // Forwarding intercept: the OEM ADAS module stays alive on bus 2 and its status/HUD
  // frames are forwarded to the car, so openpilot only injects the command it replaces.
  // disable_static_blocking lets fisker_fwd_hook forward the OEM's 0x1D0 when disengaged
  // and block it only while openpilot steers; check_relay still guards against the OEM's
  // steering leaking onto bus 0.
  // 0x52A/0x35B go to the ADAS module on bus 2. No check_relay: a relay malfunction stops ALL
  // forwarding, and 0x52A seen on bus 2 isn't reliable evidence of one. fisker_fwd_hook blocks
  // the ICC's originals instead.
  static const CanMsg FISKER_TX_MSGS[] = {
    {0x1D0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // steering angle
    {0x1C0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // lateral activation
    {FISKER_RELAY_CTRL_ADDR, 0, 8, .check_relay = false},                   // relay release request (never sent)
    {0x52A, 2, 8, .check_relay = false},                                    // ICC feature settings
    {0x35B, 2, 8, .check_relay = false},                                    // ICC SVS/BSD/APA settings
  };
  static const CanMsg FISKER_LONG_TX_MSGS[] = {
    {0x1D0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // steering angle
    {0x1C0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // lateral activation
    {0x121, 0, 8, .check_relay = true, .disable_static_blocking = true},    // accel (op long only)
    {FISKER_RELAY_CTRL_ADDR, 0, 8, .check_relay = false},                   // relay release request (never sent)
    {0x52A, 2, 8, .check_relay = false},                                    // ICC feature settings
    {0x35B, 2, 8, .check_relay = false},                                    // ICC SVS/BSD/APA settings
  };

  static RxCheck fisker_rx_checks[] = {
    {.msg = {{0x115, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // wheel speeds
    {.msg = {{0x318, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // speed + brake
    {.msg = {{0x1C2, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // steering angle
    {.msg = {{0x1C4, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // driver torque
    {.msg = {{0x313, 2, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // ADAS ACC state
    // Stock steering/long commands on bus 2, for the relay arbiter's stock counters.
    {.msg = {{0x1D0, 2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // stock steering angle
    {.msg = {{0x1C0, 2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // stock lateral activation
    {.msg = {{0x121, 2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // stock accel
    // MFS_0x514 (MFSS buttons) whitelisted so our fisker_rx_hook actually runs on it —
    // the safety framework gates rx_hook execution on rx_checks membership. Without this,
    // our MFS_RiBtnSouth read in the rx hook was dead code and mads_button_press never
    // toggled, so panda's controls_allowed_lateral never opened on MADS press.
    {.msg = {{0x514, 0, 8,  20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // MFSS buttons (MADS trigger)
  };

  // Fisker on-vehicle bring-up: honor the LONG_CONTROL bit on release too. Stock openpilot
  // hides longitudinal behind ALLOW_DEBUG for release builds, but this is a personal test
  // fork and we need the flag to work so alpha_long can gate 0x121 in this
  // firmware (release-tizi panda is not built with ALLOW_DEBUG). openpilot still gates
  // openpilotLongitudinalControl behind AlphaLongitudinalEnabled, so this is only reachable
  // by an explicit developer opt-in.
  const uint16_t FISKER_FLAG_LONGITUDINAL_CONTROL = 1;
  fisker_longitudinal = GET_FLAG(param, FISKER_FLAG_LONGITUDINAL_CONTROL);

  fisker_relays_reset();
  fisker_icc_settings_tx_last = microsecond_timer_get();
  fisker_icc_0x35b_tx_seen = false;
  fisker_icc_0x35b_tx_last = 0U;

  // cppcheck-suppress knownConditionTrueFalse
  return fisker_longitudinal ? BUILD_SAFETY_CFG(fisker_rx_checks, FISKER_LONG_TX_MSGS)
                             : BUILD_SAFETY_CFG(fisker_rx_checks, FISKER_TX_MSGS);
}

static bool fisker_fwd_hook(int bus_num, int addr) {
  // Forwarding intercept: keep the OEM ADAS module (bus 2) alive by forwarding all
  // traffic between it and the vehicle (bus 0), so the car stays fault-free. Only steal
  // the steering command — block the OEM's 0x1D0 (bus 2 -> vehicle) while openpilot is
  // actively steering; openpilot injects its own on bus 0. When disengaged the OEM's
  // 0x1D0 is forwarded so the EPS keeps receiving a steering frame.
  bool block_msg = false;
  // ICC feature settings (0x52A, ICC -> ADAS module): openpilot sends its rewritten copy on
  // bus 2, so drop the ICC's original — but only while openpilot's copies keep arriving.
  if ((bus_num == 0) && (addr == 0x52A)) {
    block_msg = fisker_icc_relay_active(fisker_icc_settings_tx_last);
  }
  if ((bus_num == 0) && (addr == 0x35B)) {
    block_msg = fisker_icc_0x35b_tx_seen && fisker_icc_relay_active(fisker_icc_0x35b_tx_last);
  }
  if (bus_num == 2) {
    // Steering angle (0x1D0), lateral activation (0x1C0) and, with the long flag, accel (0x121):
    // the relay arbiter blocks the stock frame only while openpilot's copies are actually flowing
    // (see the arbiter above). The old rule keyed on controls_allowed_lateral, which only drops on a
    // 1 Hz heartbeat mismatch, so it kept blocking the stock stream for 1-3 s after openpilot had
    // stopped sending and starved the EPS.
    fisker_relay_t *relay = fisker_relay_get((uint32_t)addr);
    if ((relay != NULL) && ((addr != 0x121) || fisker_longitudinal)) {
      block_msg = fisker_relay_blocks_stock(relay);
    }
  }
  return block_msg;
}

const safety_hooks fisker_hooks = {
  .init = fisker_init,
  .rx = fisker_rx_hook,
  .tx = fisker_tx_hook,
  .fwd = fisker_fwd_hook,
  .get_counter = fisker_get_counter,
};
