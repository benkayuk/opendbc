"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class FiskerFlagsSP(IntFlag):
  # Flags are stored in whichever polarity makes "no flag set" mean "default
  # baseline behaviour", so a brand-new install with no params present gets the
  # same wire output the port shipped with. Each flag is only set when the user
  # has explicitly diverged from that default via the sunnypilot UI toggle.
  ACC_AUTO_SPEED_OFF = 1    # FiskerACCAutoSpeed param (default '1' / on)  → flag OFF
  ACC_TERRAIN_ON = 2        # FiskerACCTerrain  param (default '0' / off) → flag OFF
  LAT_CTRL_LCA = 4          # FiskerLateralType param (default '0' / LKA) → flag OFF
  VCU_BASIC_CRUISE = 8      # FiskerBasicCruise param (default '0' / off) → flag OFF


class FiskerSafetyFlagsSP:
  # CP_SP.safetyParam bits, read by opendbc/safety/modes/fisker.h from current_safety_param_sp
  VCU_BASIC_CRUISE = 1      # FISKER_PARAM_SP_VCU_BASIC_CRUISE: VCU_0x358 basic cruise engages too
