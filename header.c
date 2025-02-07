/*
   BSD 3-Clause License

   Copyright (c) 2023, KORG INC.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

   * Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 *  File: header.c
 *
 *  NTS-3 kaoss pad kit generic effect unit header definition
 *
 */

#include "unit_genericfx.h"   // Note: Include base definitions for genericfx units

// ---- Unit header definition  --------------------------------------------------------------------
 
const __unit_header genericfx_unit_header_t unit_header = {
   .common = {
      .header_size = sizeof(genericfx_unit_header_t),           // Size of this header. Leave as is.
      .target = UNIT_TARGET_PLATFORM | k_unit_module_genericfx, // Target platform and module pair for this unit
      .api = UNIT_API_VERSION,                                  // API version for which unit was built. See runtime.h
      .dev_id = 0x2163326B,                                            // Developer ID. See https://github.com/korginc/logue-sdk/blob/master/developer_ids.md
      .unit_id = 0x0U,                                          // ID for this unit. Scoped within the context of a given dev_id.
      .version = 0x00010000U,                                   // This unit's version: major.minor.patch (major<<16 minor<<8 patch).
      .name = "dubbler",                                          // Name for this unit, will be displayed on device
      .num_params = 7,                                          // Number of valid parameter descriptors. (max. 8)
       
 
      // uint8_t cast for frac and frac mode???
      .params = {
         // Format: min, max, center, default, type, frac. bits, frac. mode, <reserved>, name
         
         // See common/runtime.h for type enum and unit_param_t structure
         {1, 6, 1, 1, k_unit_param_type_none, 0, 0, 0, {"num copies"}},
         // 0-100%, increments of 0.05, default 30% (times 0.5 == (0.85x - 1.15x playback rate))
         {0, 2000, 0, 600, k_unit_param_type_percent, 2, 1, 0, {"lfo scale"}},
         // increment 0.1
         {-1000, 1000, 0, 0, k_unit_param_type_drywet, 1, 1, 0, {"mix"}},
         // frac_mode 1 frac 3 (0 0.033 0.067 0.1) (n/(10*3))
         // default min delay 33 ms, range 0 - 1092ms
         {0, 32767, 0, 990, k_unit_param_type_none, 3, 1, 0, {"min del ms"}},
         // default 750ms range 0 - 4000ms
         //{0, 32767, 0, 22500, k_unit_param_type_none, 3, 1, 0, {"max del ms"}},
         {0, 7000, 0, 750, k_unit_param_type_none, 0, 0, 0, {"max del ms"}},

         // hz: decimal fixed point, frac_mode = 1 and frac = 2
         // https://blog.boochow.com/article/loguesdk2-params.html
         // 0 - 250 HZ
         // default min 0.05hz 20 seconds
         // default max 0.3hz 3.33 seconds
         {0, 200, 0, 1, k_unit_param_type_hertz, 2, 1, 0, {"min lfo hz"}},
         {0, 2000, 0, 6, k_unit_param_type_hertz, 2, 1, 0, {"max lfo hz"}},
 
         {0, 0, 0, 0, k_unit_param_type_none, 0, 0, 0, {""}}
      },
   },
   .default_mappings = {
      // By default, the parameters described above will be mapped to controls as described below.
      // These assignments can be overriden by the user.
      
      // Format: assign, curve, curve polarity, min, max, default value

      /* params:
       *   num copies - y
       *   lfo scale  - x
       *   mix        - depth
       *   min del ms
       *   max del ms
       *   min lfo hz
       *   max lfo hz
       */
      
      {k_genericfx_param_assign_y, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 1, 6, 1},
      {k_genericfx_param_assign_x, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 2000, 600},
      {k_genericfx_param_assign_depth, k_genericfx_curve_exp, k_genericfx_curve_bipolar, -1000, 1000, 0},
      {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 22500, 990},
      {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 22500, 22500},
      {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 100, 1},
      {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 200, 6},

      {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 0, 0}
   }
};
 