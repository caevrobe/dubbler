#pragma once
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

//*/

/*
 *  File: effect.h
 *
 *  Dummy generic effect template instance.
 *
 */

#include <atomic>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstdlib>


#include "unit_genericfx.h" // Note: Include base definitions for genericfx units

#include "utils/buffer_ops.h" // for buf_clr_f32()
#include "utils/int_math.h"   // for clipminmaxi32()
#include "fx_api.h"



const float SAMPLE_RATE = 48000.0;
const float SAMPLE_RATE_KHZ = SAMPLE_RATE / 1000.0;

//constexpr double PI = 3.141592653589793;



// this circular buffer assumes reader(s) keep track of their own index
// and readers will never be caught up to by the write head
// it's also assumed that the actual buffer creation and management is handled outside this class
//   (5.46 seconds at 48kHz with 0x80000 buffer length)
class LRMultiReadCircularBuffer {
   public:
      int write_idx = 0;

      LRMultiReadCircularBuffer() {}

      void Init(float *buffer, int len) {
         this->buffer = buffer;
         this->len = len;
      } 

      // assumes buffer length is multiple of two
      // and that buffer readers (read heads) will never be lapped by the write head
      void write(float val1, float val2) {
         buffer[write_idx] = val1;
         buffer[write_idx+1] = val2; // TODO here mod len?
         
         write_idx += 2;
         write_idx %= len;
      }

      // read at index (even #)
      f32pair_t read(int idx) {
         return f32pair_t{buffer[idx], buffer[idx+1]};
      }

   private:
      float *buffer;
      int len;
}; 


class DelayLine {
   public:
      DelayLine() {}

      void Init(float msDelay, LRMultiReadCircularBuffer *buffer, int buffer_len) {
         this->msDelay = msDelay;
         this->buffer = buffer;
         this->buffer_len = buffer_len;

         setDelay(msDelay);
      }

      void setDelay(float ms) {
         this->sampleDelay = ms * SAMPLE_RATE_KHZ;
         this->readHeadPos = this->buffer->write_idx - sampleDelay;
      }

      void setSpeed(float speed) {
         this->readHeadSpeed = speed;
      }

      f32pair_t Process() {
         float tempReadHeadPos = readHeadPos;

         readHeadPos+=readHeadSpeed;
         readHeadPos = fmod(readHeadPos, buffer_len/2);

         // effect just initialized, buffer not full enough for this delay yet
         if (tempReadHeadPos < 0.0)
            return f32pair_t{0, 0};

         // return LR sample at index if integer
         int floored = floorf(tempReadHeadPos);
         if (floored==tempReadHeadPos)
            return this->buffer->read(tempReadHeadPos*2);


         // get neighboring samples for fractional index
         float diff = tempReadHeadPos - floored;
         f32pair_t before = this->buffer->read(floored*2);
         f32pair_t after = this->buffer->read(((floored+1)*2)%buffer_len);

         // linear interpolation between samples
         float l = std::lerp(before.a, after.a, diff);
         float r = std::lerp(before.b, after.b, diff);

         return f32pair_t{l, r};
      }



   private:
      float msDelay;
      LRMultiReadCircularBuffer *buffer;
      int buffer_len;
      float sampleDelay;

      // buffer is two channel stored in 1d, this pos is index of 2D LR sample!!!!!!!!!!!!
      float readHeadPos;
      float readHeadSpeed = 1.0;
};

class Oscillator {
   public:
      Oscillator() {}

      Oscillator(float freqHz) : freq(freq) {
         updatePhaseIncrement();
      }

      void setFrequency(float hz) {
         freq = hz;
         updatePhaseIncrement();
      }

      void setPhase(float phase) {
         phase = phase;
      }

      float next() {
         float sample = std::sin(phase);
         phase += phaseIncrement;

         if (phase >= 2.0f * PI) {
            phase = std::fmod(phase, 2.0f * PI);
         }
         
         return sample;
      }
   
   private:
      float freq;
      float phase = 0;
      float phaseIncrement = 0;
      
      void updatePhaseIncrement() {
         phaseIncrement = (2.0f * PI * freq) / SAMPLE_RATE;
      }
};




class Effect {
   public:
   /*===========================================================================*/
   /* Public Data Structures/Types/Enums. */
   /*===========================================================================*/

   enum {
      BUFFER_LENGTH = 0x80000
   };

   enum {
      NUM_COPIES = 0U,
      LFO_SCALE,
      MIX,
      MIN_DEL_MS,
      MAX_DEL_MS,
      MIN_LFO_HZ,
      MAX_LFO_HZ,
      NUM_PARAMS
   };

   // Note: Make sure that default param values correspond to declarations in header.c
   struct Params {
      float numCopies{1};
      float lfoScale{0.3};
      float mix{0.5f}; // 0.0 - 1.0
      float minDelayMs{33.0};
      float maxDelayMs{750.0};
      float minLFOHz{0.05};
      float maxLFOHz{0.3};

      void reset() {
         numCopies = 1;
         lfoScale = 0.3;
         mix = 0.5f;
         minDelayMs = 33.0;
         maxDelayMs = 750.0;
         minLFOHz = 0.05;
         maxLFOHz = 0.3;
      }
   };

   LRMultiReadCircularBuffer buffer;
   DelayLine dls[6];
   Oscillator lfos[6];

   /*===========================================================================*/
   /* Lifecycle Methods. */
   /*===========================================================================*/

   Effect(void) : buffer() {}
   ~Effect(void) {} // Note: will never actually be called for statically allocated instances

   inline int8_t Init(const unit_runtime_desc_t *desc) {
      if (!desc)
         return k_unit_err_undef;

      // Note: make sure the unit is being loaded to the correct platform/module target
      if (desc->target != unit_header.common.target)
         return k_unit_err_target;

      // Note: check API compatibility with the one this unit was built against
      if (!UNIT_API_IS_COMPAT(desc->api))
         return k_unit_err_api_version;

      // Check compatibility of samplerate with unit, for NTS-3 kaoss pad kit should be 48000
      if (desc->samplerate != 48000)
         return k_unit_err_samplerate;

      // Check compatibility of frame geometry
      if (desc->input_channels != 2 || desc->output_channels != 2) // should be stereo input/output
         return k_unit_err_geometry;

      // If SDRAM buffers are required they must be allocated here
      if (!desc->hooks.sdram_alloc)
         return k_unit_err_memory;
      float *m = (float *)desc->hooks.sdram_alloc(BUFFER_LENGTH * sizeof(float));
      if (!m)
         return k_unit_err_memory;

      // Make sure memory is cleared
      buf_clr_f32(m, BUFFER_LENGTH);



      allocated_buffer_ = m;

      // Cache the runtime descriptor for later use
      runtime_desc_ = *desc;

      // Make sure parameters are reset to default values
      params_.reset();

      // todo calculate delay range then evenly space delay times between ACTIVE delaylines

      // [mindel, maxdel]
      // numcopies
      // min 100 max 762 num 3
      // (762-100)/(num-1) = 331
      // 100 441 77

      for (int x = 0; x < 6; x++) {
         dls[x].Init(60*x+1, &buffer, BUFFER_LENGTH);
         lfos[x].setFrequency(params_.minLFOHz);
      }

      //setDelays(params_.minDelayMs, params_.maxDelayMs, params_.numCopies);

      buffer.Init(m, BUFFER_LENGTH);

      fx_rand();

      return k_unit_err_none;
   }

   inline void setDelays(float min, float max, float num) {
      float increment = (max-min)/(num-1.0);

      for (float x = 0; x < 6; x++) {
         dls[(int)x].setDelay(min+increment*x);
      }
   }

   inline void Teardown() {
      // Note: buffers allocated via sdram_alloc are automatically freed after unit teardown
      // Note: cleanup and release resources if any
      allocated_buffer_ = nullptr;
   }

   inline void Reset() {
      // Note: Reset effect state, excluding exposed parameter values.
   }

   inline void Resume() {
      // Note: Effect will resume and exit suspend state. Usually means the synth
      // was selected and the render callback will be called again

      // Note: If it is required to clear large memory buffers, consider setting a flag
      //       and trigger an asynchronous progressive clear on the audio thread (Process() handler)
   }

   inline void Suspend() {
      // Note: Effect will enter suspend state. Usually means another effect was
      // selected and thus the render callback will not be called
   }

   /*===========================================================================*/
   /* Other Public Methods. */
   /*===========================================================================*/

   f32pair_t hii;
   f32pair_t hii2;

   float in_l;
   float in_r;
   float del_l;
   float del_r;
   float out_l;
   float out_r;
   f32pair_t sample;

   fast_inline void Process(const float *in, float *out, size_t frames) {
      const float *__restrict in_p = in;
      float *__restrict out_p = out;
      const float *out_e = out_p + (frames << 1); // assuming stereo output

      // Caching current parameter values. Consider interpolating sensitive parameters.
      const Params p = params_;

      for (; out_p != out_e; in_p += 2, out_p += 2) {
         in_l = in_p[0];
         in_r = in_p[1];
         del_l = 0;
         del_r = 0;

         for (int x = 0; x < p.numCopies; x++) {
            sample = dls[x].Process();

            del_l += sample.a;
            del_r += sample.b;
            
            //del_l += dls[x].Process();
            //del_r += dls[x].Process();

            
            dls[x].setSpeed(1.0 + lfos[x].next()*p.lfoScale*0.5);
         }

         out_l = ((1-p.mix) * in_l) + (p.mix * del_l / p.numCopies);
         out_r = ((1-p.mix) * in_r) + (p.mix * del_r / p.numCopies);

         // todo add feedback
         buffer.write(in_l, in_r);

         out_p[0] = out_l;
         out_p[1] = out_r;

         // hii = delayLine1.Process();
         // hii2 = delayLine2.Process();

         // delayLine1.setSpeed(1.0 + lfo2.next()*0.02);
         // delayLine2.setSpeed(1.0 + lfo3.next()*0.04);
      
         // out_p[0] = hii.a * 0.3 + hii2.a * 0.3 + in_l * 0.4;
         // out_p[1] = hii.b * 0.3 + hii2.b * 0.3 + in_r * 0.4;

         // buffer.write(in_l, in_r);
      }
   }

   inline void setParameter(uint8_t index, int32_t value) {
      switch (index) {
         case NUM_COPIES:
            value = clipminmaxi32(1, value, 6);
            params_.numCopies = value;
            setDelays(params_.minDelayMs, params_.maxDelayMs, params_.numCopies);
            break;

         case LFO_SCALE:
            value = clipminmaxi32(0, value, 2000);
            params_.lfoScale = value/2000.0;
            break;

         case MIX:
            // Single digit base-10 fractional value, bipolar dry/wet
            value = clipminmaxi32(-1000, value, 1000);
            params_.mix = (value + 1000.0)/2000.0; // [-1000, 1000] to [0.0, 1.0]
            break;

         case MIN_DEL_MS:
            value = clipminmaxi32(0, value, 32767);
            params_.minDelayMs = value / 30.0;
            setDelays(params_.minDelayMs, params_.maxDelayMs, params_.numCopies);
            break;

         case MAX_DEL_MS:
            value = clipminmaxi32(0, value, 7000);
            params_.maxDelayMs = value;
            setDelays(params_.minDelayMs, params_.maxDelayMs, params_.numCopies);
            break;

         case MIN_LFO_HZ:
            value = clipminmaxi32(0, value, 5000);
            params_.minLFOHz = value / 20.0;
            break;

         case MAX_LFO_HZ:
            value = clipminmaxi32(0, value, 5000);
            params_.maxLFOHz = value / 20.0;
            break;

         default:
            break;
      }
   }

   inline int32_t getParameterValue(uint8_t index) const {
      switch (index) {
         case NUM_COPIES:
            return params_.numCopies;
            break;

         case LFO_SCALE:
            return params_.lfoScale * 2000.0;
            break;

         case MIX:
            return (params_.mix * 2.0 - 1.0) * 1000.0; // [0.0, 1.0] to [-1000, 1000]
            break;

         case MIN_DEL_MS:
            return params_.minDelayMs * 30.0;
            break;

         case MAX_DEL_MS:
            return params_.maxDelayMs;
            break;

         case MIN_LFO_HZ:
            return params_.minLFOHz * 20.0;
            break;

         case MAX_LFO_HZ:
            return params_.maxLFOHz * 20.0;
            break;

         default:
            break;
      }

      return INT_MIN; // Note: will be handled as invalid
   }

   inline const char *getParameterStrValue(uint8_t index, int32_t value) const {
      // Note: String memory must be accessible even after function returned.
      //       It can be assumed that caller will have copied or used the string
      //       before the next call to getParameterStrValue

      /* static const char *param4_strings[NUM_PARAM4_VALUES] = {
          "VAL 0",
          "VAL 1",
          "VAL 2",
          "VAL 3",
      };

      switch (index) {
      case PARAM4:
         if (value >= PARAM4_VALUE0 && value < NUM_PARAM4_VALUES)
            return param4_strings[value];
         break;
      default:
         break;
      } */

      return nullptr;
   }

   inline void setTempo(uint32_t tempo) {
      // const float bpmf = (tempo >> 16) + (tempo & 0xFFFF) / static_cast<float>(0x10000);
      (void)tempo;
   }

   inline void tempo4ppqnTick(uint32_t counter) {
      (void)counter;
   }

   inline void touchEvent(uint8_t id, uint8_t phase, uint32_t x, uint32_t y) {
      // Note: Touch x/y events are already mapped to specific parameters so there is usually there no need to set parameters from here.
      //       Audio source type effects, for instance, may require these events to trigger enveloppes and such.

      (void)id;
      (void)phase;
      (void)x;
      (void)y;

      // switch (phase) {
      // case k_unit_touch_phase_began:
      //   break;
      // case k_unit_touch_phase_moved:
      //   break;
      // case k_unit_touch_phase_ended:
      //   break;
      // case k_unit_touch_phase_stationary:
      //   break;
      // case k_unit_touch_phase_cancelled:
      //   break;
      // default:
      //   break;
      // }
   }

   /*===========================================================================*/
   /* Static Members. */
   /*===========================================================================*/

   private:
   /*===========================================================================*/
   /* Private Member Variables. */
   /*===========================================================================*/

   std::atomic_uint_fast32_t flags_;

   unit_runtime_desc_t runtime_desc_;

   Params params_;

   float *allocated_buffer_;

   /*===========================================================================*/
   /* Private Methods. */
   /*===========================================================================*/

   /*===========================================================================*/
   /* Constants. */
   /*===========================================================================*/
};
