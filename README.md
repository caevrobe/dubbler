Modulated delay inspired by Ken Townsend's automatic double tracking technique.
Developed for the NTS-3 with the [logue SDK](https://github.com/korginc/logue-sdk).


### Parameters:
* **num copies (1 - 5)**     - number of delays.
* **min del ms (0 - 3000)**  - minimum delay time in milliseconds.
* **max del ms (0 - 3000)**  - maximum delay time in milliseconds.
* **min lfo hz (0 - 250)**   - minimum LFO speed in hertz.
* **max lfo hz (0 - 250)**   - maximum LFO speed in hertz.
* **lfo scale  (0% - 100%)** - LFO scaling factor, affects the delay playback rate (100% = 0.5x - 1.5x).
* **mix        (0% - 100%)** - dry/wet ratio.
* 



### TODO!!!!
* finish param tweak implementation (lfo frequency etc etc)
* invert LFOs for delay lines with short delay time (don't want to speed up past write head when effect starts), also randomize phases
* fix copies param (silent when set to 1, just clicks (might have to do with playback rate modulation))
* add feedback param
* remove clicking when changing delay time and # parameters (wait for zero crossing)
* correctly handle user delay time changes! need to adjust playback rate until the delay line makes it to the new distance from the write head (preferably) or just jump to it
* fix Korg Kontrol editor param ranges 
* !!!!!!! add boundary handling for play head modulation. flip direction at boundaries