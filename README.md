# VDP33 — vocoder / strings / human-voice ensemble for Schwung

A sound-generator module for [Schwung](https://github.com/charlesvestal/schwung)
(Ableton Move), laid out like the **Behringer VC340** (a clone of the Roland
VP-330 "Vocoder Plus"): a paraphonic divide-down keyboard feeding **Strings**,
**Human Voice** and a **10-band Vocoder**, with Ensemble, Vibrato and Pitch Shift.

Everything is written from scratch. There is no code, sample or ROM from any
original instrument. All DSP is in one C file, `src/dsp/vdp33.c`.

## Panel map

The controls follow the VC340 Quick Start Guide (numbers are the guide's).

| VC340 section | VDP33 parameters |
|---|---|
| **Balance** (6–8): Direct Mic, Human Voice, Strings. There is no vocoder fader | Direct Mic, Human Voice Level, Strings Level, Volume |
| **Vocoder** (12–16): Tone, Mic Level, Lower, Upper, Ensemble | Vocoder Tone, Mic Level, Vocoder Keys (Off / Lower / Upper / All), Vocoder Ensemble |
| **Human Voice** (17–22): Male 8' Lower, Male 4' Lower, Male 8' Upper, Female 4' Upper, Ensemble, Attack | four on/off switches with the same names, Voice Ensemble, Voice Attack |
| **Strings** (24–27): Attack, Tone, Lower, Upper | Strings Attack, Strings Tone, Strings Keys (Off / Lower / Upper / All) |
| **Release** (23): one fader for all three sections | Release |
| **Vibrato** (9–11): Rate, Delay Time, Depth — acts on Vocoder and Human Voice | Vibrato Rate / Delay / Depth |
| **Pitch Shift** (31–34): Auto / Manual / Off (pedal), Down–Normal, Pitch Set, Time | Pitch Shift Mode, Pitch Down, Pitch Set, Pitch Shift Time |
| Tune (29), keyboard split, velocity (Fixed / Dynamic) | Tune, Split Point, Velocity |
| Footswitches: Vocoder Hold, Pitch Shift | MIDI CC 64 and CC 66 |

Signal flow:

```
 keys ─► oscillator bank S (Strings: tune, bend)
      └► oscillator bank V (Voice + Vocoder: tune, bend, vibrato, pitch shift)
          └─► octave dividers ─► 16' / 8' / 4' registers, phase-locked
               └─► per-key attack/release envelopes

 Strings   ─► Tone low-pass ─────────────────────────────┐ (Ensemble always on)
 Male 8'/4'─► male formants ────┐                        │
 Female 4' ─► female formants ──┴─► Ensemble (switch) ───┼─► out
 Vocoder carrier × 10 bands of mic/line-in ─► Tone ─► Ensemble (switch)
 Direct Mic ─────────────────────────────────────────────┘
```

* **Paraphonic divide-down**: every note of a pitch class comes from one master
  oscillator through octave dividers (32.32 fixed-point phase), so notes are
  phase-locked as on the original. Waveforms are band-limited (PolyBLEP).
* **Vibrato and pitch shift only reach Voice and Vocoder**, as the manual says
  for vibrato; Strings come from their own oscillator bank.
* **Pitch Shift**: in *Auto*, every key press drops the pitch by the Pitch Set
  amount and it glides back up over Time. *Manual* holds a static shift from the
  Pitch Down control. *Off/Pedal* does nothing unless CC 66 is held, then it
  behaves like Auto.
* **Vocoder Hold footswitch**: in the manual it engages / disengages the
  vocoder. Set *Vocoder Hold Pedal* to **Gate** and the vocoder sounds only
  while CC 64 is held; **Off** leaves it always on.
* Attack and Release at 0 mean no delay, as the manual specifies.

## Install

Requirements: a Move with Schwung installed (see the Schwung installer), and
Docker (or an `aarch64-linux-gnu-` toolchain) to build.

```bash
bash scripts/build.sh      # -> dist/vdp33/ and dist/vdp33-module.tar.gz
```

Then install the module in one of two ways:

* **Schwung Manager** (`http://move.local:7700`): *Modules* -> install a custom
  module from a GitHub URL **or a tarball**, and give it
  `dist/vdp33-module.tar.gz`.
* **SSH**: `bash scripts/install.sh` copies `dist/vdp33/` to
  `/data/UserData/schwung/modules/sound_generators/vdp33/` on `move.local`.

Restart the Move afterwards: Schwung may keep running an older copy of a module
it had already loaded.

## Try it on the Move

1. Open a slot: hold `Track 1-4` for half a second (or `Shift+Vol` + `Track 1-4`).
2. Put **VDP33** in the sound-generator position of the chain.
3. Wire the MIDI: on the Move track set **MIDI Out** to a channel 1-4 and make
   sure the slot's **Receive Ch** is the same number (usually automatic). Load an
   empty Drum Rack on that Move track so the native sound does not double.
4. Play the pads. The first preset, *Strings*, needs no microphone.
5. For the vocoder, plug in headphones or a line-in cable, pick the *Vocoder*
   preset and speak. To see the cost of the module on the device use
   Schwung Manager -> *System* -> *Open CPU view* (the frame budget is the
   figure that matters).

> **Vocoder input.** The modulator is the Move audio input, as for Schwung's
> own Line In modules. With the built-in speakers active and no line-in cable,
> Schwung's feedback guard bypasses the slot (a **B** appears) and shows a
> "Feedback Risk" notice. Plug in headphones or a cable and it re-enables on its
> own; `Mute` + jog-click un-bypasses it if you really want it live.

Eight knobs per page. Main page: Volume, Strings, Voice and Direct Mic levels,
Mic Level, Vocoder Tone, Strings Tone, Release. The other pages are Balance /
Tone, Keyboard, Envelope, Ensemble, Vibrato and Pitch Shift. 15 factory presets
cover strings, choirs, vocoder, split layouts and pitch dives.

## What the manual confirms, and what is an estimate

**Confirmed by the VC340 Quick Start Guide:** the section structure above; the
four Human Voice switches and their names; one shared Release; separate Attack
for Human Voice and Strings; Tone on Strings and on Vocoder; vibrato affecting
Vocoder and Human Voice; Ensemble switches on Vocoder and Human Voice; the
three Pitch Shift modes and their controls; velocity selectable between fixed
and dynamic; Direct Mic in the Balance section; split between Lower and Upper;
footswitches for Vocoder Hold and Pitch Shift; 37-key keyboard with an octave
switch (use Octave on the Pitch page).

**Estimates, not measured from hardware (the manual does not give them):**

* centre frequencies and Q of the ten vocoder bands (about half-octave spacing);
* harmonic mix of the 16' / 8' / 4' registers, Tone filter ranges;
* choir formant frequencies (typical male and female /a/ values);
* Ensemble delay times and LFO rates, attack / release / pitch-shift time
  ranges, vibrato depth, the maximum Pitch Set shift (12 semitones), Tune range
  (±1 semitone);
* **Strings always go through the Ensemble** (the manual has no Strings
  Ensemble switch; this follows descriptions of the VP-330);
* **Pitch Shift affects Voice and Vocoder, not Strings** (the manual does not
  say which sections it reaches).

**Additions that are not on the VC340** (defaults keep them neutral):
Sibilance (a noise path for consonants, default off), Ensemble Depth / Speed,
Bend range.

**Not modelled:** the External Synth input (Move has a single input, so it is
the vocoder modulator), the L/M/H output switch, SysEx, analog drift and
component tolerances, BBD companding noise.

If you can measure a unit, the constants worth adjusting first are `k_band_hz`
and `BAND_Q`, the register mix in `vdp_render`, the formant tables and the
ensemble constants; they are isolated and commented.

## Test

```bash
bash scripts/test.sh
```

Builds a native test that renders every preset (WAV files in `tests/demo/`)
with a synthetic vowel sequence as the vocoder modulator, and checks pitch
accuracy, levels, vocoder tracking, the Vocoder Hold gate, Direct Mic, zones and
the four Human Voice switches, male and female formants, vibrato reaching only
Voice and Vocoder, Pitch Shift in its three modes, Tune and bend, velocity
modes, Ensemble level and stereo width, zero Attack / Release, state round-trip,
JSON, and CPU load (about 4 % of one desktop core for 24 keys with everything
on). MP3 renders of every preset are in `demo/`.

## Licence

MIT. See `LICENSE` and `NOTICE`. "VP-330" and "Vocoder Plus" are trademarks of
Roland Corporation; "VC340" of Music Tribe / Behringer. They are used only to
describe the sound this module is modelled on; this project is not affiliated
with either company.
