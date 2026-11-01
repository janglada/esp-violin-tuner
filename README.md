# esp-violin-tuner

A violin tuner for the ESP8266. It listens through a microphone module,
detects the pitch with the YIN algorithm and shows it on a 2.4" TFT: the note
in solfege (Do, Re, Mi, Fa, Sol, La, Si), a cents meter and a Catalan hint
telling you which way to turn the peg (`APRETA` / `AFLUIXA`).

## Features

- Pitch detection with YIN and parabolic interpolation, tuned for the violin
  range (150-1100 Hz), median-filtered over three frames.
- Large note name and octave, colored by how close you are: green (within
  5 cents), yellow (within 15) or red.
- Cents meter from -50 to +50 with a moving needle.
- `APRETA >>` (flat, tighten), `<< AFLUIXA` (sharp, loosen) or `AFINAT!`
  (in tune).
- Chips for the four open strings (Sol3, Re4, La4, Mi5), lit when the note
  shown is exactly an open string.
- Microphone level bar in the header.
- The same readings are printed on the Serial Monitor (9600 baud).

## Hardware

| Part | Notes |
|---|---|
| ESP8266 board | NodeMCU or Wemos D1 mini (they have a divider on `A0` for 0-3.3 V) |
| FC-109 microphone | MAX9812 amplifier module |
| TFT 2.4" 240x320 | TJCTM24024, ILI9341 controller, SPI |

### Wiring

| Module pin | ESP8266 pin |
|---|---|
| FC-109 VCC | 3V3 |
| FC-109 GND | GND |
| FC-109 OUT | A0 |
| TFT VCC | 3V3 |
| TFT GND | GND |
| TFT LED | 3V3 |
| TFT SCK | D5 (GPIO14) |
| TFT SDI/MOSI | D7 (GPIO13) |
| TFT CS | D8 (GPIO15) |
| TFT DC/RS | D2 (GPIO4) |
| TFT RESET | D1 (GPIO5) |

SDO/MISO and the touch pins of the TFT are not used. On a bare ESP-12 module
`A0` only accepts 0-1 V, so add a divider first. To use other CS/DC/RESET
pins, change `TFT_CS`, `TFT_DC` and `TFT_RST` at the top of the sketch.

## Build and upload

Install the ESP8266 core and the Adafruit ILI9341 library (it pulls in
Adafruit GFX and BusIO), then compile and upload with `arduino-cli`:

```bash
arduino-cli core install esp8266:esp8266
arduino-cli lib install "Adafruit ILI9341"
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 .
arduino-cli upload --fqbn esp8266:esp8266:nodemcuv2 -p /dev/ttyUSB0 .
```

Adjust the `--fqbn` for your board and `-p` for your serial port. The Arduino
IDE works too.

## How it works

1. WiFi is switched off, because it disturbs the ADC.
2. `captureFrame()` samples 1024 values from `A0` at a fixed rate
   (`TARGET_FS`); the rate actually achieved is used for the analysis.
3. The DC offset is removed and the RMS level is computed. Below
   `SILENCE_RMS` the frame is ignored.
4. `detectPitch()` runs YIN. It stops at the first dip below `YIN_THRESH`; if
   there is none, it falls back to the deepest dip below `YIN_LOOSE`.
5. The frequency is converted to a note and cents, and the screen is updated.
   Only the parts that changed are repainted.
6. After about a second of silence (or sound without a clear pitch) the
   screen returns to its idle state.

## Settings

All are constants at the top of [violin_tuner_esp8266.ino](violin_tuner_esp8266.ino).

| Constant | Meaning |
|---|---|
| `TARGET_FS` | Sample rate in Hz |
| `N` | Samples per frame |
| `MIN_FREQ` / `MAX_FREQ` | Pitch search range in Hz |
| `YIN_THRESH` | Strict threshold (lower = stricter) |
| `YIN_LOOSE` | Fallback threshold for weak or noisy notes |
| `SILENCE_RMS` | Level below which nothing is analyzed |

## Troubleshooting

- **Nothing is detected from a distance.** The signal is probably too weak.
  Move the microphone closer to the instrument or lower `SILENCE_RMS`. The
  FC-109 gain pin also sets the amplification: floating gives the most, GND
  or VCC give less.
- **It reacts to room noise.** Raise `SILENCE_RMS`.
- **Blank or white screen.** Check the wiring, especially DC and RESET, and
  that the module runs from 3.3 V.
- **Wrong octave or unstable reading.** Check the level bar: a clipped or very
  weak signal gives poor results.
