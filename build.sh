#!/usr/bin/env bash
# Build AND flash in one go.
#   ./build.sh atmega  atmega.c      -> compile for ATmega32 + flash
#   ./build.sh arduino arduino.ino   -> compile for Arduino  + flash
#
# Needs: arduino-cli, avr-gcc, avr-libc, avrdude
#   sudo apt install gcc-avr avr-libc avrdude

set -euo pipefail

# ============================ SETTINGS ============================
# --- Arduino ---
ARDUINO_FQBN="arduino:avr:uno"     # nano: arduino:avr:nano  (old clones: arduino:avr:nano:cpu=atmega328old)
ARDUINO_PORT="/dev/ttyUSB0"        # or /dev/ttyACM0 ; find with: arduino-cli board list

# --- ATmega32 ---
MCU="atmega32"
AVRDUDE_PART="m32"
F_CPU="1000000UL"
PROGRAMMER="usbasp"                # usbasp | arduino (Arduino as ISP) | avrispmkII ...
ISP_PORT=""                        # e.g. /dev/ttyACM0 when using Arduino as ISP
ISP_BAUD=""                        # Arduino as ISP usually 19200
# ==================================================================

say()  { printf '\033[1;36m==> %s\033[0m\n' "$*"; }
die()  { echo "Error: $*" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || die "missing tool: $1"; }
usage() {
  echo "Usage:"
  echo "  $0 atmega  <file.c>     build + flash ATmega32"
  echo "  $0 arduino <file.ino>   build + flash Arduino"
  exit 1
}

[ $# -eq 2 ] || usage
CHIP="$1"; FILE="$2"
[ -f "$FILE" ] || die "file not found: $FILE"

FILE="$(cd "$(dirname "$FILE")" && pwd)/$(basename "$FILE")"
DIR="$(dirname "$FILE")"
NAME="$(basename "$FILE")"; BASE="${NAME%.*}"
HEX="$DIR/$BASE.hex"

do_atmega() {
  need avr-gcc; need avr-objcopy; need avrdude
  say "Compiling $NAME for $MCU @ $F_CPU"
  local elf; elf="$(mktemp --suffix=.elf)"
  avr-gcc -mmcu="$MCU" -DF_CPU="$F_CPU" -Os -std=gnu99 -Wall \
          -ffunction-sections -fdata-sections -Wl,--gc-sections \
          -o "$elf" "$FILE"
  avr-objcopy -O ihex -R .eeprom "$elf" "$HEX"
  avr-size --mcu="$MCU" -C "$elf" 2>/dev/null || true
  rm -f "$elf"
  say "Created $HEX"

  local args=(-c "$PROGRAMMER" -p "$AVRDUDE_PART")
  [ -n "$ISP_PORT" ] && args+=(-P "$ISP_PORT")
  [ -n "$ISP_BAUD" ] && args+=(-b "$ISP_BAUD")
  say "Flashing $BASE.hex to $MCU via $PROGRAMMER"
  avrdude "${args[@]}" -U flash:w:"$HEX":i
}

do_arduino() {
  need arduino-cli
  say "Compiling $NAME for $ARDUINO_FQBN"
  # temp sketch folder so other files (like atmega.c) are not pulled in
  local tmp; tmp="$(mktemp -d)"
  mkdir -p "$tmp/$BASE"
  cp "$FILE" "$tmp/$BASE/$BASE.ino"
  arduino-cli compile --fqbn "$ARDUINO_FQBN" --output-dir "$tmp/out" "$tmp/$BASE"
  cp "$tmp/out/$BASE.ino.hex" "$HEX"
  rm -rf "$tmp"
  say "Created $HEX"

  say "Uploading $BASE.hex to $ARDUINO_PORT"
  avrdude -v -p atmega328p -c arduino -P /dev/ttyACM0 -b 115200 -D -U flash:w:"$HEX":i
  }

case "$CHIP" in
  atmega)  do_atmega ;;
  arduino) do_arduino ;;
  *) usage ;;
esac

say "Done"
