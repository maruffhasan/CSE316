AVR_MCU="atmega32"
AVR_PROG="usbasp"

avrbuild() {
  if [ $# -lt 1 ]; then
    echo "Usage: avrbuild <file.c> [file2.c ...]"
    return 1
  fi

  local src="$1"
  local name="${src##*/}"
  name="${name%.c}"

  mkdir -p out

  avr-gcc \
    -mmcu="$AVR_MCU" \
    -Os \
    -Wall \
    "$@" \
    -lm \
    -o "$name.elf" || return 1

  avr-objcopy \
    -O ihex \
    -R .eeprom \
    "$name.elf" \
    "$name.hex" || return 1

  rm -f "$name.elf"

  echo "✓ Built $name.hex"
}

avrbuild $@
