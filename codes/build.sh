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
    -DF_CPU=16000000UL \
    -Os \
    -Wall \
    "$@" \
    -lm \
    -o "out/$name.elf" || return 1

  avr-objcopy \
    -O ihex \
    -R .eeprom \
    "out/$name.elf" \
    "out/$name.hex" || return 1

  rm -f "out/$name.elf"

  echo "✓ Built out/$name.hex"
}

avrbuild $@
