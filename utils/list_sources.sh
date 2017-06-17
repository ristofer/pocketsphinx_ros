#!/bin/sh
# pacmd list-sources | grep -e 'alsa_input'
pacmd list-sources | grep -e 'alsa_input' | cut -d'<' -f2 | cut -d'>' -f1
