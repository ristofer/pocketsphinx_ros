#!/bin/sh
#
# This file will be automatically loaded when you source the bender.sh file 
# (i.e, open a new terminal window). It is intended for configuration of 
# required shell environment variables. 
#
#
# [Importante]: Sólo usar comandos del lenguaje sh, nada de bash. (comandos bash no sirven al trabajar por red)
#
# - Preferir '.' por sobre 'source' (hacen exactamente lo mismo, pero 'source' es exclusivo de bash)
# - Todos los archivos linkeados acá deben usar comandos válidos de sh y nada exclusivo de bash.

# for pocketsphinx stuff
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig

alias bender_hwcheck_mic="rosrun uchile_speech_pocketsphinx hwcheck_mic.py"