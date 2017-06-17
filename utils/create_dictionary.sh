#!/bin/sh
_jsgf_file="$1"


_file_path="$(rospack find uchile_speech_pocketsphinx)"/Grammar
_utils_path="$(rospack find uchile_speech_pocketsphinx)"/utils

_fullname_noext="$_file_path"/"$_jsgf_file"

sphinx_jsgf2fsg -jsgf "$_fullname_noext".jsgf -fsg "$_fullname_noext".fsg

# fsg   -->   words
perl "$_utils_path"/fsg2wlist.pl<"$_fullname_noext".fsg> "$_fullname_noext".words
#text2wfreq < "$_fullname_noext".txt | wfreq2vocab > "$_fullname_noext".vocab
#sed 's:#.*$::g' "$_fullname_noext".vocab > "$_fullname_noext".words

# delete repeated words
perl -ne 'print unless $seen{$_}++' "$_fullname_noext".words > "$_fullname_noext".word

# convierte las palabras a minusculas
perl -pe '$_= lc($_)' "$_fullname_noext".word > "$_fullname_noext".words


unset _jsgf_file
unset _file_path
unset _utils_path
unset _fullname_noext