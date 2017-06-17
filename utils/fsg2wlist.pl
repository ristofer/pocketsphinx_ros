#!/usr/bin/perl -w
#
# It builds an .words file from a .fsg file.
# This script is a tool used by create_dictionary.bash 
#
use strict;

while (<>) {
    chomp;
    if (/^TRANSITION \S+ \S+ \S+ (\S+)$/) {
	print "$1\n";
    }
}
