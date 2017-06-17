#! /usr/bin/env python
import sys
from hwcheck import mic_check

if __name__  == '__main__':
    if mic_check.check():
        sys.exit(0)
    sys.exit(-1)

