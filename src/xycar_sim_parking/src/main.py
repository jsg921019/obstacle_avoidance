#!/usr/bin/env python
import sys
import os
import signal
import start
def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
start.main()