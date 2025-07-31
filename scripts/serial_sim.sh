#!/bin/bash
socat -d -d pty,raw,echo=0,link=/tmp/port1 pty,raw,echo=0,link=/tmp/port2