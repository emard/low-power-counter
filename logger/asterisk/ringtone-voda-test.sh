#!/bin/sh
/usr/sbin/asterisk -rx 'console dial *3767@home'
sleep 3
/usr/sbin/asterisk -rx 'console hangup'
