#!/bin/bash

NODE_NO=3
NODE_SSH_REMOTE_PORT=`expr 50000 + $NODE_NO`
NODE_VNC_REMOTE_PORT=`expr 50020 + $NODE_NO`

if [ ! -z "`nmcli g | grep "(local only)"`" ]; then
    nmcli c up PAL3.0
fi
# initial connection
# runs ssh connection in the background.
# Forwarding localhost:22 to 134.209.53.144:50000+NODE_NUM
ssh -fNT -R $NODE_SSH_REMOTE_PORT:localhost:22 root@134.209.53.144
echo "Mapped local SSH port to 134.209.53.144:$NODE_SSH_REMOTE_PORT."
ssh -fNT -R $NODE_VNC_REMOTE_PORT:localhost:5900 root@134.209.53.144
echo "Mapped local VNC port to 134.209.53.144:$NODE_VNC_REMOTE_PORT."

while true; do
    # if not connected to PAL3.0
    if [ ! -z "`nmcli g | grep "(local only)"`" ]; then
    # connects to PAL3.0
        nmcli c up PAL3.0
        # fix broken pipe
        # runs ssh connection in the background.
        # Forwarding localhost:22 to 134.209.53.144:50003
        ssh -fNT -R $NODE_SSH_REMOTE_PORT:localhost:22 root@134.209.53.144
        echo "Mapped local SSH port to 134.209.53.144:$NODE_SSH_REMOTE_PORT."
        ssh -fNT -R $NODE_VNC_REMOTE_PORT:localhost:5901 root@134.209.53.144
        echo "Mapped local VNC port to 134.209.53.144:$NODE_VNC_REMOTE_PORT."
    fi
    sleep 20
# put to background
done &

