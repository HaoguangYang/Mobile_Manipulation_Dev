#!/bin/bash

NODE_NO=5
NODE_SSH_REMOTE_PORT=`expr 50000 + $NODE_NO`
NODE_VNC_REMOTE_PORT=`expr 50020 + $NODE_NO`
NODE_HTTP_REMOTE_PORT=`expr 50040 + $NODE_NO`

if [ ! -z "`nmcli g | grep "(local only)"`" ]; then
    nmcli c up URWintek
fi
# initial connection
# runs ssh connection in the background.
# Forwarding localhost:22 to 134.209.53.144:50000+NODE_NUM

SSH_AUTH_SOCK=0 ssh -fNT -R $NODE_SSH_REMOTE_PORT:localhost:22 ubuntu@34.221.135.10
echo "Mapped local SSH port to 34.221.135.10:$NODE_SSH_REMOTE_PORT."
SSH_AUTH_SOCK=0 ssh -fNT -R $NODE_VNC_REMOTE_PORT:localhost:5900 ubuntu@34.221.135.10
echo "Mapped local VNC port to 34.221.135.10:$NODE_VNC_REMOTE_PORT."
SSH_AUTH_SOCK=0 ssh -fNT -R $NODE_HTTP_REMOTE_PORT:localhost:8080 ubuntu@34.221.135.10
echo "Mapped local HTTP port to 34.221.135.10:$NODE_VNC_REMOTE_PORT."

setupdir=`find . -type d -iname "devel*" | head -1`
. $setupdir/setup.sh
screen -dm -S robotAutorun bash -c 'source devel/setup.sh; python ./autorun.py'
#gnome-terminal -x sh -c ". devel/setup.sh; python ./autorun.py"

# in case the network drops offline
while true; do
    # if not connected to PAL3.0
    #nc -v -z -w 3 34.221.135.10 22 &>/dev/null
    #status=$( echo $? ) 
    if [ ! -z "`nmcli g | grep "(local only)"`" ]; then
        #echo $status
        # connects to PAL3.0
        nmcli c down URWintek
        nmcli c up URWintek
        # fix broken pipe
        # runs ssh connection in the background.
        # Forwarding localhost:22 to 134.209.53.144:50003
        sudo kill $(ps aux | grep "ubuntu@34.221.135.10" | awk '{print $2}')
        sudo service ssh restart
        SSH_AUTH_SOCK=0 ssh -fNT -R $NODE_SSH_REMOTE_PORT:localhost:22 ubuntu@34.221.135.10
        echo "Mapped local SSH port to 34.221.135.10:$NODE_SSH_REMOTE_PORT."
        SSH_AUTH_SOCK=0 ssh -fNT -R $NODE_VNC_REMOTE_PORT:localhost:5900 ubuntu@34.221.135.10
        echo "Mapped local VNC port to 34.221.135.10:$NODE_VNC_REMOTE_PORT."
        SSH_AUTH_SOCK=0 ssh -fNT -R $NODE_HTTP_REMOTE_PORT:localhost:8080 ubuntu@34.221.135.10
        echo "Mapped local HTTP port to 34.221.135.10:$NODE_VNC_REMOTE_PORT."
    fi
    sleep 20
# put to background
done &

