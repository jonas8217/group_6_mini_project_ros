To load kernel module on startup:

Add "sudo insmod <path>/reservedmemLKM.ko && sudo chmod 666 /dev/reservedmemLKM && lsmod | grep "reservedmemLKM"" to the /etc/rc.local file


Problems:
If there is no etc/rc.local:
sudo nano /etc/rc.local

add below to the file

#!/bin/bash
sudo insmod /etc/reservedmemLKM.ko && sudo chmod 666 /dev/reservedmemLKM && lsmod | grep "reservedmemLKM"
exit 0

If /etc/rc.local us run run on startup:
sudo nano /etc/systemd/system/rc-local.service

add below to the file

[Unit]
 Description=/etc/rc.local Compatibility
 ConditionPathExists=/etc/rc.local

[Service]
 Type=forking
 ExecStart=/etc/rc.local start
 TimeoutSec=0
 StandardOutput=tty
 RemainAfterExit=yes
 SysVStartPriority=99

[Install]
 WantedBy=multi-user.target
