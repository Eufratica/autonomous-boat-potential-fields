#!/bin/bash
sim_vehicle.py  -D -f rover-skid -S 10 --model gazebo-rover --out=udpout:192.168.0.114:14550 --out=udpout:127.0.0.1:14552 --out=udpout:127.0.0.1:15555 -L SAE_XASV_SIM --console --add-param-file mig_sitl.params
