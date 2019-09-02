#!/bin/bash

if [ $1x = "start"x ]; then
  sudo supervisorctl start learm:learm_0
elif [ $1x = "stop"x ]; then
  sudo supervisorctl stop learm:learm_0
else
  sudo supervisorctl restart learm:learm_0
fi
