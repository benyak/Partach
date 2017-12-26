#!/bin/bash
echo Latitude,Longitude,Icon
grep Waypoint: $1 | awk -v f=2 -v t=3 '{for(i=f;i<=t;i++) printf("%s,117\n",$i);}'