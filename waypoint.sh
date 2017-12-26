#!/bin/bash
echo Latitude,Longitude,Icon
grep Waypoint: $1 | awk -v f=3 -v t=4 '{for(i=f;i<=t;i++) if(i==t) printf("%s\n",$i);else printf("%s,",$i)}'