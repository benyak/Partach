#!/bin/bash
echo Latitude,Longitude,Icon
grep Waypoint: $1 | awk -v f=3 -v t=5 '{for(i=f;i<=t;i++)if(i==t) printf("%s","117\n");else printf("%s,",$i)}'
