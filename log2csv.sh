#!/bin/bash
logname = $1
echo Latitude,Longitude,Icon,LineStringColor

grep Tracking: $logname | awk -v f=2 -v t=4 '{for(i=f;i<=t;i++)if(i==t) printf("%s",($i==0)?"none,lime\n":"111,lime\n");else printf("%s,",$i)}'  