#!/bin/sh
TARGETDIR=/rpicopter
if [ "$1" != "start" ]; then
	echo "stoping"
	killall raspivid
else
	ts=`date +%s`
	case "$4" in
	0) echo "stopping" 
		killall raspivid
	;; 
	1) raspivid -n -w 640 -h 480 -b 500000 -ex sports -fps 24 -g 60 -t 0 -o - | \
		gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink port=$3 host=$2 &
	;;
	2) raspivid -n -w 640 -h 480 -b 2500000 -ex sports -fps 24 -g 60 -t 0 -o - | \
		gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink port=$3 host=$2 &
	;;
	3) raspivid -n -w 800 -h 600 -b 3000000 -ex sports -fps 24 -g 60 -t 0 -o - | \
		gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink port=$3 host=$2 &
	;;
	4) raspivid -n -w 1280 -h 720 -b 4500000 -ex sports -fps 24 -g 60 -t 0 -o - | \
		gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink port=$3 host=$2 &
	;;
        11) raspivid -n -w 640 -h 480 -b 2500000 -ex sports -fps 24 -g 60 -t 0 -o - | tee $TARGETDIR/VIDEO-$ts.h264 | \
                gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink port=$3 host=$2 &
	;;
        12) raspivid -n -w 640 -h 480 -b 2500000 -ex sports -fps 30 -g 60 -t 0 -o - | tee $TARGETDIR/VIDEO-$ts.h264 | \
                gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink port=$3 host=$2 &
	;;
        13) raspivid -n -w 800 -h 600 -b 3000000 -ex sports -fps 24 -g 60 -t 0 -o - | tee $TARGETDIR/VIDEO-$ts.h264 | \
                gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink port=$3 host=$2 &
	;;
        14) raspivid -n -w 1280 -h 720 -b 4500000 -ex sports -fps 24 -g 60 -t 0 -o - | tee $TARGETDIR/VIDEO-$ts.h264 | \
                gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink port=$3 host=$2 &
	;;
	*) raspivid -n -w 640 -h 480 -b 500000 -ex sports -fps 24 -g 60 -t 0 -o - | \
		gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink port=$3 host=$2 &
	;;
	esac
	echo "starting;"
fi

