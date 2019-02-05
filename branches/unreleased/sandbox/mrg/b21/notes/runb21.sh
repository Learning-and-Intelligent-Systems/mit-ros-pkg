bot-sick -c SICK_FRONT -z 75 -r 100 -f 180 -d /dev/ttyUSB.FTCY3BKK &
bot-sick -c SICK_BACK -z 75 -r 100 -f 180 -d /dev/ttyUSB.FTEKRTR0 &
/home/ruser/marine/pods/v4l/build/bin/rFlex &
/home/ruser/camunits-pods/build/bin/camlog -n -c /home/ruser/camunits-pods/simplechain &

