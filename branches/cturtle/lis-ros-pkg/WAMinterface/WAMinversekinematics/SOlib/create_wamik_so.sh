gcc -Wall -fno-stack-protector -fPIC -c ../wamik.c ../WAMKinematics.c wamik_interface.c
ld -shared -soname wamik.so.1 -o wamik.so.1.0 -lc *.o
rm *.o
cp wamik.so.1.0 ../../ROS
