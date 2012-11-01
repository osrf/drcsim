#! /bin/bash

rxplot -t "Motor Winding Temp" /handle/sensors/raw/motorWindingTemp[0]:motorWindingTemp[1]:motorWindingTemp[2] &

rxplot -t "Motor Encoder" /handle/sensors/raw/motorHallEncoder[0]:motorHallEncoder[1]:motorHallEncoder[2] &

rxplot -t "Motor Current" /handle/sensors/raw/motorCurrent[0]:motorCurrent[1]:motorCurrent[2] &

