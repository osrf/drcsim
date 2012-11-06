#! /bin/bash

rxplot -t "Motor Winding Temp" /handle/sensors/raw/motorWindingTemp[1] &

rxplot -t "Motor Encoder" /handle/sensors/raw/motorHallEncoder[1] &

rxplot -t "Motor Current" /handle/sensors/raw/motorCurrent[1] &

