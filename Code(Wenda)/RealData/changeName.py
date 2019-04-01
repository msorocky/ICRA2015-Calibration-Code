#!/usr/bin/env python
# coding=utf-8

import os
import sys
path ='/home/william/Desktop/Courses/Mobile Robots/Project Code/Real Data/velodyne_points/D1'
i=0
maxlen=10
for filename in os.listdir(path):
    
    os.rename(os.path.join(path, filename), os.path.join(path, '0' + str(filename) ))
    i=i+1







