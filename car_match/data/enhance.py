import numpy as np
import cv2 as cv
import csv
from random import random
import sys

max_light = 13
shift_param = (-800, 800, 2)
pre_base = max_light * 4        # presence 地址
car_base = max_light * 5        # car 地址
delete_proba = 0.5              # 随机删除概率

tp = np.array([0, 0], dtype = float)
br = np.array([1440, 1080], dtype = float)

def loadFrom(path='transformer.csv'):
    data = []
    with open(path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            buf = [float(elem) for elem in row]
            data.append(buf)
    return np.array(data)

def randomShift(data:np.array):
    shift = data.copy()
    for row in shift:
        sft = np.random.uniform(*shift_param)
        for i in range(max_light):
            if int(row[pre_base + i]) == 0: continue
            center = row[4 * i : 4 * i + 2]
            center += sft
            too_small = center < tp
            too_big = center > br
            if any(too_small) or any(too_big):          # 越界删除
                row[pre_base + i] = 0
    return shift
            
def randomDelete(data:np.array):
    delete = data.copy()
    for row in delete:
        for i in range(max_light):
            if int(row[pre_base + i]) == 0: continue
            if int(row[car_base + i]) > 0: continue
            if random() > delete_proba:
                row[pre_base + i] = 0                   # 随机删除
    return delete

# 左右镜像
def mirroData(data:np.array):
    mirror = data.copy()
    for row in mirror:
        for i in range(max_light):
            if int(row[pre_base + i]) == 0: continue
            row[4 * i] = br[0] - row[4 * i]
            row[4 * i + 2] *= -1
    return mirror

def enhance(data:np.array, opath:str = "./enhanced.csv"):
    shift = randomShift(data)
    mirro = mirroData(data)
    delete = randomDelete(data)
    result = np.vstack((data, shift, mirro, delete))
    with open(opath, 'w') as file:
        writer = csv.writer(file)
        for row in result:
            writer.writerow(row)
    print("Data enchanced.")

def visualize(data:np.array, start_frame = 0):
    colors = ((0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 255, 255))
    for frame, row in enumerate(data):
        if frame < start_frame:
            continue
        img = np.zeros((1080, 1440, 3), dtype = np.uint8)
        cv.putText(img, str(frame), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.75, colors[3], 2)
        for i in range(max_light):
            if int(row[pre_base + i]) == 0: continue
            center = row[4 * i : 4 * i + 2]
            vec = row[4 * i + 2 : 4 * i + 4]
            start_p = (center + vec).astype(int)
            end_p = (center - vec).astype(int)
            pos_text = end_p + 1
            car_id = int(row[car_base + i])
            img = cv.line(img, tuple(start_p), tuple(end_p), colors[car_id], 3)
            cv.putText(img, str(i), tuple(pos_text), cv.FONT_HERSHEY_SIMPLEX, 0.75, colors[3], 1)
        cv.imshow('enhance', img)
        print("Drawn")
        k = cv.waitKey(0)
        if k == 27:
            break

if __name__ == "__main__":
    enhanced_path = './enhanced.csv'
    non_path = './data_disp1500.csv'
    number = 0
    if len(sys.argv) < 2:
        print("Usage: python ./enhance.py <flag>(flag=0 means enhancement, flag=1 means visualization)")
        exit(-1)
    number = int(sys.argv[1])
    start_frame = 0
    if len(sys.argv) > 2:
        start_frame = int(sys.argv[2])
    if number == 0:
        data = loadFrom(non_path)
        enhance(data)
    else:
        data = loadFrom(enhanced_path)
        visualize(data, start_frame)