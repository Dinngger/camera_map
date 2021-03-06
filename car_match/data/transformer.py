# -*-coding:utf-8-*-

import csv

max_light = 13
max_light_4 = 52


def loadFrom(path='data.csv'):
    data = []
    with open(path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            buf = [int(row[0])]
            i = 1
            for i in range(1, 4 * buf[0] + 1):
                buf.append(float(row[i]))
            for elem in row[i + 1:-1]:
                buf.append(int(elem))
            data.append(buf)
    return data


def makeFormat(data):
    new_data = []
    for row in data:
        lights = row[0]
        new_row = row[1:lights * 4 + 1]
        length = len(new_row)
        if length < max_light_4:
            new_row.extend([0.0 for i in range(max_light_4 - length)])
        exists = [0 for i in range(max_light)]
        for i in range(lights):
            exists[i] = 1
        tags = row[lights * 4 + 2:]         # 省略2
        belongs = [0 for i in range(max_light)]
        cnt = 0
        for i, elem in enumerate(tags):
            if elem == -1:
                cnt += 1
            else:
                belongs[i - cnt] = cnt
        new_row.extend(exists)
        new_row.extend(belongs)
        new_data.append(new_row)
    return new_data


def writeCsv(data, opath='transformer.csv'):
    with open(opath, 'w') as file:
        writer = csv.writer(file)
        for row in data:
            writer.writerow(row)
    print("Data transformed.")


if __name__ == "__main__":
    data = loadFrom()
    new_data = makeFormat(data)
    writeCsv(new_data)
