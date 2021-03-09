import os
import pdb
import sys
import traceback
import numpy as np

module_path = "/home/dinger/mine/RoboMaster/camera_map/"
input_path = module_path + "fifos/input.pipe"
output_path = module_path + "fifos/output.pipe"
data_path = module_path + "car_match/data/transformer.csv"


def main():
    input_f = os.open(input_path, os.O_SYNC | os.O_CREAT | os.O_RDWR)
    output_f = os.open(output_path, os.O_RDONLY)

    data = np.loadtxt(data_path, dtype=np.float32, delimiter=",")
    res_string = bytes()
    for i in range(2):
        input_data = data[i, 0:4*13+13]  # 1 x 5*13 float32
        print("send bytes: ", os.write(input_f, input_data.tobytes()))
        print("send data: ", input_data)

        while True:
            res_string += os.read(output_f, 13*2*4)
            if (len(res_string) >= 13*2*4):
                result = np.frombuffer(res_string[0:13*2*4], dtype=np.float32, count=13*2)
                res_string = res_string[13*2*4:]
                result.shape = (13, 2)
                print(result)
                break

    os.close(input_f)
    os.close(output_f)


if __name__ == '__main__':
    try:
        main()
    except Exception as err:
        last_traceback = sys.exc_info()[2]
        traceback.print_tb(last_traceback)
        print(err)
        pdb.post_mortem(last_traceback)
