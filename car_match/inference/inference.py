
import os
import pdb
import sys

import traceback

import numpy as np
import tensorflow as tf

module_path = "/home/dinger/mine/RoboMaster/camera_map/"
input_path = module_path + "fifos/input.pipe"
output_path = module_path + "fifos/output.pipe"
model_path = module_path + "car_match/saved_model/"


def main():
    try:
        os.mkfifo(input_path)
        os.mkfifo(output_path)
        print('created pipe file!')
    except OSError as e:
        print("mkfifo error: ", e)

    output_f = os.open(output_path, os.O_SYNC | os.O_CREAT | os.O_RDWR)
    input_f = os.open(input_path, os.O_RDONLY)

    with tf.Session() as sess:
        meta_graph_def = tf.saved_model.loader.load(sess, ['car_match_model'], model_path)
        signature = meta_graph_def.signature_def
        x_tensor = tf.saved_model.utils.get_tensor_from_tensor_info(signature['sig_inout'].inputs['x'])
        presence_tensor = tf.saved_model.utils.get_tensor_from_tensor_info(signature['sig_inout'].inputs['presence'])
        belong_tensor = tf.saved_model.utils.get_tensor_from_tensor_info(signature['sig_inout'].outputs['belong'])

        data_string = bytes()
        while True:
            data_string += os.read(input_f, 5*13*4)
            # print('data string len: ', len(data_string))
            if (len(data_string) >= 5*13*4):
                data = np.frombuffer(data_string[0:5*13*4], dtype=np.float32, count=5*13)
                data_string = data_string[5*13*4:]
                print('get data: ', data)
                x = data[0:4*13]
                presence = data[4*13:4*13+13].astype(np.int32)
                x = x.reshape((1, 13, 4))
                presence = presence.reshape((1, 13))
                belong_pred = sess.run(belong_tensor, feed_dict={x_tensor: x, presence_tensor: presence})
                os.write(output_f, belong_pred.tobytes())
                print('send result!', belong_pred)
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
