
from absl import flags
from collections import namedtuple
import numpy as np
from sklearn.model_selection import train_test_split
import tensorflow as tf

flags.DEFINE_integer('batch_size', 32, 'batch size')
flags.DEFINE_integer('repeat', 1, 'dataset repeat')


def loadData():
    path = "./data/transformer.csv"
    data = np.loadtxt(path, dtype=np.float32, delimiter=",")[0:1152, :]
    x = data[:, 0:4*13]
    x = x.reshape((-1, 13, 4))
    presence = data[:, 4*13:4*13+13].astype(np.int32)
    belong = data[:, 4*13+13:].astype(np.int32)
    return x, presence, belong


def getDataset(config):
    batch_size = config.batch_size
    x, presence, belong = loadData()
    x_train, x_test, presence_train, presence_test, belong_train, belong_test = train_test_split(x, presence, belong, test_size=0.1666)
    dataset_type = namedtuple("dataset", ["x", "presence", "belong"])
    train_dataset = tf.data.Dataset.from_tensor_slices((x_train, presence_train, belong_train)).repeat(config.repeat).batch(batch_size)
    test_dataset = tf.data.Dataset.from_tensor_slices((x_test, presence_test, belong_test)).batch(batch_size)
    train_iter_data = train_dataset.make_one_shot_iterator()
    test_iter_data = test_dataset.make_one_shot_iterator()
    train_input_batch = train_iter_data.get_next()
    test_input_batch = test_iter_data.get_next()
    for v in train_input_batch:
        v.set_shape([batch_size] + v.shape[1:].as_list())
    for v in test_input_batch:
        v.set_shape([batch_size] + v.shape[1:].as_list())
    return dataset_type(*train_input_batch), dataset_type(*test_input_batch)


if __name__ == "__main__":
    x, presence, belong = loadData()
    print(x.shape)
    print(x[0, 0:2, :])
    print(presence[0, :])
    print(belong[0, :])
