
from absl import flags
from collections import namedtuple
import numpy as np
from sklearn.model_selection import train_test_split
import tensorflow as tf

flags.DEFINE_integer('batch_size', 32, 'batch size')
flags.DEFINE_integer('repeat', 1, 'dataset repeat')


def loadData(batch_size):
    path = "./data/transformer.csv"
    data = np.loadtxt(path, dtype=np.float32, delimiter=",")
    batch_num = data.shape[0] // batch_size
    data = data[0:batch_num*batch_size, :]
    x = data[:, 0:4*13]
    x = x.reshape((-1, 13, 4))
    presence = data[:, 4*13:4*13+13].astype(np.int32)
    belong = data[:, 4*13+13:].astype(np.int32)
    return x, presence, belong


def getDataset(config):
    batch_size = config.batch_size
    repeat = config.repeat
    x, presence, belong = loadData(batch_size)
    x_train, x_test, presence_train, presence_test, belong_train, belong_test = train_test_split(
        x, presence, belong, test_size=1/6)
    dataset_type = namedtuple("dataset", ["x", "presence", "belong"])

    def train_generator():
        for _ in range(repeat):
            for i in range(1152 * 2 // batch_size * 5 // 6):
                yield dict(x=x_train[i*batch_size:(i+1)*batch_size],
                           presence=presence_train[i*batch_size:(i+1)*batch_size],
                           belong=belong_train[i*batch_size:(i+1)*batch_size])

    def test_generator():
        for _ in range(repeat):
            for i in range(1152 * 2 // batch_size // 6):
                yield dict(x=x_test[i*batch_size:(i+1)*batch_size],
                           presence=presence_test[i*batch_size:(i+1)*batch_size],
                           belong=belong_test[i*batch_size:(i+1)*batch_size])

    minibatch = next(train_generator())
    dtypes = {k: v.dtype for k, v in minibatch.items()}
    shapes = {k: v.shape for k, v in minibatch.items()}

    train_dataset = tf.data.Dataset.from_generator(
        train_generator, dtypes, shapes)
    test_dataset = tf.data.Dataset.from_generator(
        test_generator, dtypes, shapes)

    train_iter_data = train_dataset.make_one_shot_iterator()
    test_iter_data = test_dataset.make_one_shot_iterator()
    train_input_batch = train_iter_data.get_next()
    test_input_batch = test_iter_data.get_next()
    for _, v in train_input_batch.items():
        v.set_shape([batch_size] + v.shape[1:].as_list())
    for _, v in test_input_batch.items():
        v.set_shape([batch_size] + v.shape[1:].as_list())
    return dataset_type(**train_input_batch), dataset_type(**test_input_batch)


if __name__ == "__main__":
    x, presence, belong = loadData()
    print(x.shape)
    print(x[0, 0:2, :])
    print(presence[0, :])
    print(belong[0, :])
