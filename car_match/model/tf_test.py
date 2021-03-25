import tensorflow as tf

tf.enable_eager_execution()
a = tf.constant([[1,  1,  2, 3],
                 [-1,  2,  1, 2],
                 [-2, -1,  3, 1],
                 [-3, -2, -1, 5]], dtype=tf.float32)
print(a)
sort_inds = tf.argsort(a, direction='DESCENDING')
print(sort_inds)
b = tf.gather(a, sort_inds)
print(b)
