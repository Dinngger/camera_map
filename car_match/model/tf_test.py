import tensorflow as tf

tf.enable_eager_execution()
# a = tf.constant([[[1,  1,  2, 3],
#                  [-1,  2,  1, 2],
#                  [-2, -1,  3, 1],
#                  [-3, -2, -1, 5]]], dtype=tf.float32)
# print(a)
# # sort_inds = tf.argsort(a, direction='DESCENDING')
# sort_inds = tf.expand_dims(tf.arg_max(a, -1), -1)
# print(sort_inds)
# b = tf.squeeze(tf.gather(a, sort_inds, axis=2, batch_dims=2), -1)
# print(b)

# routing = tf.constant([[[1,  1,  2, 3],
#                        [-1,  2,  1, 2],
#                        [-2, -1,  3, 1],
#                        [-3, -2, -1, 5]]], dtype=tf.float32)
# presence = tf.constant([[1,  1,  1, 0]], dtype=tf.float32)
# print(routing * tf.expand_dims(presence, -1) * tf.expand_dims(presence, -2))

# belong = tf.constant([[0,  1,  1, 2]], dtype=tf.float32)
# N = int(belong.shape[1])
# class_target = tf.cast(tf.math.greater(belong, 0), tf.float32)
# mask_a = tf.tile(tf.expand_dims(belong, 2), multiples=[1, 1, N])
# mask_b = tf.tile(tf.expand_dims(belong, 1), multiples=[1, N, 1])
# mask_target = tf.cast(tf.math.equal(mask_a, mask_b), tf.float32)
# print(class_target)
# print(mask_target)

# masks = tf.constant([[[0.999574721, 0.0443382263, 0.0129578114, 0.998751044, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                     [0.140030086, 0.999574721, 0.999691844, 0.745542645, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                     [0.0827793479, 0.999827087, 0.999574721, 0.593562722, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                     [0.994281292, 0.457126588, 0.368007958, 0.999574721, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
#                     [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
#                     [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
#                     [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
#                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
#                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
#                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
#                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]], dtype=tf.float32)
masks = tf.constant([[[1, 1, 0, 0],
                      [1, 1, 0, 0],
                      [0, 0, 1, 1],
                      [0, 0, 1, 1]]], dtype=tf.float32)
N = int(masks.shape[1])
intersection = tf.matmul(masks, tf.transpose(masks, [0, 2, 1]))
areas = tf.tile(tf.reduce_sum(masks, 2, keepdims=True), multiples=[1, 1, N])
union = areas + tf.transpose(areas, [0, 2, 1]) - intersection
adjacent = intersection / union     # iou
print(adjacent)

degree = tf.reduce_sum(adjacent, 2)
laplacian = tf.matrix_diag(degree) - adjacent
sqrtDegree = tf.matrix_diag(1.0 / tf.sqrt(degree))
normLaplacian = tf.matmul(tf.matmul(sqrtDegree, laplacian), sqrtDegree)
print(normLaplacian)
e, v = tf.linalg.eigh(normLaplacian)
print(e, tf.transpose(v, [0, 2, 1]))
