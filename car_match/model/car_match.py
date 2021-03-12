# coding=utf-8
# Copyright 2020 The Google Research Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from monty.collections import AttrDict
import tensorflow as tf
from .attention import SelfSetTransformer
from .general_model import Model


def focalLoss(predict, label, valid):
    p = tf.square(label - predict)
    # p = p * tf.cast(tf.less(p, 0.99), tf.float32)
    return tf.reduce_sum(tf.expand_dims(valid, -1) * p, [1, 2])


class CarMatch(Model):
    """Car Match Model."""

    def __init__(self):
        super(CarMatch, self).__init__()
        self.n_outputs = 2
        self._encoder = SelfSetTransformer(
            n_outputs=self.n_outputs,            # car num
        )

    def _build(self, data):
        """Builds the module.

        Args:
          x: Tensor of shape [B, N, 4].             float32
          belong: Tensor of shape [B, N].           int32
          presence: None or tensor of shape [B, N]. int32

        Returns:
          Res
        """
        x, presence, belong = data['x'], data['presence'], data['belong']
        presence_f = tf.cast(presence, tf.float32)
        presence_all = tf.reduce_sum(presence_f)

        # [B, N, n_car]
        belong_pred = self._encoder(x, presence)
        belong_inverse = tf.floordiv(belong + tf.cast(tf.equal(belong, 1), tf.int32) * 3, 2)
        # belong = tf.Print(belong, [belong[0], belong_inverse[0]], message='belong: ', summarize=32)

        belong_one_hot = tf.one_hot(belong - 1, self.n_outputs, on_value=1.0, off_value=0.0, axis=-1, dtype=tf.float32)
        belong_inv_one_hot = tf.one_hot(belong_inverse - 1, self.n_outputs, on_value=1.0, off_value=0.0, axis=-1, dtype=tf.float32)
        loss1 = focalLoss(belong_pred, belong_one_hot, presence_f)
        loss2 = focalLoss(belong_pred, belong_inv_one_hot, presence_f)
        loss = tf.reduce_sum(tf.minimum(loss1, loss2), 0) / presence_all

        belong_pred_round = tf.cast(tf.greater(belong_pred, 0.5), tf.float32)
        accelerate1 = tf.cast(tf.reduce_all(tf.less(tf.abs(belong_pred_round - belong_one_hot), 0.5), axis=-1), tf.float32)
        accelerate2 = tf.cast(tf.reduce_all(tf.less(tf.abs(belong_pred_round - belong_inv_one_hot), 0.5), axis=-1), tf.float32)
        accelerate1B = tf.reduce_sum(presence_f * accelerate1, 1)
        accelerate2B = tf.reduce_sum(presence_f * accelerate2, 1)
        acc = tf.reduce_sum(tf.maximum(accelerate1B, accelerate2B), 0) / presence_all
        return AttrDict(
            loss=loss,
            acc=acc,
            input={'x': tf.saved_model.utils.build_tensor_info(x),
                   'presence': tf.saved_model.utils.build_tensor_info(presence)},
            output={'belong': tf.saved_model.utils.build_tensor_info(belong_pred)}
        )

    def _loss(self, data, res):
        return res.loss

    def _report(self, data, res):
        return AttrDict(loss=res.loss, acc=res.acc)
