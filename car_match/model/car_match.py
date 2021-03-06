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
import sonnet as snt
import tensorflow as tf
from .attention import SetTransformer, MultiHeadQKVAttention
from .general_model import Model


def mask_reduce_mean(input_tensor, mask):
    mask = tf.cast(mask, tf.float32)
    mask_sum = tf.reduce_sum(mask, axis=1, keepdims=True)
    mask_weight = tf.expand_dims(mask / mask_sum, axis=-1)
    output = tf.reduce_sum(input_tensor * mask_weight, axis=1)
    return output


class CarMatch(Model):
    """Car Match Model."""

    def __init__(self):
        super(CarMatch, self).__init__()
        self.n_outputs = 2
        self._encoder = SetTransformer(
            n_layers=3,             # SAB num
            n_heads=1,
            n_dims=16,              # attention dims
            n_output_dims=32,       # car dims
            n_outputs=self.n_outputs,            # car num
            layer_norm=True,
            dropout_rate=0.,
            n_inducing_points=0     # use SAB
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

        h = self._encoder(x, presence)
        x_expand = snt.BatchApply(snt.Linear(32))(x)

        # [B, N, n_car]
        belong_pred = MultiHeadQKVAttention(2, onehot_V=True)(x_expand, h, None)
        # b, n, n_car = belong_pred.shape.as_list()

        mask0 = presence * tf.cast(tf.equal(belong, 0), tf.int32)
        res = tf.reduce_sum(tf.expand_dims(tf.cast(mask0, tf.float32), -1) * belong_pred) / tf.cast(tf.reduce_sum(mask0), tf.float32)
        # for i in range(self.n_outputs):
        #     mask = tf.expand_dims(tf.cast(tf.equal(belong, i+1), tf.float32), -1)
        #     res -= tf.reduce_sum(tf.log1p(mask * belong_pred + 1 - mask)) / (b * n * n_car)
        # dist = (mask_reduce_mean(belong_pred, tf.equal(belong, 1)) - mask_reduce_mean(belong_pred, tf.equal(belong, 2)))
        # res -= tf.reduce_sum(tf.sqrt(tf.reduce_sum(tf.square(dist), axis=-1))) / b
        return AttrDict(
            res=res,
        )

    def _loss(self, data, res):
        return res.res

    def _report(self, data, res):
        reports = super(CarMatch, self)._report(data, res)
        return reports
