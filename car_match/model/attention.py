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

"""Implementation of multi-head self attention."""


import numpy as np
import sonnet as snt
import tensorflow as tf


class SelfSetTransformer(snt.AbstractModule):
    """Permutation-invariant Transformer."""

    def __init__(self):
        super(SelfSetTransformer, self).__init__()

    def _build(self, x, presence=None):
        # TODO: Bottom up and Top down.

        h = snt.TileByDim([2], [2])(x)  # dim = 4*2 = 8

        for _ in range(1):
            h = SelfAttention(n_heads=2, layerNorm=False)(h, presence)

        ins_mask = QKAttention()(h, h, presence)
        return ins_mask


class QKAttention(snt.AbstractModule):
    """Transformer-like minus attention with one-hot output"""

    def __init__(self):
        super(QKAttention, self).__init__()

    def _build(self, queries, keys, presence=None):
        """Builds the module.

        Args:
          queries: Tensor of shape [B, M, d_k].
          keys: Tensor of shape [B, N, d_k].
          presence: None or tensor of shape [B, N].

        Returns:
          Tensor of shape [B, M, N]
        """
        M = int(queries.shape[1])
        N = int(keys.shape[1])

        # [B, M, d] x [B, d, N] = [B, M, N]
        # routing = tf.matmul(queries, keys, transpose_b=True)
        queries_expand = tf.tile(tf.expand_dims(queries, 2), multiples=[1, 1, N, 1])
        keys_expand = tf.tile(tf.expand_dims(keys, 1), multiples=[1, M, 1, 1])
        routing = tf.nn.relu(snt.Conv2D(16, (1, 1))(queries_expand - keys_expand))
        routing = tf.squeeze(snt.Conv2D(1, (1, 1))(routing), axis=-1)

        if presence is not None:
            presence_f = tf.cast(presence, tf.float32)
            routing *= tf.expand_dims(presence_f, -1)
            routing *= tf.expand_dims(presence_f, -2)

        return routing


class QKVAttention(snt.AbstractModule):
    """Transformer-like attention."""

    def __init__(self):
        super(QKVAttention, self).__init__()

    def _build(self, queries, keys, values, presence=None):
        """Builds the module.

        Args:
          queries: Tensor of shape [B, M, d_k].
          keys: Tensor of shape [B, N, d_k].
          values: : Tensor of shape [B, N, d_v].
          presence: None or tensor of shape [B, N].

        Returns:
          Tensor of shape [B, M, d_v]
        """

        n_dim = int(queries.shape[-1])
        routing = QKAttention()(queries, keys)

        if presence is not None:
            presence = tf.cast(tf.expand_dims(presence, -2), tf.float32)
            routing -= (1. - presence) * 1e32

        routing = tf.nn.softmax(routing / np.sqrt(n_dim), -1)

        # every output is a linear combination of all inputs
        # [B, M, N] x [B, N, d_v] = [B, M, d_v]
        res = tf.matmul(routing, values)
        return res


class MultiHeadQKVAttention(snt.AbstractModule):
    """Multi-head version of Transformer-like attention."""

    def __init__(self, n_heads):
        super(MultiHeadQKVAttention, self).__init__()
        self._n_heads = n_heads

    def _build(self, queries, keys, values, presence=None):

        def transform(x, n=self._n_heads):
            if x is None:
                return None
            n_dim = np.ceil(float(int(x.shape[-1])) / n)
            return snt.BatchApply(snt.Linear(int(n_dim)))(x)

        outputs = []
        for _ in range(self._n_heads):
            args = [transform(i) for i in [queries, keys, values]]
            if presence is not None:
                args.append(presence)
            outputs.append(QKVAttention()(*args))

        linear = snt.BatchApply(snt.Linear(values.shape[-1]))
        return linear(tf.concat(outputs, -1))


class SelfAttention(snt.AbstractModule):
    """Self-attention where keys, values and queries are the same."""

    def __init__(self, n_heads, layerNorm=True):
        super(SelfAttention, self).__init__()
        self._n_heads = n_heads
        self.layerNorm = layerNorm

    def _build(self, x, presence=None):
        n_dim = int(x.shape[-1])

        y = MultiHeadQKVAttention(self._n_heads)(x, x, x, presence)
        y = snt.BatchApply(snt.Linear(int(n_dim)))(tf.nn.relu(y))
        y += x

        if presence is not None:
            y *= tf.expand_dims(tf.cast(presence, tf.float32), -1)
        if self.layerNorm:
            y = snt.LayerNorm(axis=1)(y)

        # h = y + snt.BatchApply(snt.nets.MLP([2*n_dim, n_dim]))(y)
        # if self.layerNorm:
        #     h = snt.LayerNorm(axis=1)(h)

        return y
