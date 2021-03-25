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


def focalLoss(predict, label, valid, gamma=2.0):
    """
    Returns:
      Loss: [B]
    """
    p = tf.sigmoid(predict)
    pos_term = label * ((1 - p) ** gamma)
    neg_term = (1 - label) * (p ** gamma)

    # Term involving the log and ReLU
    log_weight = pos_term + neg_term
    log_term = tf.math.log1p(tf.math.exp(-tf.math.abs(predict)))
    log_term += tf.nn.relu(-predict)
    log_term *= log_weight

    # Combine all the terms into the loss
    loss = neg_term * predict + log_term
    loss = tf.expand_dims(valid, -1) * loss
    loss = tf.reduce_sum(loss, [1, 2])
    return loss / tf.reduce_sum(valid, [1])


def dice_loss(predict, target, valid):
    """
    Args:
      predict: [B, N, N]
      target:  [B, N, N]
      valid:   [B, N]       float32
    Returns:
      Loss: [B]
    """
    predict *= tf.expand_dims(valid, -1)
    target *= tf.expand_dims(valid, -1)
    a = tf.reduce_sum(predict * target, [1, 2])
    b = tf.reduce_sum(predict * predict, [1, 2]) + 0.001
    c = tf.reduce_sum(target * target, [1, 2]) + 0.001
    d = (a * 2) / (b + c)
    return 1 - d


def matrix_nms(masks, scores):
    """
    Args:
      masks:   [B, N, N]
      scores:  [B, N]
      valid:   [B, N]       float32
    Returns:
      Loss: [B]
    """
    N = int(scores.shape[1])
    intersection = tf.matmul(masks, tf.transpose(masks, [0, 2, 1]))
    areas = tf.tile(tf.reduce_sum(masks, 2, keepdims=True), multiples=[1, 1, N])
    union = areas + tf.transpose(areas, [0, 2, 1]) - intersection
    ious = intersection / union
    ious = ious - tf.matrix_band_part(ious, -1, 0)
    ious_cmax = tf.transpose(tf.tile(tf.reduce_max(ious, 1, keepdims=True), multiples=[1, N, 1]), [0, 2, 1])
    decay = (1 - ious) / (1 - ious_cmax)
    decay = tf.reduce_min(decay, 1)
    return scores * decay


class CarMatch(Model):
    """Car Match Model."""

    def __init__(self):
        super(CarMatch, self).__init__()
        self.loss_cls_rate = 0.2
        self._encoder = SelfSetTransformer()

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
        belong *= presence
        presence_f = tf.cast(presence, tf.float32)

        N = int(belong.shape[1])
        class_target = tf.cast(tf.gt(belong, 0), tf.float32)
        mask_a = tf.tile(tf.expand_dims(belong, 2), multiples=[1, 1, N])
        mask_b = tf.tile(tf.expand_dims(belong, 1), multiples=[1, N, 1])
        mask_target = tf.cast(tf.eq(mask_a, mask_b), tf.float32)

        # ins_mask: [B, N, N]
        # classification: [B, N]
        ins_mask, classification = self._encoder(x, presence)
        loss_cls = focalLoss(classification, class_target, presence_f)
        loss_ins = dice_loss(tf.sigmoid(ins_mask), mask_target, class_target)
        loss = tf.reduce_sum(loss_ins, 0) * (1.0 - self.loss_cls_rate) + tf.reduce_sum(loss_cls, 0) * self.loss_cls_rate

        sort_inds = tf.argsort(classification, direction='DESCENDING')
        ins_mask = ins_mask[sort_inds]

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
