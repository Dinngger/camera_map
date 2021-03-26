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


def focalLoss(predict, label, valid, gamma=2.0, expand_dims=False):
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
    if expand_dims:
        loss = tf.expand_dims(valid, -1) * loss
        loss = tf.reduce_sum(loss, [1, 2])
    else:
        loss = valid * loss
        loss = tf.reduce_sum(loss, 1)
    return loss / (tf.reduce_sum(valid, 1) + 0.001)


def dice_loss(predict, target, valid):
    """
    Args:
      predict: [B, N, N]
      target:  [B, N, N]
      valid:   [B, N]       float32
    Returns:
      Loss: [B]
    """
    a = tf.reduce_sum(predict * target, 2)
    b = tf.reduce_sum(predict * predict, 2)
    c = tf.reduce_sum(target * target, 2)
    d = 1 - (a * 2) / (b + c + 0.001)
    return tf.reduce_sum(d * valid, 1) / (tf.reduce_sum(valid, 1) + 0.001)


def matrix_nms(masks, scores):
    """
    Args:
      masks:   [B, N, N]
      scores:  [B, N]
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


def spectralClustering(masks):
    masks = tf.Print(masks, [masks], message="masks", summarize=256)
    N = int(masks.shape[1])
    intersection = tf.matmul(masks, tf.transpose(masks, [0, 2, 1]))
    areas = tf.tile(tf.reduce_sum(masks, 2, keepdims=True), multiples=[1, 1, N])
    union = areas + tf.transpose(areas, [0, 2, 1]) - intersection
    adjacent = intersection / union     # iou

    degree = tf.reduce_sum(adjacent, 2)
    laplacian = tf.matrix_diag(degree) - adjacent
    sqrtDegree = tf.matrix_diag(1.0 / tf.sqrt(degree))
    normLaplacian = tf.matmul(tf.matmul(sqrtDegree, laplacian), sqrtDegree)
    e, v = tf.linalg.eigh(normLaplacian)
    return e, tf.transpose(v, [0, 2, 1])


class CarMatch(Model):
    """Car Match Model."""

    def __init__(self):
        super(CarMatch, self).__init__()
        self.loss_cls_rate = 0.5
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

        B = int(belong.shape[0])
        N = int(belong.shape[1])
        class_target = tf.cast(tf.math.greater(belong, 0), tf.float32) * presence_f
        mask_a = tf.tile(tf.expand_dims(belong, 2), multiples=[1, 1, N])
        mask_b = tf.tile(tf.expand_dims(belong, 1), multiples=[1, N, 1])
        mask_target = tf.cast(tf.math.equal(mask_a, mask_b), tf.float32) * tf.expand_dims(presence_f, -1) * tf.expand_dims(presence_f, -2)

        # ins_mask: [B, N, N]
        ins_mask_ = self._encoder(x, presence)
        ins_mask = tf.sigmoid(ins_mask_) * tf.expand_dims(presence_f, -1) * tf.expand_dims(presence_f, -2)
        classification = tf.reduce_sum(ins_mask - tf.matrix_band_part(ins_mask, 0, 0), -1)

        single = (1 - class_target) * presence_f
        loss_cls = tf.reduce_sum(single * classification) / tf.reduce_sum(single)
        loss_ins = tf.reduce_sum(dice_loss(ins_mask, mask_target, class_target), 0) / B
        # loss_cls = tf.Print(loss_cls, [loss_cls, loss_ins], message="loss_cls, loss_ins", summarize=256)
        loss = loss_ins * (1.0 - self.loss_cls_rate) + loss_cls * self.loss_cls_rate

        e, v = spectralClustering(ins_mask + tf.matrix_diag((1 - presence_f) * tf.ones([B, N])))

        return AttrDict(
            loss=loss,
            loss_cls=loss_cls,
            loss_ins=loss_ins,
            # acc=acc,
            input={'x': tf.saved_model.utils.build_tensor_info(x),
                   'presence': tf.saved_model.utils.build_tensor_info(presence)},
            output={'e': tf.saved_model.utils.build_tensor_info(e),
                    'v': tf.saved_model.utils.build_tensor_info(v)}
        )

    def _loss(self, data, res):
        return res.loss

    def _report(self, data, res):
        return AttrDict(loss=res.loss, loss_cls=res.loss_cls, loss_ins=res.loss_ins)  # , acc=res.acc)
