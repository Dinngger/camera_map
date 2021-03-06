
from absl import flags
from monty.collections import AttrDict
import tensorflow as tf
from .car_match import CarMatch

flags.DEFINE_float('lr', 1e-4, 'Learning rate.')
flags.DEFINE_boolean('use_lr_schedule', True, 'Uses learning rate schedule'
                     ' if True.')

flags.DEFINE_integer('n_channels', 1, 'Number of input channels.')

flags.DEFINE_boolean('colorize_templates', False, 'Whether to infer template '
                     'color from input.')
flags.DEFINE_boolean('use_alpha_channel', False, 'Learns per-pixel mixing '
                     'proportions for every template; otherwise mixing '
                     'probabilities are constrained to have the same value as '
                     'image pixels.')

flags.DEFINE_string('template_nonlin', 'relu1', 'Nonlinearity used to normalize'
                    ' part templates.')
flags.DEFINE_string('color_nonlin', 'relu1', 'Nonlinearity used to normalize'
                    ' template color (intensity) value.')


def get(config):
    """Builds the model."""
    model = CarMatch()

    lr = config.lr
    if config.use_lr_schedule:
        global_step = tf.train.get_or_create_global_step()
        lr = tf.train.exponential_decay(
            global_step=global_step,
            learning_rate=lr,
            decay_steps=1e4,
            decay_rate=.96)

    eps = 1e-2 / float(config.batch_size) ** 2
    opt = tf.train.RMSPropOptimizer(config.lr, momentum=.9, epsilon=eps)

    return AttrDict(model=model, opt=opt, lr=config.lr)
