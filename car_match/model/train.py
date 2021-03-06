
r"""Training loop."""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import pdb
import shutil
import sys

import traceback
from absl import flags
from absl import logging

import numpy as np
import tensorflow as tf

from . import model_config
from .dataset import getDataset
from .hooks import create_hooks
from . import tools

# from tensorflow.python import debug as tf_debug

flags.DEFINE_string('name', None, '')
flags.mark_flag_as_required('name')

flags.DEFINE_string('logdir', 'checkpoints/{name}',
                    'Log and checkpoint directory for the experiment.')

flags.DEFINE_float('grad_value_clip', .0, '')
flags.DEFINE_float('grad_norm_clip', .0, '')

flags.DEFINE_float('ema', .9, 'Exponential moving average weight for smoothing '
                   'reported results.')

flags.DEFINE_integer('run_updates_every', 10, '')
flags.DEFINE_boolean('global_ema_update', True, '')

flags.DEFINE_integer('max_train_steps', int(3e5), '')
flags.DEFINE_integer('snapshot_secs', 0, '')
flags.DEFINE_integer('snapshot_steps', 500, '')
flags.DEFINE_integer('snapshots_to_keep', 5, '')

flags.DEFINE_integer('summary_steps', 250, '')
flags.DEFINE_integer('report_loss_steps', 250, '')

flags.DEFINE_boolean('plot', False, 'Produces intermediate results plots '
                     'if True.')
flags.DEFINE_integer('plot_steps', 1000, '')

flags.DEFINE_boolean('overwrite', False, 'Overwrites any existing run of the '
                     'same name if True; otherwise it tries to restore the '
                     'model if a checkpoint exists.')

flags.DEFINE_boolean('check_numerics', False, 'Adds check numerics ops.')
flags.DEFINE_boolean('save_model', False, 'Save model as saved_model')


def main(_=None):
    FLAGS = flags.FLAGS
    if FLAGS.save_model:
        FLAGS.batch_size = 1
        FLAGS.overwrite = False
    config = FLAGS
    FLAGS.__dict__['config'] = config

    FLAGS.logdir = FLAGS.logdir.format(name=FLAGS.name)

    logdir = FLAGS.logdir
    save_model_dir = "saved_model/"
    logging.info('logdir: %s', logdir)

    if os.path.exists(logdir) and FLAGS.overwrite and not FLAGS.save_model:
        logging.info(
            '"overwrite" is set to True. Deleting logdir at "%s".', logdir)
        shutil.rmtree(logdir)
    if os.path.exists(save_model_dir) and FLAGS.save_model:
        logging.info(
            '"save_model" is set to True. Deleting save_model_dir at "%s".', save_model_dir)
        shutil.rmtree(save_model_dir)

    with tf.Graph().as_default():
        model_dict = model_config.get(FLAGS)

        lr = model_dict.lr
        opt = model_dict.opt
        model = model_dict.model

        lr = tf.convert_to_tensor(lr)
        tf.summary.scalar('learning_rate', lr)

        global_step = tf.train.get_or_create_global_step()
        train_dataset, test_dataset = getDataset(config)
        target, res = model.make_target(train_dataset)

        gvs = opt.compute_gradients(target)

        suppress_inf_and_nans = (config.grad_value_clip > 0
                                 or config.grad_norm_clip > 0)
        report = tools.gradient_summaries(gvs, suppress_inf_and_nans)
        valid_report = dict()

        gvs = tools.clip_gradients(gvs, value_clip=config.grad_value_clip,
                                   norm_clip=config.grad_norm_clip)

        try:
            report.update(model.make_report(train_dataset))
            valid_report.update(model.make_report(test_dataset))
        except AttributeError:
            logging.warning(
                'Model %s has no "make_report" method.', str(model))
            raise

        report = tools.scalar_logs(report, config.ema, 'train',
                                   global_update=config.global_ema_update)
        report['lr'] = lr
        valid_report = tools.scalar_logs(
            valid_report, config.ema, 'valid',
            global_update=config.global_ema_update)

        reports_keys = sorted(report.keys())

        def _format(k):
            if k in ('lr', 'learning_rate'):
                return '.2E'
            return '.3f'

        report_template = ', '.join(['{}: {}{}:{}{}'.format(
            k, '{', k, _format(k), '}') for k in reports_keys])

        logging.info('Trainable variables:')
        tools.log_variables_by_scope()

        # inspect gradients
        for g, v in gvs:
            if g is None:
                logging.warning('No gradient for variable: %s.', v.name)

        tools.log_num_params()

        update_ops = tf.get_collection(tf.GraphKeys.UPDATE_OPS)
        if FLAGS.check_numerics:
            update_ops += [tf.add_check_numerics_ops()]

        with tf.control_dependencies(update_ops):
            train_step = opt.apply_gradients(gvs, global_step=global_step)

        sess_config = tf.ConfigProto()
        sess_config.gpu_options.allow_growth = True

        if config.save_model:
            saver = tf.train.Saver()
            with tf.Session() as sess:
                saver.restore(sess, logdir + "/best.ckpt")
                signature = tf.saved_model.signature_def_utils.build_signature_def(res.input, res.output, 'sig_inout')
                builder = tf.saved_model.builder.SavedModelBuilder("saved_model/")
                builder.add_meta_graph_and_variables(sess, ['car_match_model'], {'sig_inout': signature})
                builder.save()
        else:
            with tf.train.SingularMonitoredSession(
                    hooks=create_hooks(FLAGS),
                    checkpoint_dir=logdir, config=sess_config) as sess:

                train_itr, _ = sess.run([global_step, update_ops])
                train_tensors = [global_step, train_step]
                report_tensors = [report, valid_report]
                all_tensors = report_tensors + train_tensors
                valid_loss = 1e5

                while train_itr < config.max_train_steps:

                    if train_itr % config.report_loss_steps == 0:
                        report_vals, valid_report_vals, train_itr, _ = sess.run(
                            all_tensors)
                        this_loss = valid_report_vals['loss']
                        if this_loss < valid_loss and tf.train.latest_checkpoint(logdir) is not None:
                            ends = ['.data-00000-of-00001', '.index', '.meta']
                            for end in ends:
                                shutil.copy(tf.train.latest_checkpoint(logdir) + end, logdir + "/best.ckpt" + end)

                        logging.info('')
                        logging.info('train:')
                        logging.info('#%s: %s', train_itr,
                                     report_template.format(**report_vals))

                        logging.info('valid:')
                        valid_logs = dict(report_vals)
                        valid_logs.update(valid_report_vals)
                        logging.info('#%s: %s', train_itr,
                                     report_template.format(**valid_logs))

                        vals_to_check = list(report_vals.values())
                        if (np.isnan(vals_to_check).any()
                                or np.isnan(vals_to_check).any()):
                            logging.fatal('NaN in reports: %s; breaking...',
                                          report_template.format(**report_vals))
                    else:
                        train_itr, _ = sess.run(train_tensors)


if __name__ == '__main__':
    try:
        logging.set_verbosity(logging.INFO)
        tf.app.run()
    except Exception as err:
        FLAGS = flags.FLAGS

        last_traceback = sys.exc_info()[2]
        traceback.print_tb(last_traceback)
        print(err)
        pdb.post_mortem(last_traceback)
