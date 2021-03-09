import tensorflow as tf

converter = tf.lite.TFLiteConverter.from_saved_model("saved_model/", tag_set={"car_match_model"}, signature_key='sig_inout')
tflite_model = converter.convert()
open("car_model.tflite", "wb").write(tflite_model)
