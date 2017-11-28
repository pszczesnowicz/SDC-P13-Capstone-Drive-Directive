import tensorflow as tf
import csv
import os
from PIL import Image
import io
import sys

sys.path.insert(0, '/Users/skunkworks/programming/SDCND/Projects/SDC-P13-Capstone-Drive-Directive/'
                   'models/research/object_detection/utils')
import dataset_util
# from object_detection.utils import dataset_util

flags = tf.app.flags
flags.DEFINE_string('directory_path', '', 'Path to input directory')
flags.DEFINE_string('labels_filename', '', 'Name of labels file')
flags.DEFINE_string('record_filename', '', 'Name of record file')
FLAGS = flags.FLAGS


def create_tf_example(example, directory_path):

    filename = example[0].encode()

    # Open image
    img_path = os.path.join(directory_path, example[0])
    img = Image.open(img_path)

    # Encode image
    img_bytes_array = io.BytesIO()
    img.save(img_bytes_array, format='JPEG')
    encoded_image_data = img_bytes_array.getvalue()

    height, width = img.size
    image_format = b'jpg'

    xmins = [float(example[3]) / width]  # List of normalized left x coordinates in bounding box (1 per box)
    xmaxs = [float(example[4]) / width]  # List of normalized right x coordinates in bounding box (1 per box)
    ymins = [float(example[5]) / height]  # List of normalized top y coordinates in bounding box (1 per box)
    ymaxs = [float(example[6]) / height]  # List of normalized bottom y coordinates in bounding box (1 per box)
    classes_text = [example[2].encode()]  # List of string class name of bounding box (1 per box)

    # List of integer class id of bounding box (1 per box)
    if example[2] == 'red':
        classes = [1]
    elif example[2] == 'yellow':
        classes = [2]
    elif example[2] == 'green':
        classes = [3]

    tf_example = tf.train.Example(features=tf.train.Features(feature={
      'image/height': dataset_util.int64_feature(height),
      'image/width': dataset_util.int64_feature(width),
      'image/filename': dataset_util.bytes_feature(filename),
      'image/source_id': dataset_util.bytes_feature(filename),
      'image/encoded': dataset_util.bytes_feature(encoded_image_data),
      'image/format': dataset_util.bytes_feature(image_format),
      'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
      'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
      'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
      'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
      'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
      'image/object/class/label': dataset_util.int64_list_feature(classes),
    }))

    return tf_example


def main(_):

    labels_path = os.path.join(FLAGS.directory_path, FLAGS.labels_filename)
    record_path = os.path.join(FLAGS.directory_path, FLAGS.record_filename)

    writer = tf.python_io.TFRecordWriter(record_path)

    with open(labels_path, 'r') as labels_reader:
        examples = csv.reader(labels_reader, delimiter=',')

        for example in examples:

            # Skip labels file header
            if example[0] != 'image':
                tf_example = create_tf_example(example, FLAGS.directory_path)
                writer.write(tf_example.SerializeToString())

    writer.close()


if __name__ == '__main__':
    tf.app.run()
