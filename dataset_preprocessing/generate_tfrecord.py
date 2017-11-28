import argparse
import tensorflow as tf
import os
import subprocess
import sys
import csv
from PIL import Image
import io

parent_path = os.path.dirname(os.getcwd())
models_path = os.path.join(parent_path, 'models')

# Check if tensorflow/models repo exists in parent directory
if not os.path.exists(models_path):

    # Clone tensorflow/models repo into parent directory if does not exist
    tf_models_repo = 'https://github.com/tensorflow/models.git'
    subprocess.call(['git', 'clone', tf_models_repo, models_path])

# Add path to dataset_util
sys.path.insert(0, os.path.join(parent_path, 'models/research/object_detection/utils'))

import dataset_util

# Parse arguments from terminal
parser = argparse.ArgumentParser()

parser.add_argument('--read_path', '-r', type=str, required=True,
                    help='Path to directory containing dataset images')

parser.add_argument('--labels_filename', '-l', type=str, required=True,
                    help='Name of file containing dataset labels')

parser.add_argument('--save_filename', '-s', type=str, required=True,
                    help='Name of file to be saved')

args = parser.parse_args()


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

    # Assuming one bounding box per image
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


def main():

    labels_path = os.path.join(args.read_path, args.labels_filename)
    record_path = os.path.join(args.read_path, args.save_filename)

    writer = tf.python_io.TFRecordWriter(record_path)

    with open(labels_path, 'r') as labels_reader:
        examples = csv.reader(labels_reader, delimiter=',')

        for example in examples:

            # Skip labels file header
            if example[0] != 'image':
                tf_example = create_tf_example(example, args.read_path)
                writer.write(tf_example.SerializeToString())

    writer.close()


if __name__ == '__main__':
    main()
