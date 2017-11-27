import argparse
import os
import glob
import sys
import numpy as np
from PIL import Image


def preprocess(read_path, write_path, class_name, num_examples, width):

    # Create an array of image file paths
    img_path_array = np.array(glob.glob(os.path.join(read_path, class_name + '*.jpg')))

    # Shuffle images file paths
    np.random.shuffle(img_path_array)

    # Reduce number of images file paths
    img_array_reduced = img_path_array[:num_examples]

    # Read, resize, and write images to new path
    img = Image.open(img_array_reduced[0])
    resize_percent = width / float(img.size[0])
    height = int(float(img.size[1]) * resize_percent)

    for i in range(len(img_array_reduced)):

        img = Image.open(img_array_reduced[i])
        img = img.resize((width, height))
        img_write_path = os.path.join(write_path, class_name + '{:0>4}'.format(i) + '.jpg')
        img.save(img_write_path)

    return None


parser = argparse.ArgumentParser()
parser.add_argument('read_path', type=str, help='path to directory containing original images')
parser.add_argument('write_directory', type=str, help='name of directory where processed images will be saved')
parser.add_argument('width', type=int, help='processed image width (aspect ratio maintained)')
parser.add_argument('num_examples', type=int, help='number of examples to keep from each class')
parser.add_argument('class_names', nargs='*', type=str,
                    help='class names corresponding to original image file base names')
args = parser.parse_args()

head, tail = os.path.split(args.read_path)

write_path = os.path.join(head, args.write_directory)

if os.path.exists(write_path):
    sys.exit('Write directory exists. Exiting.')
else:
    os.makedirs(write_path)

class_names = np.array(args.class_names)

for class_name in class_names:

    preprocess(args.read_path, write_path, class_name, args.num_examples, args.width)
