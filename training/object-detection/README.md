generate_tfrecord:
from 
/models/research/object_detection/youtube/object-detection run:

python3 generate_tfrecord.py --csv_input=data/train_labels.csv --output_path=data/train.record
python3 generate_tfrecord.py --csv_input=data/test_labels.csv --output_path=data/test.record


adjust steps


merge folders

data, images, ssd_mobilenet...2017, training, ssd_mobilenet...config

to models/object_detection



remove old stuff training folder



train:
From within HDD/System_Integration/models/research/object_detection

python3 train.py --logtostderr --train_dir=training/ --pipeline_config_path=training/ssd_mobilenet_v1_coco.config







Modifications:

batch_size = 8 (needs additional ~6GB of memory)
models/research/object_detection/youtube/object-detection/ssd_mobilenet_v1_coco.config
models/research/object_detection/youtube/object-detection/training/ssd_mobilenet_v1_coco.config




export_inference_graph.py: The number(310) is #steps from last checkpoint

python3 export_inference_graph.py     --input_type image_tensor     --pip
eline_config_path training/ssd_mobilenet_v1_coco.config     --trained_checkpoint_prefix training/model.ckpt-500     --output_directory red_inference_graph


