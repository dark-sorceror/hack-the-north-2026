#!/bin/sh
# One-time setup on the QNX Pi 5 (quick-start image, network with internet).
# Everything is built and installed on the Pi itself: no SDP, no cross-compiler.
#   scp -r hardware/qnx qnxuser@<pi>:supervisor && ssh qnxuser@<pi> 'sh supervisor/setup.sh'
set -e
cd "$(dirname "$0")"
SUDO=${SUDO:-sudo}   # SUDO="sudo -S" to read the password from stdin

# oss.qnx.com packages (repo.oss.qnx.com is preconfigured in /etc/apk/repositories)
$SUDO apk add python3-tflite-runtime python3-numpy python3-opencv

# COCO person detectors with TFLite_Detection_PostProcess built in
if [ ! -f ssd_mobilenet_v1_quant.tflite ]; then
    curl -fsSL -o /tmp/ssd.zip https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
    unzip -o -q /tmp/ssd.zip detect.tflite   # the image's unzip ignores -d: extract here
    mv detect.tflite ssd_mobilenet_v1_quant.tflite
fi
[ -f effdet_lite0_rpi.tflite ] || curl -fsSL -o effdet_lite0_rpi.tflite \
    https://storage.googleapis.com/download.tensorflow.org/models/tflite/task_library/object_detection/rpi/lite-model_efficientdet_lite0_detection_metadata_1.tflite

# Test frames: a photo with people, and a synthetic "walks up to the robot" clip from it
[ -f people.jpg ] || curl -fsSL -o people.jpg \
    https://raw.githubusercontent.com/tensorflow/models/master/research/object_detection/test_images/image2.jpg
[ -d approach ] || python3 make_approach.py

make
echo "setup done: ./run.sh start | status | stop"
