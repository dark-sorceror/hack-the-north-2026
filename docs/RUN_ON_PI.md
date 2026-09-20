#!/usr/bin/env python3
"""
Quick-start guide for running the microphone → VLA → object detection pipeline

Three ways to run this test:

1. LOCAL TESTING (on your computer):
   python test_microphone_vla_detection.py
   - Tests VLA task decomposition
   - Camera access will fail locally (no RealSense hardware)

2. REMOTE TESTING (on Raspberry Pi via SSH):
   bash deploy_to_pi.sh
   - Uploads test to Pi
   - Installs dependencies
   - Runs full pipeline with RealSense camera

3. DIRECT SSH COMMAND (one-liner):
   ssh gisooj@gisoopi.local 'cd ~/VLA-HTN && source venv/bin/activate && python test_microphone_vla_detection.py'

============================================================================
                    RECOMMENDED: Running on the Pi
============================================================================

Prerequisites (already done):
  ✓ SSH access to gisoopi.local
  ✓ Python 3.11+ venv at ~/VLA-HTN/venv
  ✓ RealSense camera connected and drivers installed
  ✓ YOLO weights at ~/VLA-HTN/yolov8s-worldv2.pt

Step 1: Copy the test file to the Pi
  scp test_microphone_vla_detection.py gisooj@gisoopi.local:~/VLA-HTN/

Step 2: SSH into the Pi and run:
  ssh gisooj@gisoopi.local
  cd ~/VLA-HTN
  source venv/bin/activate
  
  # Install latest version with all deps (one-time only)
  pip install -e '.[camera,audio,camera-realsense]'
  
  # Run the test
  python test_microphone_vla_detection.py

Expected output on Pi:
  ======================================================================
  LIVE TEST: Microphone Command → VLA → Object Detection
  ======================================================================
  
  [1/4] MICROPHONE INPUT SIMULATION
  ------...
  🎤 Simulated microphone input: 'detect a bottle'
  
  [2/4] VLA TASK DECOMPOSITION
  ------...
  📊 Initializing VLA decomposer (model: qwen3.5-plus)
  ✓ Task plan generated:
    Target object: a bottle
    Confidence: 1.0
    Subgoals: 4 steps
      1. navigate
      2. approach_arm
      3. grasp
      4. deliver
  
  [3/4] CAMERA CAPTURE
  ------...
  ✓ Frame captured: 640x480 pixels
  
  [4/4] OBJECT DETECTION
  ------...
  🔍 Detecting 'a bottle' in frame...
  ✓ DETECTION SUCCESSFUL!
    Object: a bottle
    Bounding box: (150.2, 120.5) → (450.8, 380.3)
    Confidence: 85.00%
    Width: 300.6px
    Height: 259.8px

============================================================================
                         Integration with Voice
============================================================================

To test with LIVE microphone input:

1. Modify voice.py coordinator to call this pipeline:
   from robot_app.camera import CameraDriver
   
   async def handle_voice_command(audio_input):
       # Parse command
       target = extract_noun(audio_input)
       
       # Decompose with VLA
       plan = await decomposer.adecompose(f"Pick up the {target}")
       
       # Detect in camera
       frame = camera_source.capture()
       detection = detector.detect(frame.image, plan['object']['class'])
       
       if detection:
           return detection  # Use for navigation
       else:
           return "Object not found"

2. The coordinator already calls this in fetch():
   observation = await step("locate", self.camera_url, target=target)

3. Camera service locate action already uses YOLO:
   detection = await asyncio.to_thread(self.detector.detect, frame.image, request.target)

============================================================================
                       Running the Full Chain
============================================================================

To test the complete robot pipeline (voice → VLA → detection → fetch):

ssh gisooj@gisoopi.local << 'EOF'
cd ~/VLA-HTN
source venv/bin/activate

# Start camera service (in background)
python -m robot_app.camera_feed &

# Start voice coordinator
python -m robot_app.voice

# Then speak: "detect a bottle"
# The system will:
#   1. Capture audio
#   2. Decompose to task plan
#   3. Call camera for detection
#   4. Execute fetch sequence
EOF

"""
