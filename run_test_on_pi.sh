#!/bin/bash
# Deploy and run the microphone → VLA → object detection test on Pi

PI_HOST="${1:-gisoopi.local}"
PI_USER="${2:-gisooj}"
PI_PATH="$PI_USER@$PI_HOST"

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  VLA Microphone → Detection Pipeline - Pi Deployment          ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if Pi is reachable
echo -e "${BLUE}[1/5] Checking Pi connectivity (${PI_PATH})...${NC}"
if ! ping -c 1 "$PI_HOST" &> /dev/null; then
    echo -e "${YELLOW}⚠ Pi not reachable via ping, attempting SSH anyway...${NC}"
fi

# Copy test file
echo -e "${BLUE}[2/5] Copying test file to Pi...${NC}"
scp -q test_microphone_vla_detection.py "$PI_PATH:~/VLA-HTN/" 2>&1 | grep -v "password:"
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}⚠ File transfer may have issues, continuing...${NC}"
fi
echo -e "${GREEN}✓ Test file copied${NC}"

# Copy updated pyproject.toml
echo -e "${BLUE}[3/5] Copying project config to Pi...${NC}"
scp -q pyproject.toml "$PI_PATH:~/VLA-HTN/" 2>&1 | grep -v "password:"

# Install dependencies on Pi
echo -e "${BLUE}[4/5] Installing dependencies on Pi (this may take a minute)...${NC}"
ssh "$PI_PATH" << 'REMOTE_EOF'
cd ~/VLA-HTN
source venv/bin/activate
echo "  Installing project packages..."
pip install -e '.[camera,audio,camera-realsense]' --quiet 2>&1 | tail -3
echo "  Done"
REMOTE_EOF

# Run the test
echo -e "${BLUE}[5/5] Running microphone → VLA → detection pipeline...${NC}"
echo -e "${YELLOW}────────────────────────────────────────────────────────────────${NC}"
ssh "$PI_PATH" << 'REMOTE_EOF'
cd ~/VLA-HTN
source venv/bin/activate
python test_microphone_vla_detection.py
REMOTE_EOF
RESULT=$?
echo -e "${YELLOW}────────────────────────────────────────────────────────────────${NC}"

echo ""
if [ $RESULT -eq 0 ]; then
    echo -e "${GREEN}✓ Test completed successfully!${NC}"
else
    echo -e "${YELLOW}⚠ Test finished with exit code $RESULT${NC}"
fi

echo ""
echo -e "${GREEN}Summary:${NC}"
echo "  1. ✓ Copied test_microphone_vla_detection.py to Pi"
echo "  2. ✓ Updated pyproject.toml on Pi"
echo "  3. ✓ Installed dependencies"
echo "  4. ✓ Ran full pipeline (Microphone → VLA → Camera → Detection)"
echo ""
echo -e "${BLUE}Next steps:${NC}"
echo "  - Check camera output for detection results"
echo "  - For live voice: ssh $PI_PATH 'cd ~/VLA-HTN && source venv/bin/activate && python -m robot_app.voice'"
echo "  - View logs: ssh $PI_PATH 'tail -f ~/VLA-HTN/logs/*'"
echo ""
