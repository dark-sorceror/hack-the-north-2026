#!/bin/bash
# Setup RealSense on Raspberry Pi
# Run this with: bash setup_realsense_pi.sh

set -e  # Exit on first error

echo "=========================================="
echo "RealSense SDK Setup for Raspberry Pi 5"
echo "=========================================="

cd ~/VLA-HTN

# Step 1: Ensure venv exists
echo -e "\n[STEP 1] Ensuring virtual environment..."
if [ ! -d "venv" ]; then
    python3 -m venv venv
    echo "✓ Created venv"
else
    echo "✓ venv already exists"
fi

# Step 2: Activate venv
echo -e "\n[STEP 2] Activating virtual environment..."
source venv/bin/activate
echo "✓ venv activated (Python: $(python3 --version))"

# Step 3: Compile RealSense
echo -e "\n[STEP 3] Compiling RealSense SDK (this takes 15-20 minutes)..."
cd librealsense/build

# Check if already compiled
if [ -f "src/CMakeFiles/realsense2.dir/build.make" ]; then
    echo "Starting compilation..."
    make -j4
    echo "✓ Compilation complete"
else
    echo "Build system ready, starting compilation..."
    make -j4
    echo "✓ Compilation complete"
fi

# Step 4: Install (requires sudo)
echo -e "\n[STEP 4] Installing RealSense libraries (requires sudo)..."
sudo make install
echo "✓ Installation complete"

# Step 5: Add PYTHONPATH to ~/.bashrc
echo -e "\n[STEP 5] Configuring PYTHONPATH..."
if ! grep -q "export PYTHONPATH=\$PYTHONPATH:/usr/local/lib" ~/.bashrc; then
    echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib' >> ~/.bashrc
    echo "✓ Added PYTHONPATH to ~/.bashrc"
else
    echo "✓ PYTHONPATH already configured"
fi

# Source bashrc
source ~/.bashrc

# Step 6: Verify installation
echo -e "\n[STEP 6] Verifying installation..."
cd ~/VLA-HTN
source venv/bin/activate

if python3 -c "import pyrealsense2; print('✓ pyrealsense2 version:', pyrealsense2.__version__)" 2>/dev/null; then
    echo "✓ SUCCESS: pyrealsense2 is ready!"
else
    echo "⚠ WARNING: pyrealsense2 import failed, checking /usr/local/lib..."
    ls -la /usr/local/lib/python*/dist-packages/pyrealsense* 2>/dev/null || echo "Not found in standard location"
    echo ""
    echo "Try running this command to add to venv:"
    echo "  pip install /usr/local/lib/python*/pyrealsense*"
fi

# Step 7: Install remaining Python dependencies
echo -e "\n[STEP 7] Installing remaining Python packages..."
pip install -e .
echo "✓ All Python packages installed"

echo -e "\n=========================================="
echo "✓ SETUP COMPLETE!"
echo "=========================================="
echo ""
echo "To verify everything works, run:"
echo "  source ~/VLA-HTN/venv/bin/activate"
echo "  python3 -c 'import pyrealsense2; print(pyrealsense2.__version__)'"
echo ""
