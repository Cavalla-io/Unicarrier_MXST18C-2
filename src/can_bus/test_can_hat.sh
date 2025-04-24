#!/bin/bash

# Color definitions
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
TEST_SCRIPT="${SCRIPT_DIR}/tests/test_can_hat.py"

# Check if Python 3 is installed
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}Python 3 is not installed. Please install it before continuing.${NC}"
    exit 1
fi

# Check if python-can is installed
python3 -c "import can" &> /dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}The python-can library is not installed. Installing it now...${NC}"
    pip3 install python-can
    if [ $? -ne 0 ]; then
        echo -e "${RED}Failed to install python-can. Please install it manually:${NC}"
        echo "pip3 install python-can"
        echo -e "${YELLOW}Continuing without python-can tests...${NC}"
    fi
fi

# Check if the test script exists
if [ ! -f "$TEST_SCRIPT" ]; then
    echo -e "${RED}Test script not found at: ${TEST_SCRIPT}${NC}"
    exit 1
fi

# Check if we need to run with sudo
if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}Note: Some tests may require sudo privileges.${NC}"
    echo -e "${YELLOW}If tests fail, try running this script with sudo.${NC}"
fi

# Execute the Python test script with all arguments passed to this script
echo -e "${GREEN}Running WaveShare 2-CH CAN HAT tests...${NC}"
python3 "$TEST_SCRIPT" "$@"

# Exit with the same status as the Python script
exit $? 