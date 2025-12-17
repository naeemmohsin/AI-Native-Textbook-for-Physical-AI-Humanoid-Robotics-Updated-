#!/bin/bash
#
# URDF to SDF Conversion Script
#
# This script converts a URDF robot description file to SDF format
# for use in Gazebo Sim (Harmonic).
#
# Usage:
#   ./urdf_to_sdf.sh <input.urdf> [output.sdf]
#
# Example:
#   ./urdf_to_sdf.sh humanoid.urdf humanoid.sdf
#
# Requirements:
#   - Gazebo Sim (Harmonic) installed
#   - gz command available in PATH

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Print usage information
usage() {
    echo "Usage: $0 <input.urdf> [output.sdf]"
    echo ""
    echo "Arguments:"
    echo "  input.urdf   Path to the URDF file to convert"
    echo "  output.sdf   Path for the output SDF file (optional)"
    echo "               If not provided, uses input filename with .sdf extension"
    echo ""
    echo "Example:"
    echo "  $0 humanoid.urdf"
    echo "  $0 humanoid.urdf models/humanoid.sdf"
    exit 1
}

# Check for required arguments
if [ $# -lt 1 ]; then
    echo -e "${RED}Error: No input file specified${NC}"
    usage
fi

INPUT_FILE="$1"
OUTPUT_FILE="${2:-${INPUT_FILE%.urdf}.sdf}"

# Verify input file exists
if [ ! -f "$INPUT_FILE" ]; then
    echo -e "${RED}Error: Input file '$INPUT_FILE' not found${NC}"
    exit 1
fi

# Check if gz command is available
if ! command -v gz &> /dev/null; then
    echo -e "${RED}Error: 'gz' command not found${NC}"
    echo "Please ensure Gazebo Sim (Harmonic) is installed:"
    echo "  sudo apt install ros-humble-ros-gz"
    exit 1
fi

# Check Gazebo version
echo -e "${YELLOW}Checking Gazebo version...${NC}"
gz sim --version

# Perform the conversion
echo -e "${YELLOW}Converting URDF to SDF...${NC}"
echo "  Input:  $INPUT_FILE"
echo "  Output: $OUTPUT_FILE"

gz sdf -p "$INPUT_FILE" > "$OUTPUT_FILE"

# Verify output was created
if [ -f "$OUTPUT_FILE" ]; then
    echo -e "${GREEN}Conversion successful!${NC}"
    echo ""
    echo "Preview of generated SDF:"
    head -20 "$OUTPUT_FILE"
    echo "..."
    echo ""
    echo -e "${GREEN}Full SDF saved to: $OUTPUT_FILE${NC}"
else
    echo -e "${RED}Error: Conversion failed - output file not created${NC}"
    exit 1
fi

# Validate the generated SDF
echo ""
echo -e "${YELLOW}Validating SDF syntax...${NC}"
if gz sdf -k "$OUTPUT_FILE"; then
    echo -e "${GREEN}SDF validation passed!${NC}"
else
    echo -e "${YELLOW}Warning: SDF validation reported issues${NC}"
    echo "The file may still work, but check for any errors above."
fi

echo ""
echo "Next steps:"
echo "  1. Load in Gazebo: gz sim $OUTPUT_FILE"
echo "  2. Or include in a world file using <include> tag"
