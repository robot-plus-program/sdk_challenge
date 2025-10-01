#!/bin/bash

# ======================= CONFIGURATION =======================
# Define allowed versions
VALID_UBUNTU=("20.04" "22.04" "24.04")
VALID_CUDA=("11.1.1" "11.7.1" "12.1.0" "12.4.1" "12.6.3")
VALID_ROS=("foxy" "humble" "jazzy")

# Default values
UBUNTU_VERSION="22.04"
CUDA_VERSION="12.6.3"
ROS_DISTRO="humble"

# ======================= ARGUMENT PARSING =======================
for arg in "$@"; do
    case $arg in
        --ubuntu=*)
            UBUNTU_VERSION="${arg#*=}"
            shift
            ;;
        --cuda=*)
            CUDA_VERSION="${arg#*=}"
            shift
            ;;
        --ros=*)
            ROS_DISTRO="${arg#*=}"
            shift
            ;;
        *)
            echo "❌ Unknown argument: $arg"
            echo "Usage: $0 [--ubuntu=<version>] [--cuda=<version>] [--ros=<distro>]"
            exit 1
            ;;
    esac
done

# ======================= INPUT VALIDATION =======================
# Function to check if a value exists in an array
function is_valid() {
    local value=$1
    shift
    local valid_list=("$@")
    for item in "${valid_list[@]}"; do
        if [[ "$item" == "$value" ]]; then
            return 0
        fi
    done
    return 1
}

# Validate Ubuntu version
if ! is_valid "$UBUNTU_VERSION" "${VALID_UBUNTU[@]}"; then
    echo "❌ Error: Invalid Ubuntu version: $UBUNTU_VERSION"
    echo "✅ Supported versions: ${VALID_UBUNTU[*]}"
    exit 1
fi

# Validate CUDA version
if ! is_valid "$CUDA_VERSION" "${VALID_CUDA[@]}"; then
    echo "❌ Error: Invalid CUDA version: $CUDA_VERSION"
    echo "✅ Supported versions: ${VALID_CUDA[*]}"
    exit 1
fi

# Validate ROS version
if ! is_valid "$ROS_DISTRO" "${VALID_ROS[@]}"; then
    echo "❌ Error: Invalid ROS distribution: $ROS_DISTRO"
    echo "✅ Supported versions: ${VALID_ROS[*]}"
    exit 1
fi

# ======================= BUILD PROCESS =======================
DOCKERFILE=Dockerfile
IMAGE_NAME="ketirobotvision/ketinet:v1"

echo "======================================="
echo "🚀 Building Docker Image"
echo "Ubuntu:    ${UBUNTU_VERSION}"
echo "CUDA:      ${CUDA_VERSION}"
echo "ROS 2:     ${ROS_DISTRO}"
echo "Tag:       ${IMAGE_NAME}"
echo "======================================="

# Build Docker Image
sudo docker build --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
                  --build-arg CUDA_VERSION=${CUDA_VERSION} \
                  --build-arg ROS_DISTRO=${ROS_DISTRO} \
                  -f ${DOCKERFILE} -t ${IMAGE_NAME} .

# Print success message
if [ $? -eq 0 ]; then
    echo "✅ Docker Image Built Successfully: ${IMAGE_NAME}"
else
    echo "❌ Docker Build Failed!"
fi

