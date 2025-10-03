#!/bin/bash

# Radar-Lidar Calibration Docker Runner
# This script provides easy commands to run the calibration in Docker

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Docker is installed
check_docker() {
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed. Please install Docker first."
        exit 1
    fi
    
    if ! command -v docker compose &> /dev/null; then
        print_error "Docker Compose is not installed. Please install Docker Compose first."
        exit 1
    fi
}

# Create output directories
create_directories() {
    print_status "Creating output directories..."
    mkdir -p calibration_output point_matches_reflector
}

# Build the Docker image
build_image() {
    print_status "Building Docker image..."
    docker build -t radar-lidar-calibration .
}

# Run calibration
run_calibration() {
    print_status "Running radar-lidar calibration..."
    docker run --rm \
        -v "$(pwd)/calibration_output:/app/calibration_output" \
        -v "$(pwd)/point_matches_reflector:/app/point_matches_reflector" \
        radar-lidar-calibration
}

# Run interactive viewer
run_viewer() {
    print_status "Starting interactive viewer..."
    
    # Check if X11 is available
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Linux
        if [ -z "$DISPLAY" ]; then
            print_error "DISPLAY environment variable not set. Please run: export DISPLAY=:0"
            exit 1
        fi
        xhost +local:docker 2>/dev/null || print_warning "Could not set X11 permissions. GUI might not work."
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        if [ -z "$DISPLAY" ]; then
            print_warning "DISPLAY not set. Make sure XQuartz is running and run: export DISPLAY=:0"
        fi
    fi
    
    docker run --rm \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$(pwd)/calibration_output:/app/calibration_output:ro" \
        -v "$(pwd)/point_matches_reflector:/app/point_matches_reflector:ro" \
        -v "$(pwd)/test_data:/app/test_data:ro" \
        radar-lidar-calibration python quick_view.py
}

# Run with docker compose
run_compose() {
    print_status "Running with Docker Compose..."
    docker compose up --build
}

# Clean up
cleanup() {
    print_status "Cleaning up Docker resources..."
    docker system prune -f
    docker image prune -f
}

# Show help
show_help() {
    echo "Radar-Lidar Calibration Docker Runner"
    echo ""
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  build       Build the Docker image"
    echo "  run         Run the calibration"
    echo "  viewer      Run the interactive viewer"
    echo "  compose     Run with Docker Compose"
    echo "  clean       Clean up Docker resources"
    echo "  help        Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 build && $0 run"
    echo "  $0 compose"
    echo "  $0 viewer"
}

# Main script logic
main() {
    check_docker
    create_directories
    
    case "${1:-help}" in
        build)
            build_image
            ;;
        run)
            build_image
            run_calibration
            ;;
        viewer)
            build_image
            run_viewer
            ;;
        compose)
            run_compose
            ;;
        clean)
            cleanup
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $1"
            show_help
            exit 1
            ;;
    esac
}

# Run main function with all arguments
main "$@"
