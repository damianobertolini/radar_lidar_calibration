# Use Python 3.9 slim image as base
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libgthread-2.0-0 \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file first for better caching
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the entire project
COPY . .

# Create output directories
RUN mkdir -p point_matches_reflector calibration_output

# Set environment variables
ENV PYTHONPATH=/app
ENV DISPLAY=:0

# Expose port for potential web interface (optional)
EXPOSE 8080

# Default command
CMD ["python", "solve_calibration.py"]
