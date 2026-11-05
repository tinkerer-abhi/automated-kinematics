# Automated Kinematics & Multi-Mode Oscillator Analytics Suite
# Production Dockerfile for Web / Cloud Hosting

FROM python:3.11-slim

WORKDIR /app

# Install minimal system build dependencies for numpy/scipy
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application directories
COPY python ./python
COPY web ./web
COPY cad ./cad
COPY docs ./docs
COPY firmware ./firmware

# Create data directory for CSV logs and history
RUN mkdir -p python/data

# Environment configuration
ENV PORT=8000
ENV HOST=0.0.0.0
EXPOSE 8000

# Start server using the dynamic PORT provided by cloud environment (Render, Railway, Fly.io, etc.)
CMD ["sh", "-c", "python -m uvicorn web_server:app --app-dir python --host 0.0.0.0 --port ${PORT:-8000}"]
