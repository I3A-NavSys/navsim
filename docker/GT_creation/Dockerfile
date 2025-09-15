FROM python:3.9-alpine

# Install necessary tools
RUN apk add --no-cache bash unzip coreutils

# Set working directory
WORKDIR /app

# Create directories
# RUN mkdir -p /app/assets
RUN mkdir -p /output

# Copy your existing ZIP file (place your ZIP file here)
COPY docker/GT_creation/navsim.zip /app/navsim.zip

# Copy the grid-planner extension
COPY extensions/grid_planner/ /app/extension/

# Copy the interactive deployment script
COPY docker/GT_creation/interactive_deploy.py /app/interactive_deploy.py

# Set the interactive deployment script as the main command
CMD ["python", "/app/interactive_deploy.py"]