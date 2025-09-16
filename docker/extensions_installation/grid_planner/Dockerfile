FROM python:3.9-alpine

# Install necessary tools
RUN apk add --no-cache bash coreutils

WORKDIR /app

# Copy the grid planner extension
COPY extensions/grid_planner/ /app/extension/

# Copy installation script
COPY docker/extensions_installation/grid_planner/install.py /app/install.py

# Run installation
CMD ["python", "/app/install.py"]