FROM python:3.9-alpine

# Install necessary tools
RUN apk add --no-cache bash unzip coreutils

WORKDIR /app

# Copy NAVSIM project ZIP
COPY docker/navsim_installation/navsim.zip /app/navsim.zip

# Copy python dependencies file
COPY docker/navsim_installation/dependencies.zip /app/dependencies.zip

# Copy installation script
COPY docker/navsim_installation/install.py /app/install.py

# Run installation with interactive support
CMD ["python", "/app/install.py"]