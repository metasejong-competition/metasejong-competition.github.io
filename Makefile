# /*******************************************************************************
#  * Copyright 2025 AISL Sejong University, Korea
#  *
#  *	Licence: TBD
#  *******************************************************************************/

.PHONY: help setup run build-and-run down build-only release clean
.SILENT: help

help:
	echo "See README.md in this folder"

# This tool now only supports compose V2, aka "docker compose" as it has replaced to old docker-compose tool.
DOCKER_COMPOSE=docker compose

# The default value for USE_VENV as none
USE_VENV := none

# If conda is activated, set USE_VENV to 'conda'.
ifneq ($(CONDA_DEFAULT_ENV),)
    USE_VENV := conda
endif

# If venv is activated, set USE_VENV to 'venv'. 
ifneq ($(VIRTUAL_ENV),)
    USE_VENV := venv
endif

# Setup only needs to be executed once.
# The setup process includes logging into the NVIDIA Docker repository and configuring the X server.
setup:
	docker login nvcr.io
	xhost +local:
# Run the simulation application in development mode.
# Launch IsaacSim and start the developed simulation application.
run:
	${DOCKER_COMPOSE} -f docker-compose.yml up -d

# Same as 'run', but forces the Docker image to be built.
# Used when the Docker image needs to be rebuilt.
build-and-run:
	${DOCKER_COMPOSE} -f docker-compose.yml up -d --build

# Stop the running Docker container.
down:
	${DOCKER_COMPOSE} -f docker-compose.yml down

# Stop the running Docker container and delete all temporary volumes.
clean:
	${DOCKER_COMPOSE} -f docker-compose.yml down -v

# Build the final Docker image for submission.
# This is an intermediate step for 'release'. do not need to execute this command manually.
build-for-release:
	${DOCKER_COMPOSE} -f docker-compose-release.yml build --no-cache

# Create a tar file for the final submission.
release: build-for-release
	docker save -o metasejong-2025-release.tar metasejong-2025:release
