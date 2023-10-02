This is a sample ROS2 workspace for IARC Mission 9 and TAC Challenge 2022

# Overview
Previously, we have had trouble that code builds, but only on one spesific computer. By building within a Docker image, we make sure that all dependencies are included and work - not just on one PC, but on all relevant computers.

Because we use a NVIDIA Jetson AGX for Mission 9, and likely a similar platform for TAC, the Dockerfile has to work for both `amd64` and `arm64` architectures.

## Useful features
- Pre-configured development container for simple development in VSCode
  - Works on most platforms including Ubuntu, Mac (Intel & M1) and Windows
  - Automatic formating and linting of C++ using clang-format and clang-tidy
  - Works in both Docker root and rootless modes (just change `remoteUser` in `.devcontainer/devcontainer.json`)
- Github Actions make sure that the code builds on all systems
  - Automatic builds for `amd64` and `arm64`
  - Ensures that tests pass with `colcon test`
- Uploads docker image to GitHub Container Registry
  - By default turned on for `main` and `devel` branches


# Using the image
## Prerequisites
### Install and run Docker
Install and run Docker. It is available for most platforms, including Ubuntu, MacOS (Intel & M2) and Windows.

On Linux, it is recommended to run Docker in rootless mode.

### Login to GitHub Container Registry
**NB!** Some of the docker images used in the build process are private for the AscendNTNU organization. In order to pull the images, you need to authenticate to GitHub Container Registry using your Ascend GitHub account. See the following guide on how to authenticate: [Authenticating To the Container Registry](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-to-the-container-registry)

## Developing in VSCode
This requires installing the [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.

**NB!** If using docker on a Mac, follow the same instructions as for root-mode. I.e. uncomment the following line in `.devcontainer/devcontainer.json`:
```javascript
// If running Docker in root mode, uncomment the "remoteUser" line.
// If using Docker rootless, leave it commented out.
// More info: https://aka.ms/vscode-remote/containers/non-root.
"remoteUser": "ros",
```

If using VSCode, simply open the development container : `CTRL + SHIFT + P  ... "Remote-Containers: Rebuild and Reopen in Container"`

## Running multiple ROS2 containers together
If you want to run multiple ROS2 containers that should be able to communicate, they need to be added to the same Docker network. To do this:
1. Create a docker network: `docker network create ros2-network-name`
2. Add the container to the network, this can be done either
    - for the devcontainer, by uncommenting the `network`-line in `devcontainer.json`
    - when starting the container by adding the option `--network=ros2-network-name` to the run command
    - while the container is running, by running `docker network connect ros2-network-name CONTAINER`.
    - in a docker compose file by adding `network: ros2-network-name`

# Developing with this template
## Requirements
- `docker/Dockerfile` must contain a stage named `deploy` that features a fully built ROS2 workspace on both `amd64` and `arm64` architectures.
    - If the program is **only ever** supposed to run on one of the architectures, remove the the unused architecure file from `TARGET_PLATFORMS` in `.github/worklfows/build-and-test.yml` and `.github/worklfows/build-and-upload.yml`
  
## Installing dependencies
Any additional dependencies should be installed in the `Dockerfile`. If there are buld stages that take a long time, consider uploading the image to our GitHub Container Registry, so that others can download a pre-built image.