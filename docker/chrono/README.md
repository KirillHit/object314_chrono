# Chrono Docker Image

> [!Note]
> This image is available on Docker Hub as `kirillhit/project_chrono:latest`:
> https://hub.docker.com/repository/docker/kirillhit/project_chrono/general

Use the published image when you do not need to rebuild Chrono locally:

```bash
docker pull kirillhit/project_chrono:latest
docker run --rm -it --gpus all kirillhit/project_chrono:latest
```

## Local Build

This Docker context builds Project Chrono `10.0.0` from source, using Ubuntu 24.04, CUDA 12.8, ROS 2 Jazzy, and the Chrono Docker layout from `contrib/docker`.

The build requires the NVIDIA OptiX SDK installer because the Chrono Sensor module is enabled. Download the Linux x86_64 OptiX installer from NVIDIA, place it in `docker/chrono/data/`, and pass it through `OPTIX_SCRIPT`.

`OPTIX_SCRIPT` is the only build argument. Ubuntu 24.04, CUDA 12.8, ROS 2 Jazzy, and Chrono 10.0.0 are fixed in the Dockerfiles.

Expected layout:

```text
docker/chrono/
  main.dockerfile
  python.dockerfile
  chrono.dockerfile
  cuda.dockerfile
  ros.dockerfile
  entrypoint.sh
  requirements.txt
  data/
    NVIDIA-OptiX-SDK-<version>-linux64-x86_64.sh
  snippets/
    ch_*.dockerfile
```

Build command:

```bash
docker build \
  -f docker/chrono/main.dockerfile \
  docker/chrono \
  --build-arg OPTIX_SCRIPT=data/NVIDIA-OptiX-SDK-9.1.0-linux64-x86_64.sh \
  -t project_chrono:10.0.0
```

Run the locally built image:

```bash
docker run --rm -it --gpus all project_chrono:10.0.0
```
