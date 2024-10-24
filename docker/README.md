## Run with Docker

### Build the Docker Image
```sh
docker/build.sh
```

### Start the Container
```sh
docker/run.sh
```

Notes:
 - The network is shared with the host, allowing ROS2 to communicate with ROS2 on the host system.
 - The repository is mounted as a volume, so there's no need to rebuild or restart the container if the source code changes.
 - The `build`, `install`, and `log` folders in the container are mounted as separate volumes, which can be found in the `docker/` folder. This prevents interference with builds on the host system. Additionally, `colcon` will recognize the build from the previous container run. To perform a clean colcon build, delete these folders under `docker/`.

### Join the Running Container
```sh
docker/join.sh
```
Use this command to open another terminal within the running container.