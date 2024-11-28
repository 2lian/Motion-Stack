#### Install Docker

Follow the steps in [Isaac Sim Container Installation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) to configure Docker with NVIDIA GPU support, or you can try [`this script`](https://github.com/AndrejOrsula/isaac_sim_rs/blob/main/.docker/host/install_docker.bash) which shoud do the whole thing for you.


By running the Docker container, you are implicitly agreeing to the [NVIDIA Omniverse EULA](https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html). If you do not agree to this license agreement, do not use this container.

#### Build Image

In order to pull the base [Isaac Sim](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim) image from the [NGC registry](https://ngc.nvidia.com), you must first create an account and [generate an API key](https://ngc.nvidia.com/setup/api-key) in order to authenticate with the registry.

```bash
docker login nvcr.io
```

To build a new Docker image from [`Dockerfile`](Dockerfile), you can run [`.docker/build.bash`](.docker/build.bash) as shown below.

```bash
isaac.docker/build.bash
```

#### Run Container

To run the Docker container, you can use [`.docker/run.bash`](.docker/run.bash) as shown below.

```bash
isaac.docker/run.bash
```

#### Run Dev Container

To run the Docker container in a development mode (the src/ folder and the scripts in the project root are mounted as read-only volumes), you can use the `--dev` as shown below.

```bash
isaac.docker/run.bash --dev
```

As an alternative, VS Code users familiar with [Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers) can modify the included [`.devcontainer/devcontainer.json`](.devcontainer/devcontainer.json) to their needs. For convenience, [`.devcontainer/open.bash`](.devcontainer/open.bash) script is available to open this repository as a Dev Container in VS Code.

```bash
.devcontainer/open.bash
```

#### Join Container

To join a running Docker container from another terminal, you can use [`isaac.docker/join.bash`](.docker/join.bash) as shown below.

```bash
isaac.docker/join.bash
```