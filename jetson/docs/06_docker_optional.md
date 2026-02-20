# 06 - Docker (Optional)

Docker is optional. Native ROS install is the default path for minimal bring-up.

## Install

```bash
cd ~/robot-sink
./jetson/scripts/setup_docker_optional.sh
```

After install, re-login so your user gets `docker` group permissions.

## Basic Validation

```bash
docker --version
docker info
```

## Suggested Uses

- Keep experimental perception dependencies isolated
- Run alternate ROS environments without changing host baseline
- Build reproducible dev/test containers
