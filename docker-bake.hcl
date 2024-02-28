variable "IMAGE_NAME" {
  default = "ars-408"
}

variable "CONTAINER_REGISTRY" {
  default = "registry.gitlab.com/polymathrobotics/sensors"
}

variable "LOCAL_PLATFORM" {
  default = regex_replace("${BAKE_LOCAL_PLATFORM}", "^(darwin)", "linux")
}

target "_common" {
  dockerfile = "Containerfile"
  args = {
    BASE_IMAGE = "docker.io/polymathrobotics/ros_base:humble"
  }
  tags = ["${CONTAINER_REGISTRY}/${IMAGE_NAME}:humble"]
  labels = {
    "org.opencontainers.image.source" = "https://github.com/polymathrobotics/sensors/ars-408"
    "org.opencontainers.image.licenses" = "Apache-2.0"
    "org.opencontainers.image.description" = "Driver for Continental ARS-408"
    "org.opencontainers.image.title" = "${IMAGE_NAME}"
    "dev.polymathrobotics.image.readme-filepath" = "README.md"
  }
}

target "local" {
  inherits = ["_common"]
  platforms = ["${LOCAL_PLATFORM}"]
}

target "arm64" {
  inherits = ["_common"]
  platforms = ["linux/arm64/v8"]
  tags = ["${CONTAINER_REGISTRY}/${IMAGE_NAME}/arm64:humble"]
}

target "default" {
  inherits = ["_common"]
  platforms = ["linux/amd64", "linux/arm64/v8"]
}
