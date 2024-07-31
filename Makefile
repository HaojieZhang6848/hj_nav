# limo-dockerx-build-push:
# 	docker buildx build --platform linux/arm64 -t huajuan6848/limo-54demo-ros-foxy:0.0.9-5 -f docker/limo/Dockerfile --push .
x3-pi-dockerx-build-push:
	docker buildx build --platform linux/arm64 -t huajuan6848/x3-54demo-ros-foxy:0.0.11-pi -f docker/x3/Dockerfile.pi --push .
x3-jetson-dockerx-build-push:
	docker buildx build --platform linux/arm64 -t huajuan6848/x3-54demo-ros-foxy:0.0.11-jetson -f docker/x3/Dockerfile.jetson --push .
x3-base-dockerx-build-push:
	docker buildx build --platform linux/arm64 -t huajuan6848/x3_ros2:0.0.1-orig-nav -f docker/x3/Dockerfile.base --push .