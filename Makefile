x3-docker-build-push:
	docker build -t huajuan6848/x3-54demo-ros-foxy:0.0.9 -f docker/x3/Dockerfile .
	docker push huajuan6848/x3-54demo-ros-foxy:0.0.9
limo-docker-build-push:
	docker build -t huajuan6848/limo-54demo-ros-foxy:0.0.9 -f docker/limo/Dockerfile .
	docker push huajuan6848/limo-54demo-ros-foxy:0.0.9
limo-dockerx-build-push:
	docker buildx build --platform linux/amd64,linux/arm64 -t huajuan6848/limo-54demo-ros-foxy:0.0.9 -f docker/limo/Dockerfile --push .
x3-dockerx-build-push:
	docker buildx build --platform linux/amd64,linux/arm64 -t huajuan6848/x3-54demo-ros-foxy:0.0.9 -f docker/x3/Dockerfile --push .