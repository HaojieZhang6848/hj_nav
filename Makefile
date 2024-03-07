docker-build:
	docker build -t huajuan6848/x3-54demo-ros-foxy:0.0.1 -f docker/x3/Dockerfile .
docker-push:
	docker push huajuan6848/x3-54demo-ros-foxy:0.0.1
limo-docker-build-push:
	docker buildx build --platform linux/amd64,linux/arm64 -t huajuan6848/limo-54demo-ros-foxy:0.0.2 -f docker/limo/Dockerfile --push .