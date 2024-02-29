docker-build:
	docker build -t huajuan6848/x3-54demo-ros-foxy:0.0.1 -f docker/x3/Dockerfile .
docker-push:
	docker push huajuan6848/x3-54demo-ros-foxy:0.0.1