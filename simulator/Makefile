# Docker stuff
APP_NAME=urab-sim
PORT=5900

build: ## Build the container
	docker build -t $(APP_NAME) .

build-nc: ## Build the container without caching
	docker build --no-cache -t $(APP_NAME) .

run: ## Run container
	# Access docker stuff at localhost:6080 (type into your browser)
	# Volumes: -v directoryA:directoryB means the stuff stored in directoryB in the docker container
	#				can also be seen from directoyA on your computer.
	#				aka the pwd one shares the git repo between the docker container and your computer
	docker run -it --rm -p 6080:80 -v="/tmp/.ros/:/root/.ros/" \
		-v=$$(pwd):/root/catkin_ws/src/ --name=${APP_NAME} ${APP_NAME}

run-more: ## Run container
	# Beware if you have less than --cpus cpus on your computer
	# Access docker stuff at localhost:6080 (type into your browser)
	# Volumes: -v directoryA:directoryB means the stuff stored in directoryB in the docker container
	#				can also be seen from directoyA on your computer.
	#				aka the pwd one shares the git repo between the docker container and your computer
	docker run -it --rm -p 6080:80 -v="/tmp/.ros/:/root/.ros/" \
		-v=$$(pwd):/root/catkin_ws/src/ --cpus=1.5 --name=${APP_NAME} ${APP_NAME}

clean: ## Remove a container, running or not
	docker rm $(APP_NAME) -f
