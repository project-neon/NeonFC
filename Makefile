build:
	docker build -t neon-fc .
run:
	docker run --net=host --name neon-fc python