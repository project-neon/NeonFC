build:
	docker build --no-cache -t neon-fc .
run:
	docker run --net=host $(params) neon-fc