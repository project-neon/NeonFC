FROM ubuntu:22.04
WORKDIR /
COPY requirements.txt ./
EXPOSE 5000
RUN apt-get update \
	&& apt-get -y install --no-install-recommends python3 pip cmake \
	&& pip install \
		protobuf==3.20.3 \
		numpy \
		scipy \
		matplotlib \
		pyqtgraph \
		PyQt5 \
		NeonPathPlanning \
		pyserial \
		rc-robosim
COPY . .
CMD ["python3", "main_real_life.py"]
