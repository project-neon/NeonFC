FROM python:3.5
WORKDIR /
COPY requirements.txt ./
EXPOSE 5000
RUN pip install --no-cache-dir -r requirements.txt
COPY . .
CMD ["python3", "/main.py"]
