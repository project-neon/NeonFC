import threading
import time

class SendDataThread(threading.Thread):
    def __init__(self, api_instance, info_api_instance):
        super().__init__()
        self.api = api_instance
        self.info_api = info_api_instance

    def run(self):
        while True:
            self.api.send_data(self.info_api)
            time.sleep(5)  