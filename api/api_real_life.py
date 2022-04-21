import sys
from xml.etree.ElementTree import tostring


from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from socket import *

servidor = "127.0.0.1"
porta = 43211

obj_socket = socket(AF_INET, SOCK_DGRAM)
obj_socket.connect((servidor, porta))
saida = ""

def greeting():
    """Slot function."""
    message = msg.text()
    if message == "play":
        msg.setText("stop")
    else:
        msg.setText("play")

    pacote = message
    obj_socket.sendto(pacote.encode(), (servidor, porta))

app = QApplication(sys.argv)
window = QWidget()
window.setWindowTitle('Signals and slots')
layout = QVBoxLayout()

btn = QPushButton('Play/Pause')
btn.clicked.connect(greeting)  # Connect clicked to greeting()

layout.addWidget(btn)
msg = QLabel('Stop')
layout.addWidget(msg)
window.setLayout(layout)
window.show()
sys.exit(app.exec_())




# while saida != "X":
#     msg = input("Sua mensagem: ")
#     obj_socket.sendto(msg.encode(), (servidor, porta))
