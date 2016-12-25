# -*- coding: utf-8 -*-
import sys
import PyQt4.QtCore as QtCore
import PyQt4.QtGui as QtGui

def print_state(state):
    if state:
        print("enabled")
    else:
        print("disabled")

        
def main():
    app = QtGui.QApplication(sys.argv)

    main_window = QtGui.QMainWindow()
    main_window.setWindowTitle("SIGNAL - SLOT Direct connection")

    panel = QtGui.QWidget()

    checkbox = QtGui.QCheckBox("The Checkbox", parent=panel)
    button = QtGui.QPushButton("Push to change checkbox state", parent=panel)
    
    layout = QtGui.QVBoxLayout()
    layout.addWidget(checkbox)
    layout.addWidget(button)

    panel.setLayout(layout)

    button.clicked.connect(checkbox.toggle)
    checkbox.stateChanged.connect(print_state)
        
    main_window.setCentralWidget(panel)
    main_window.show()

    app.exec_()


if __name__ == "__main__":
    main()
    
