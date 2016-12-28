# -*- coding: utf-8 -*-
import sys
import os
import PyQt4.QtCore as QtCore
import PyQt4.QtGui as QtGui
from PyQt4.phonon import Phonon


class VideoPlayerWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent=parent)
        self.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Preferred)


    def setup_ui(self, media_file_name):
        self.player = Phonon.VideoPlayer(Phonon.VideoCategory, parent=self)
        self.player.load(Phonon.MediaSource(media_file_name))

        media_object = self.player.mediaObject()
        media_object.setTickInterval(100)
        media_object.stateChanged.connect(self.state_changed)

        self.play_pause_button = QtGui.QPushButton(">")
        self.play_pause_button.clicked.connect(self.play_clicked)
        
        self.slider = Phonon.SeekSlider(self.player.mediaObject() , parent=self)

        topLayout = QtGui.QVBoxLayout(self)
        topLayout.addWidget(self.player)
        
        layout = QtGui.QHBoxLayout(self)
        layout.addWidget(self.play_pause_button)
        layout.addWidget(self.slider)
        
        topLayout.addLayout(layout)
        
        self.setLayout(topLayout)


    def play_clicked(self):
        if self.player.mediaObject().state() == Phonon.PlayingState:
            self.player.pause()
        else:
            self.player.play()


    def state_changed(self, new, old):
        if new == Phonon.PlayingState:
            self.play_pause_button.setText("||")
        else:
            self.play_pause_button.setText(">")



def main():
    app = QtGui.QApplication(sys.argv)
    main_window = QtGui.QMainWindow()
    main_window.setWindowTitle("Tiny Media Player")

    if len(sys.argv) <= 1:
        print("usage : %s media_file_name" % sys.argv[0])
        sys.exit(0)

    video_player_widget = VideoPlayerWidget()
    video_player_widget.setup_ui(sys.argv[1])
    
    main_window.setCentralWidget(video_player_widget)
    main_window.show()

    app.exec_()
    

if __name__ == "__main__":
    main()
