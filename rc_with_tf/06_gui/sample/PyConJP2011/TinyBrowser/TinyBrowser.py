# -*- coding: utf-8 -*-
"""
Python Hack-a-thon 2010.11 / PyQt4 Hands on
About QtWebKit 02 : シンプルなブラウザとして機能するようにしましょう
"""

import sys
import PyQt4.QtCore as QtCore
import PyQt4.QtGui as QtGui
import PyQt4.QtWebKit as QtWebKit
from MainWindowUI import Ui_MainWindow

class MainWindow(QtGui.QMainWindow, Ui_MainWindow):
    def __init__(self, *args, **kw):
        QtGui.QMainWindow.__init__(self, *args, **kw)

        self.setupUi(self)

        # リンク上にカーソルが乗った時のシグナルをつなぐ
        self.web_page = self.webView.page()
        self.web_page.linkHovered.connect(self.showUrlInStatusBar)

        # 進む＆戻るボタンが押されたときのシグナルをつなぐ
        self.back_button.clicked.connect(self.webView.back)
        self.forward_button.clicked.connect(self.webView.forward)

        # アドレスバー部分を初期化する
        self.setAddressBarUrl(self.webView.url())

        # StatusBarに表示するProgressBarを準備する
        self.load_progress_bar = QtGui.QProgressBar()
        self.statusBar().addPermanentWidget(self.load_progress_bar)
        self.statusBar().setSizeGripEnabled(False)

        # ProgressBar表示サイズの設定(固定幅にする)
        self.load_progress_bar.setSizePolicy(
            QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Preferred))
        progress_bar_size = self.load_progress_bar.size()
        progress_bar_size.setWidth(200)
        self.load_progress_bar.setMinimumSize(progress_bar_size)
        self.load_progress_bar.resize(progress_bar_size)



    @QtCore.pyqtSlot(bool, name="on_actionQuit_triggered")
    def quitApplication(self, checked=False):
        sys.exit(QtGui.QApplication.instance().quit())


    @QtCore.pyqtSlot(name="on_address_bar_returnPressed")
    def loadNewPage(self):
        url = self.address_bar.text()
        if not QtCore.QRegExp(r"^(https?)|(ftp)://").exactMatch(url):
            url_str = "http://" + url

        self.webView.setUrl(QtCore.QUrl(url_str))


    @QtCore.pyqtSlot(QtCore.QUrl, name="on_webView_urlChanged")
    def setAddressBarUrl(self, url):
        self.address_bar.setText(url.toString())


    def showUrlInStatusBar(self, link, title, textContent):
        self.statusBar().showMessage(link)


    @QtCore.pyqtSlot(QtCore.QUrl, name="on_webView_linkClicked")
    def setAddressBarAndLoadPage(self, url):
        self.address_bar.setText(url.toString())
        self.webView.load(url)


    @QtCore.pyqtSlot(name="on_webView_loadStarted")
    def resetProgressBar(self):
        self.load_progress_bar.setValue(0)


    @QtCore.pyqtSlot(int, name="on_webView_loadProgress")
    def setProgressBarValue(self, value):
        self.load_progress_bar.setValue(value)
    

def main():
    application = QtGui.QApplication(sys.argv)

    web_settings = QtWebKit.QWebSettings.globalSettings()
    web_settings.setAttribute(QtWebKit.QWebSettings.PluginsEnabled, False)
    web_settings.setAttribute(QtWebKit.QWebSettings.JavaEnabled, True)
    web_settings.setAttribute(QtWebKit.QWebSettings.JavascriptEnabled, True)
    web_settings.setAttribute(QtWebKit.QWebSettings.LocalContentCanAccessRemoteUrls, True)

    main_window = MainWindow()
    main_window.show()

    application.exec_()


if __name__ == "__main__":
    main()
