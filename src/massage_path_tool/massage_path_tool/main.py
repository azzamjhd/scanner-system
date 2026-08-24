import argparse, sys
def main():
    parser = argparse.ArgumentParser(description='Massage Path Authoring Tool')
    parser.add_argument('--image', default=None)
    parser.add_argument('--session', default=None)
    args = parser.parse_args()
    from PyQt6.QtWidgets import QApplication
    app = QApplication(sys.argv)
    from massage_path_tool.ui.main_window import MainWindow
    win = MainWindow(image_path=args.image, session_path=args.session)
    win.show()
    sys.exit(app.exec())
if __name__ == '__main__': main()
