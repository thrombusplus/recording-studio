import os


class FileManager:
    def __init__(self, file_path):
        self.file_path = file_path
        self.file = None

    def get_subfodlers(self):
        self.subfolder = []
        files = os.listdir(self.file_path)
        for file in files:
            if os.path.isdir(os.path.join(self.file_path, file)):
                self.subfolder.append(file)
        return self.subfolder