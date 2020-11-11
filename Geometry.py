from abc import ABC

class Geometry(ABC):
    def __init__(self):
        self.entities = []

    def getEntities(self):
        pass

    def scurvePath(self):
        pass