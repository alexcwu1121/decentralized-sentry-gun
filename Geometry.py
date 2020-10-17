from abc import ABC

class Geometry(ABC):
    def __init__(self):
        self.entities = []

    def getEntities(self):
        return self.entities
