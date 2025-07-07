class Buoy():
    def __init__(self, id: int, color: str, centroid: tuple, limit: int, counter: int = 0):
        self.id = id
        self.color = color
        self.centroid = centroid

        self.limit = limit
        self.limit_counter = counter

        self.distance = None
        self.angle = None

        self.name = ""

        self.set_name()

    def __str__(self):
        return f"{self.name}, Distance: {self.distance}, Angle: {self.angle}"

    def get_id(self):
        return self.id

    def get_color(self):
        return self.color
    
    def set_distance(self, distance):
        self.distance = distance

    def get_distance(self):
        return self.distance
    
    def set_angle(self, angle):
        self.angle = angle

    def get_angle(self):
        return self.angle
    
    def set_name(self):
        if self.id % 2 == 0:
            self.name = f"Buoy {self.id} - {self.color} - Left"
        else:
            self.name = f"Buoy {self.id} - {self.color} - Right"