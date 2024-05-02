import math
class FormationKeeper:
    def __init__(self, cart: str, my_x: float, my_y: float, standard_x: float, standard_y: float, last_my_x: float, last_my_y: float):
        self.cart = cart
        self.my_x = my_x
        self.my_y = my_y
        self.standard_x = standard_x
        self.standard_y = standard_y
        self.last_my_x = last_my_x
        self.last_my_y = last_my_y
        self.target_x = None
        self.target_y = None
        self.linear_x = None
        self.angular_z = None
        self.distance = None

    def process(self):
        self.distance = math.sqrt(pow(self.standard_x - self.my_x, 2) + pow(self.standard_y - self.my_y, 2))

        if self.distance < 3 or self.distance > 7:
            self.linear_x = 1
        else:
            self.linear_x = 0.5









