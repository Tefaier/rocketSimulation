class PrimitiveCamera:
    def __init__(self, wight_coord=0, height_coord=1, wight_axis_name="x", height_axis_name="y"):
        self.wight_coord = wight_coord
        self.height_coord = height_coord
        self.wight_axis_name = wight_axis_name
        self.height_axis_name = height_axis_name
        self.orientation = (-1) ** ((wight_coord + 1) % 3 != height_coord)

    def first_coord(self):
        return self.wight_coord

    def second_coord(self):
        return self.height_coord

    def third_coord(self):
        return ({0, 1, 2} - {self.second_coord(), self.first_coord()}).pop()

    def first_axis_name(self):
        return self.wight_axis_name

    def second_axis_name(self):
        return self.height_axis_name


camera_xy = PrimitiveCamera(0, 1, 'x', 'y')
camera_xz = PrimitiveCamera(0, 2, 'x', 'z')
camera_yz = PrimitiveCamera(1, 2, 'y', 'z')