class GearReductionDeg:
	def __init__(self,reduction):
		self.scale = reduction/360
	def pos(self, deg):
		return self.scale * deg
	def vel(self, deg_per_sec):
		return self.scale * deg_per_sec
	def acc(self, deg_per_sec2):
		return self.scale * deg_per_sec2

# gear1 = GearReductionDeg(192)
# gear2 = GearReductionDeg(27)

# velY = gear2.vel(96) 
# velP = gear1.vel(96)

# print(f'yaw -> {velY}, pitch -> {velP}')
# Ak 10 - 9 > RPM=228 DPS=1368 after reduction DPS = 456
# Ak 80 - 64 > RPM=48 DPS=288 after reduction DPS = 96
