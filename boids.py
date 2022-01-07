import vpython as vp
import random
import math


#==============================================================================
# Globals
#==============================================================================
WIDTH = 1024
HEIGHT = 768

NBOIDS = 100
BOIDS = []
BOID_SIZE = 4
BOID_MASS = 1

SPHERES = []
SPHERE_MASS = 1e5


vp.scene.title = "Modeling boids"
vp.scene.width = WIDTH
vp.scene.height = HEIGHT


def gravitationalForce(p1, p2):
	G = 1 # real-world value is : G = 6.67e-11
	rVector = p1 - p2
	rMagnitude = vp.mag(rVector)
	rHat = rVector / rMagnitude
	F = - rHat * G * BOID_MASS * SPHERE_MASS /rMagnitude**2
	return F


class Boid:

	maxspeed = 2

	cohesionRange = 100
	cohesionStrength = 0.001

	separationRange = 50
	separationStrength = 0.01

	alignementRange = 100
	alignementStrength = 0.05

	def __init__(self, position, velocity, radius):
		self.position = position
		self.velocity = velocity
		self.radius = radius

		r = (random.randint(0, 1) + 1)/2
		g = (random.randint(0, 1) + 1)/2
		b = (random.randint(0, 1) + 1)/2

		axis = vp.norm(self.velocity) * 4 * self.radius

		self.figure = vp.cone(
			pos=self.position,
			axis=axis,
			radius=self.radius,
			color=vp.vector(r, g, b),
			make_trail=False,
			trail_type="curve")

	def update_distances(self):
		global BOIDS

		self.dist = []
		for obj in BOIDS:
			d = vp.mag(self.position - obj.position)
			self.dist.append(d)

	def cohesion(self):
		global BOIDS

		c = vp.vector(0, 0, 0)
		count = 0

		for i, d in enumerate(self.dist):
			if d > 0 and d < self.cohesionRange:
				c += BOIDS[i].position
				count += 1

		if count:
			return vp.norm(c/count - self.position) * self.cohesionStrength

		return c

	def separation(self):
		global BOIDS

		c = vp.vector(0, 0, 0)
		count = 0

		for i, d in enumerate(self.dist):
			if d > 0 and d < self.separationRange:
				diff = self.position - BOIDS[i].position
				k = math.pow((self.separationRange - d)/self.separationRange, 2)
				c += diff * k
				count += 1

		return vp.norm(c) * self.separationStrength

	def alignement(self):
		global BOIDS

		c = vp.vector(0, 0, 0)
		count = 0

		for i, d in enumerate(self.dist):
			if d > 0 and d < self.alignementRange:
				c += BOIDS[i].velocity
				count += 1

		if count:
			return vp.norm(c - self.velocity) * self.alignementStrength

		return c

	def attraction(self):
		global SPHERES

		force = vp.vector(0, 0, 0)
		for obj in SPHERES:
			force += gravitationalForce(self.position, obj.pos)
		
		self.velocity += force * dt/BOID_MASS
		self.position += self.velocity * t + force * dt**2/BOID_MASS

	def check_borders(self, w, h):
		changed = False

		if self.position.x < -w//2:
			self.position.x += w
			changed = True

		if self.position.x > w//2:
			self.position.x -= w
			changed = True

		if self.position.y < -h//2:
			self.position.y += h
			changed = True

		if self.position.y > h//2:
			self.position.y -= h
			changed = True

		if self.position.z < -h//2:
			self.position.z += h
			changed = True

		if self.position.z > h//2:
			self.position.z -= h
			changed = True

		if changed and obj.figure.make_trail:
			self.figure.clear_trail()

	def move(self, w, h):
		self.update_distances()

		self.velocity += self.cohesion()
		self.velocity += self.separation()
		self.velocity += self.alignement()
		self.position += self.velocity

		self.attraction()
		if vp.mag(self.velocity) > Boid.maxspeed:
			self.velocity = self.velocity/vp.mag(self.velocity) * Boid.maxspeed

		self.check_borders(w, h)

		self.figure.pos = self.position
		self.figure.axis = vp.norm(self.velocity) * 4 * self.radius

#==============================================================================
# Interface handlers
#==============================================================================
def createSphere(ev):
	loc = ev.pos
	obj = vp.sphere(pos=loc, radius=20, color=vp.color.cyan)
	SPHERES.append(obj)


def show_trail(ev):
	for obj in BOIDS:
		obj.figure.clear_trail()
		obj.figure.make_trail = ev.checked


def set_cohesionRange(ev):
	Boid.cohesionRange = ev.value
	cohesionRangeReadout.text = " {}".format(ev.value)


def set_separationRange(ev):
	Boid.separationRange = ev.value
	separationRangeReadout.text = " {}".format(ev.value)


def set_alignementRange(ev):
	Boid.alignementRange = ev.value
	alignementRangeReadout.text = " {}".format(ev.value)

#==============================================================================
# Interface handlers binding
#==============================================================================
vp.scene.bind('click', createSphere)

vp.scene.append_to_caption('\n')
cohesionRange = vp.slider(bind=set_cohesionRange,
	vertical=False,
	min=10, max=150, step=10,
	value=100,
	length=200, width=10)
vp.scene.append_to_caption("Cohesion range =")
cohesionRangeReadout = vp.wtext(text=" {}".format(Boid.cohesionRange))


vp.scene.append_to_caption('\n')
separationRange = vp.slider(bind=set_separationRange,
	vertical=False,
	min=10, max=150, step=10,
	value=50,
	length=200, width=10)
vp.scene.append_to_caption("Separation range =")
separationRangeReadout = vp.wtext(text=" {}".format(Boid.separationRange))

vp.scene.append_to_caption('\n')
alignementRange = vp.slider(bind=set_alignementRange,
	vertical=False,
	min=10, max=150, step=10,
	value=100,
	length=200, width=10)
vp.scene.append_to_caption("Alignement range =")
alignementRangeReadout = vp.wtext(text=" {}".format(Boid.alignementRange))

vp.scene.append_to_caption('\n')
vp.checkbox(bind=show_trail, text="Show trail")

#==============================================================================
# Bounding box
#==============================================================================
mybox = vp.box(pos=vp.vector(0, 0, 0),
	length=WIDTH, height=WIDTH, width=HEIGHT, opacity=0.1) 

#==============================================================================
# Generate boids
#==============================================================================
for i in range(NBOIDS):
	x = random.randint(-WIDTH//2, WIDTH//2)
	y = random.randint(-HEIGHT//2, HEIGHT//2)
	z = random.randint(-HEIGHT//2, HEIGHT//2)

	vx = random.randint(-2, 2)
	vy = random.randint(-2, 2)
	vz = random.randint(-2, 2)

	obj = Boid(vp.vector(x, y, z), vp.vector(vx, vy, vz), BOID_SIZE)
	BOIDS.append(obj)

#==============================================================================
# Main cycle
#==============================================================================
t = 0
dt = 0.001 #The step size. This should be a small number

while True:
	vp.rate(1000)
	for obj in BOIDS:
		obj.move(WIDTH, HEIGHT)
	t += dt
