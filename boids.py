from vpython import *
import random
import math


#==============================================================================
# Globals
#==============================================================================
WIDTH = 1024
HEIGHT = 768

NBOIDS = 150
BOIDS = []
BOID_SIZE = 4
BOID_MASS = 1

SPHERES = []
SPHERE_SIZE = 20
SPHERE_MASS = 1e5


scene.title = "Modeling boids"
scene.width = WIDTH
scene.height = HEIGHT


def gravitationalForce(p1, p2):
    """ Returns distance between points (p1, p2) and gravity force """
    G = 1 # real-world value is : G = 6.67e-11
    rVector = p1 - p2
    rMagnitude = mag(rVector)
    rHat = rVector / rMagnitude
    F = - rHat * G * BOID_MASS * SPHERE_MASS /rMagnitude**2
    return rMagnitude, F


def get_alive_particles():
    """ Returns number of alive (visible) particles """
    return len([x for x in BOIDS if x.figure.visible])


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

        # Peek random color
        r = (random.randint(0, 1) + 1)/2
        g = (random.randint(0, 1) + 1)/2
        b = (random.randint(0, 1) + 1)/2

        # Make cone's length 4 times bigger than its radius
        axis = 4 * norm(self.velocity) * self.radius

        self.figure = cone(
            pos=self.position,
            axis=axis,
            radius=self.radius,
            color=vector(r, g, b),
            make_trail=False,
            trail_type="curve")

    def update_distances(self):
        """ Calculates distances between the given and all other particles """
        global BOIDS

        maxRange = max(self.cohesionRange, self.separationRange, self.alignementRange)

        # Particles within the maxRange distance
        self.neighbors = []

        for obj in BOIDS:
            if self == obj:
                continue

            if not obj.figure.visible:
                continue

            dx = math.fabs(self.position.x - obj.position.x)
            dy = math.fabs(self.position.y - obj.position.y)
            dz = math.fabs(self.position.z - obj.position.z)

            if dx > maxRange or dy > maxRange or dz > maxRange:
                continue

            d = (dx**2 + dy**2 + dz**2)**0.5
            if d > maxRange:
                continue

            self.neighbors.append((obj, d))

    def cohesion(self):
        """ Calculates cohesion adjustement (geometric center) """
        c = vector(0, 0, 0)
        count = 0

        if not self.figure.visible:
            return c

        for obj, d in self.neighbors:
            if d < self.cohesionRange:
                c += obj.position
                count += 1

        if count:
            return norm(c/count - self.position) * self.cohesionStrength

        return c

    def separation(self):
        """ Calculates separation adjustement (avoidance) """
        c = vector(0, 0, 0)
        count = 0

        if not self.figure.visible:
            return c

        for obj, d in self.neighbors:
            if d < self.separationRange:
                diff = self.position - obj.position
                k = math.pow((self.separationRange - d)/self.separationRange, 2)
                c += diff * k
                count += 1

        return norm(c) * self.separationStrength

    def alignement(self):
        """ Calculates velocity adjustement """
        c = vector(0, 0, 0)
        count = 0

        if not self.figure.visible:
            return c

        for obj, d in self.neighbors:
            if d < self.alignementRange:
                c += obj.velocity
                count += 1

        if count:
            return norm(c - self.velocity) * self.alignementStrength

        return c

    def attraction(self):
        """ Calculates gravity forces between particle and all ohter spheres """
        if not self.figure.visible:
            return

        force = vector(0, 0, 0)
        for obj in SPHERES:
            d, F = gravitationalForce(self.position, obj.pos)
            force += F
            # Check for collision
            if d <= SPHERE_SIZE:
                self.figure.visible = False
        
        self.velocity += force * dt/BOID_MASS
        self.position += self.velocity * dt + force * dt**2/BOID_MASS

    def check_borders(self, w, h):
        """ Cyclic conditions (hypertorus) """
        if self.position.x < -w//2:
            self.figure.clear_trail()
            self.position.x += w

        if self.position.x > w//2:
            self.figure.clear_trail()
            self.position.x -= w

        if self.position.y < -h//2:
            self.figure.clear_trail()
            self.position.y += h

        if self.position.y > h//2:
            self.figure.clear_trail()
            self.position.y -= h

        if self.position.z < -h//2:
            self.figure.clear_trail()
            self.position.z += h

        if self.position.z > h//2:
            self.figure.clear_trail()
            self.position.z -= h

    def move(self, w, h):
        self.update_distances()

        self.velocity += self.cohesion()
        self.velocity += self.separation()
        self.velocity += self.alignement()

        if mag(self.velocity) > Boid.maxspeed:
            self.velocity = self.velocity/mag(self.velocity) * Boid.maxspeed

        self.position += self.velocity

        self.attraction()
        self.check_borders(w, h)

        self.figure.pos = self.position
        self.figure.axis = 4 * norm(self.velocity) * self.radius
        aliveParticles.text = "{}".format(get_alive_particles())

#==============================================================================
# Interface handlers
#==============================================================================
def createSphere(ev):
    loc = ev.pos
    obj = sphere(pos=loc, radius=SPHERE_SIZE, color=color.cyan)
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
scene.bind('click', createSphere)

scene.append_to_caption('\n')
cohesionRange = slider(bind=set_cohesionRange,
    vertical=False,
    min=10, max=150, step=10,
    value=100,
    length=200, width=10)
scene.append_to_caption("Cohesion range =")
cohesionRangeReadout = wtext(text=" {}".format(Boid.cohesionRange))


scene.append_to_caption('\n')
separationRange = slider(bind=set_separationRange,
    vertical=False,
    min=10, max=150, step=10,
    value=50,
    length=200, width=10)
scene.append_to_caption("Separation range =")
separationRangeReadout = wtext(text=" {}".format(Boid.separationRange))

scene.append_to_caption('\n')
alignementRange = slider(bind=set_alignementRange,
    vertical=False,
    min=10, max=150, step=10,
    value=100,
    length=200, width=10)
scene.append_to_caption("Alignement range =")
alignementRangeReadout = wtext(text=" {}".format(Boid.alignementRange))

scene.append_to_caption('\n')
checkbox(bind=show_trail, text="Show trail")

scene.append_to_caption("\tAlive particles =")
aliveParticles = wtext(text=" {}".format(NBOIDS))

#==============================================================================
# Bounding box
#==============================================================================
mybox = box(pos=vector(0, 0, 0),
    length=WIDTH, height=HEIGHT, width=HEIGHT, opacity=0.1) 

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

    obj = Boid(vector(x, y, z), vector(vx, vy, vz), BOID_SIZE)
    BOIDS.append(obj)

#==============================================================================
# Main cycle
#==============================================================================
t = 0
dt = 0.001 #The step size. This should be a small number

while True:
    rate(1000)
    for obj in BOIDS:
        obj.move(WIDTH, HEIGHT)
    t += dt
