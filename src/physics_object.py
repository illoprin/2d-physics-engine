from settings import *
from utils import *
import enum


class Collision:
    def __init__(self, overlap_depth, overlap: np.ndarray, a_type: int, b_type: int):
        self.depth = overlap_depth
        self.overlap = overlap
        self.a_type = a_type
        self.b_type = b_type

    def get_overlap(self):
        return {
            'a': -self.overlap,
            'b': self.overlap
        }

class ColliderShape(enum.IntEnum):
    NoCollider = 0
    AABB = 1
    Circle = 2

class CollisionTag(enum.IntEnum):
    Static = 0
    Dynamic = 1
    Trigger = 2



def circle_by_circle_collision(
        a_radius,
        b_radius,
        a_pos,
        b_pos):
    x1, y1 = a_pos.tolist()
    x2, y2 = b_pos.tolist()
    r1, r2 = a_radius, b_radius
    s_dist_mag = (x1 - x2)**2 + (y1 - y2)**2
    if (r1 + r2)**2 >= s_dist_mag:
        dist_mag = np.sqrt(s_dist_mag)
        dist_vec = b_pos - a_pos
        depth = r1 + r2 - dist_mag
        dir_x = dist_vec[0]/(dist_mag if dist_mag != 0 else 0.001)
        dir_y = dist_vec[1]/(dist_mag if dist_mag != 0 else 0.001)
        overlap = np.array([
            (dir_x / 2) * abs(dist_mag - r1 - r2),
            (dir_y / 2) * abs(dist_mag - r1 - r2)
        ])
        return Collision(depth, overlap, ColliderShape.Circle, ColliderShape.Circle)
    return None

def circle_by_AABB_collision(
        circle_radius,
        aabb_h_size: tuple[float, float],
        circle_pos,
        aabb_transform):
    dist = aabb_transform.position - circle_pos
    dist_mag = np.linalg.norm(dist)
    dist_norm = dist / dist_mag

    left = aabb_transform.position[0] - aabb_h_size[0]
    right = aabb_transform.position[0] + aabb_h_size[0]
    top = aabb_transform.position[1] + aabb_h_size[1]
    bottom = aabb_transform.position[1] - aabb_h_size[1]

    circle_point = circle_pos + dist_norm * circle_radius
    rectangle_nearest = np.array([clamp(circle_pos[0], left, right), clamp(circle_pos[1], bottom, top)])
    normal = circle_pos - rectangle_nearest
    normal_mag = np.linalg.norm(normal)

    if normal_mag <= circle_radius:
        overlap = normalize(normal) * (circle_radius - normal_mag)
        return Collision(normal_mag, -overlap, ColliderShape.AABB, ColliderShape.Circle)

    return None

def AABB_by_AABB_collision(
        a_size,
        b_size,
        a_transform,
        b_transform
):
    return None


class Collider:
    def __init__(self):
        self.type = 0
    def test_collision(self, a_transform, b_collider, b_transform) -> Collision: ...

class CircleCollider(Collider):
    def __init__ (self, radius=12):
        super().__init__()
        self.radius = radius
        self.type = ColliderShape.Circle

    def test_collision(self, a_transform: Transform, b_collider, b_transform: Transform):
        if b_collider.type == ColliderShape.Circle:
            return circle_by_circle_collision(self.radius, b_collider.radius, a_transform.position, b_transform.position)
        elif b_collider.type == ColliderShape.AABB:
            return circle_by_AABB_collision(self.radius, (b_collider.H_width, b_collider.H_height), a_transform.position, b_transform)

class AABB(Collider):
    def __init__(self, H_width=.0, H_height=.0):
        super().__init__()
        self.H_width = H_width
        self.H_height = H_height if H_height else H_width
        self.type = ColliderShape.AABB

    def test_collision(self, a_transform: Transform, b_collider, b_transform: Transform):
        if b_collider.type == ColliderShape.AABB:
            return AABB_by_AABB_collision((self.H_width, self.H_height), (b_collider.H_width, b_collider.H_height), a_transform, b_transform)
        elif b_collider.type == ColliderShape.Circle:
            return circle_by_AABB_collision(b_collider.radius, (self.H_width, self.H_height), a_transform.position, b_transform)




class PhysicsModel:
    def __init__(self,
                 transform: Transform,
                 velocity: list[float, float] = [0, 0],
                 mass=1.0
                 ):
        self.velocity = np.asarray(velocity, dtype='f4')
        self.force = np.zeros(2, dtype='f4')
        self.mass = mass

        self.transform = transform
        self.collider: Collider = None
        self.tag = CollisionTag.Dynamic

    def add_velocity(self, vel: np.ndarray):
        self.velocity += vel

    def add_force(self, vec: np.ndarray, mass: float):
        self.force += vec * mass

    def reset_velocity(self):
        self.velocity = np.zeros(2, dtype='f4')

    def reset_force(self):
        self.force = np.zeros(2, dtype='f4')

