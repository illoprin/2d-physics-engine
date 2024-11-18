from settings import *
from physics_object import *


class PhysicsWorld:

    GRAVITY = np.asarray([0, -9.81])*UNIT
    y_limit = -100
    windage = .95 # air resistance

    def __init__(self):
        # ALL WORLD OBJECTS
        self.objects = []
        # LAST COLLISIONS
        self.collision_pairs: list[tuple[PhysicsModel, PhysicsModel], ...] = []

    def add_object(self, object: PhysicsModel):
        self.objects.append(object)

    def remove_object(self, object: PhysicsModel):
        self.objects.remove(object)

    def get_intersection(self, x, y):
        for obj in self.objects:
            if obj.collider.type == ColliderShape.Circle:
                distance = (x - obj.transform.position[0])**2 + (y - obj.transform.position[1])**2
                if distance < obj.collider.radius**2: return obj
            else:
                h_w, h_h = obj.collider.H_width, obj.collider.H_height
                o_x, o_y = obj.transform.position.tolist()
                if o_x - h_w < x < o_x + h_w and o_y - h_h < y < o_y + h_h: return obj

    def update(self, tick: int, dt: float):
        # self.apply_world_forces(dt)
        for obj in self.objects:
            self.apply_world_forces(obj, dt)
            self.resolve_collisions(obj, dt)
            self.collision_response(dt)
            self.apply_velocity(obj, dt)
            self.resolve_bounds_collisions(obj)


        # if not tick % FPS:
        #     self.clear_extra()

    def apply_world_forces(self, obj: PhysicsModel, dt: float):
        if obj.tag != CollisionTag.Static or obj.tag != CollisionTag.Trigger:
            # obj.force += obj.mass * PhysicsWorld.GRAVITY
            obj.velocity *= PhysicsWorld.windage
            obj.velocity += obj.force / obj.mass * dt

        obj.reset_force()
        if np.linalg.norm(obj.velocity) <= 0.03 and np.linalg.norm(obj.velocity) > 0:
            obj.reset_velocity()
            print ("Velocity reset")

    def apply_velocity(self, obj: PhysicsModel, dt: float):
        obj.transform.position += obj.velocity * dt

    def resolve_collisions(self, active: PhysicsModel, dt: float):
        if active.collider == None or active.collider.type == ColliderShape.NoCollider or active.tag == CollisionTag.Static: return
        for target in self.objects:
            if active == target: continue
            if target.collider == None or target.collider.type == ColliderShape.NoCollider: continue

            collision: Collision = active.collider.test_collision(
                active.transform,
                target.collider,
                target.transform
            )
            if collision:
                self.collision_pairs.append((active, target))
                res = collision.get_overlap()  # getting overlap vector
                if target.tag == CollisionTag.Static:
                    res['a'] *= 2
                    res['b'] = np.zeros(2, dtype='f4')

                active.transform.position += res['a']
                target.transform.position += res['b']


                
    def collision_response(self, dt: float):
        # apply physics behaviour after colliding
        for active, target in self.collision_pairs:

            active_response = not (active.tag == CollisionTag.Static or active.tag == CollisionTag.Trigger)
            target_response = not (target.tag == CollisionTag.Static or target.tag == CollisionTag.Trigger)

            if active_response or target_response:

                dist_mag = np.linalg.norm(active.transform.position - target.transform.position)

                # normal vector
                normal_vec = (target.transform.position - active.transform.position) / dist_mag

                # tangential vector (perpendicular to normal)
                tangential_vec = np.array([-normal_vec[1], normal_vec[0]])

                # tangent dot product
                active_tangent_dot = active.velocity @ tangential_vec
                active_normal_dot = active.velocity @ normal_vec

                # tangent dot product
                target_tangent_dot = target.velocity @ tangential_vec
                target_normal_dot = target.velocity @ normal_vec

                if active_response:
                    # calculate momentum (импульс)
                    momentum_active = (active_normal_dot * (active.mass - target.mass) + 2 * target.mass * target_normal_dot) / (active.mass + target.mass)
                    # apply calculated velocity
                    active.velocity = tangential_vec * active_tangent_dot + normal_vec * momentum_active

                if target_response:
                    # calculate momentum (импульс)
                    momentum_target = (target_normal_dot * (target.mass - active.mass) + 2 * active.mass * active_normal_dot) / (active.mass + target.mass)
                    # apply calculated velocity
                    target.velocity = tangential_vec * target_tangent_dot + normal_vec * momentum_target
        self.collision_pairs.clear()

    def resolve_bounds_collisions(self, obj: PhysicsModel):
        if obj.transform.position[1] < 0:
            obj.transform.position[1] = H - 1
        elif obj.transform.position[1] > H:
            obj.transform.position[1] = 1

        if obj.transform.position[0] < 0:
            obj.transform.position[0] = W - 1
        elif obj.transform.position[0] > W:
            obj.transform.position[0] = 1



    def clear_extra(self):
        for obj in reversed(self.objects):
            if obj.transform.position[1] < self.y_limit:
                self.remove_object(obj)


class CircleObject(PhysicsModel):
    def __init__(self, color=(0, 0, 0), radius=UNIT, **kwargs):
        super().__init__(**kwargs)
        self.color = color
        self.collider = CircleCollider(radius)

class RectangleObject(PhysicsModel):
    def __init__(self, color=(0, 0, 0), size=(UNIT*2, UNIT), **kwargs):
        super().__init__(**kwargs)
        self.color = color
        self.width = size[0]
        self.height = size[1]
        self.collider = AABB(self.width/2, self.height/2)