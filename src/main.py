from settings import *
from physics_engine import *



class App:
    def __init__(self):
        pg.init()
        self.screen = pg.display.set_mode(RES)
        self.clock = pg.time.Clock()
        self.is_running = True

        self.delta_time = 0.01
        self.tick = 0
        self.init_world()


    def handle_events(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self.is_running = False
            if event.type == pg.KEYDOWN:
                # KEY EVENTS
                if event.key == pg.K_ESCAPE:
                    self.is_running = False

            if event.type == pg.MOUSEBUTTONDOWN:
                # MOUSE EVENTS
                # Add circle
                x, y = pg.mouse.get_pos()
                if event.button == 3 and not self.grabbed_selection:
                    self.grabbed_selection = self.pw.get_intersection(x, H - y)
                if event.button == 1:
                    if not self.grabbed_selection:
                        self.current_selection = self.pw.get_intersection(x, H - y)
                    else:
                        mouse_pos = np.array([x, H-y], dtype='f4')
                        self.grabbed_selection.velocity += (self.grabbed_selection.transform.position - mouse_pos)*UNIT
                        self.grabbed_selection = None

                print (f'\n{self.current_selection}')
                print (self.grabbed_selection)

            if event.type == pg.MOUSEBUTTONUP:
                if self.current_selection: self.current_selection = None
                print(f'\n{self.current_selection}')
                print(self.grabbed_selection)


    def init_world(self):
        self.pw = PhysicsWorld()

        ##### ADDING MULTIPLE DYNAMIC BALLS #####
        objects = 30
        for i in range(objects):
            radius = (2+np.random.random()*13)*UNIT
            x = radius+random.random()*(W-radius)
            y = radius+random.random()*(H-radius)
            self.pw.add_object(
                CircleObject(color=get_rnd_color(), radius=radius, transform=Transform([x, y]), mass=radius/UNIT))

        rect = RectangleObject(color=get_rnd_color(), size=(200, 30), transform=Transform([W/2, H/2]), mass=10)
        rect.tag = CollisionTag.Static

        self.pw.add_object(rect)

        self.current_selection = None
        self.grabbed_selection = None

    def render(self):
        for obj in self.pw.objects:
            x, y = obj.transform.position.tolist()
            vel_x, vel_y = obj.velocity.tolist()
            vel_dir = obj.velocity / np.linalg.norm(obj.velocity)
            if (x > 0 and y > 0 and x < W and y < H):
                if obj.collider.type == ColliderShape.Circle:
                    pg.draw.circle(
                        self.screen,
                        (230, 230, 230),
                        (x, H - y),
                        obj.collider.radius, 3)
                    pg.draw.line (
                        self.screen,
                        (230, 75, 50),
                        (x + vel_dir[0]*obj.collider.radius, H - y - vel_dir[1]*obj.collider.radius),
                        (x + vel_x/UNIT + vel_dir[0]*obj.collider.radius, H - y - (vel_y/UNIT + vel_dir[1]*obj.collider.radius)),
                        2
                    )
                elif obj.collider.type == ColliderShape.AABB:
                    pg.draw.rect(self.screen, (230, 230, 230),
                                 (x - obj.width/2, H - y - obj.height/2, obj.width, obj.height),
                                 2)

        if self.grabbed_selection:
            mouse_pos = pg.mouse.get_pos()
            gr_pos = self.grabbed_selection.transform.position.tolist()
            pg.draw.line(self.screen, (60, 80, 240), (gr_pos[0], H-gr_pos[1]), mouse_pos, 2)

    def update_world(self):
        self.pw.update(self.tick, self.delta_time)
        if self.current_selection:
            mouse_pos = pg.mouse.get_pos()
            self.current_selection.transform.position = np.array([mouse_pos[0], H - mouse_pos[1]], dtype='f4')

    def exit(self):
        pg.quit()
        sys.exit()

    def run(self):
        while self.is_running:
            self.delta_time = 1 / FPS
            self.tick += 1
            self.handle_events()
            self.update_world()

            self.screen.fill(CLEAR_COLOR)
            self.render()
            pg.display.flip()
            self.clock.tick(FPS)

        self.exit()


if __name__ == '__main__':
    app = App()
    app.run()
