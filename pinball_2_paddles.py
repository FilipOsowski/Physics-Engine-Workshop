import pygame
import pymunk
import pymunk.pygame_util
import math
import random

# These two values keep track of the dampened rotary springs of both paddles
r_spring = None
l_spring = None

collision_types = {
    "ball": 0,
    "boundary": 1,
    "bumper": 2
}


def unit_vector_between(a, b):
    vector = [a.x - b.x, a.y - b.y]
    magnitude = (vector[0]**2 + vector[1]**2)**(1/2)
    return [vector[0]/magnitude, vector[1]/magnitude]


def setup_boundaries(space):
    radius = 5

    # Segments are shapes, meaning that they need a body. static_body is a body that every space contains
    # Because our segments won't be moving (i.e. no need to move their bodies) we can use space.static_body.
    segments = [
        pymunk.Segment(body=space.static_body, a=(50, 160), b=(190, 85), radius=radius),
        pymunk.Segment(space.static_body, (550, 160), (410, 85), radius),
        pymunk.Segment(space.static_body, (50, 160), (50, 800), radius),
        pymunk.Segment(space.static_body, (550, 800), (550, 160), radius),
        pymunk.Segment(space.static_body, (50, 800), (550, 800), radius),
    ]

    # Our segments need to have a nonzero elasticity to have the ball bounce off them
    for segment in segments:
        segment.elasticity = 0.5

    out_of_bounds_segment = pymunk.Segment(space.static_body, (0, -20), (700, -20), 5)
    out_of_bounds_segment.collision_type = collision_types["boundary"]

    # Collision handlers has methods that are called when a collision between the two collision types specified occurs.
    ch = space.add_collision_handler(collision_types["ball"], collision_types["boundary"])

    # This is the method that will be called when a ball collides with the boundary segment
    def remove_ball(arbiter, space, data):
        ball_shape = arbiter.shapes[0]  # Get a reference to the ball that is colliding using the arbiter's data
        space.remove(ball_shape, ball_shape.body)  # Remove that ball's shape and body
        return False  # Tells Pymunk to ignore this collision. It's already been handled by us.

    # When a collision between a ball and boundary segment occurs, handle it using our remove_ball method
    ch.begin = remove_ball

    segments.append(out_of_bounds_segment)

    space.add(segments)


def setup_paddles(space):
    global l_spring, r_spring

    paddle_mass = 10000
    stiffness = 5E10  # Stiffness of the spring
    damping = 2E9  # Dampening on the spring

    # The angle that the spring will naturally turn to
    r_rest_angle = -math.pi/6

    # Because our paddle will be rotating, it needs a moment of inertia. Pymunk helps us calculate this.
    r_paddle_vertices = ((0, 0), (80, 0), (80, 20), (0, 20))
    r_paddle_moment_of_inertia = pymunk.moment_for_poly(mass=paddle_mass, vertices=r_paddle_vertices)

    r_paddle_body = pymunk.Body(mass=paddle_mass, moment=r_paddle_moment_of_inertia)
    r_paddle_body.position = (200, 60)

    # We create a polynomial using the paddle vertices defined above.
    r_paddle_shape = pymunk.Poly(vertices=r_paddle_vertices, body=r_paddle_body)

    # Now our paddle is just a block. We need to constrain it in such a way that it can only rotate
    # about its origin. To do this, we must constrain our paddle to a static body (joint) using a pin joint.

    # We only need our joint to act as a fixed point in space. Therefore, r_paddle_joint does not need a body.
    r_paddle_joint = pymunk.Body(body_type=pymunk.Body.STATIC)
    r_paddle_joint.position = r_paddle_body.position

    # This constrains our paddle so that it can only rotate about its origin.
    j = pymunk.constraint.PinJoint(r_paddle_joint, r_paddle_body)

    # Now we need to constrain our paddle's angular position. For this, we use a dampened rotary spring.
    # This ensures that our paddle will always be pushed towards its rest angle.

    r_spring = pymunk.constraint.DampedRotarySpring(a=r_paddle_body,
                                                    b=r_paddle_joint,
                                                    rest_angle=r_rest_angle,
                                                    stiffness=stiffness,
                                                    damping=damping)

    space.add(r_paddle_body, r_paddle_shape, r_paddle_joint, j, r_spring)

    l_rest_angle = math.pi/6
    l_paddle_vertices = ((0, 0), (-80, 0), (-80, 20), (0, 20))
    l_paddle_moment_of_inertia = pymunk.moment_for_poly(mass=paddle_mass, vertices=l_paddle_vertices)

    l_paddle_body = pymunk.Body(mass=paddle_mass, moment=l_paddle_moment_of_inertia)
    l_paddle_body.position = (400, 60)

    l_paddle_shape = pymunk.Poly(vertices=l_paddle_vertices, body=l_paddle_body)

    l_paddle_joint = pymunk.Body(body_type=pymunk.Body.STATIC)
    l_paddle_joint.position = l_paddle_body.position
    j = pymunk.constraint.PinJoint(l_paddle_joint, l_paddle_body)

    l_spring = pymunk.constraint.DampedRotarySpring(a=l_paddle_body,
                                                    b=l_paddle_joint,
                                                    rest_angle=l_rest_angle,
                                                    stiffness=stiffness,
                                                    damping=damping)

    space.add(l_paddle_body, l_paddle_shape, l_paddle_joint, j, l_spring)


# We need both a body and a shape to create an object on screen
def add_ball(space):
    # All bodies hold properties of mass, moment of inertia, position, and angular position.
    # Our ball's body will have an infinite moment of inertia meaning that it won't rotate.
    ball_body = pymunk.Body(mass=1, moment=pymunk.inf)
    ball_body.position = (random.randrange(100, 500), 750)

    # Shapes keep track of things like vertices (when making a polynomial) and the radius (when creating a circle).
    # All shapes hold properties of elasticity and friction.
    ball_shape = pymunk.Circle(radius=13, body=ball_body)

    ball_shape.elasticity = 0.4  # Makes the ball bouncy.
    ball_shape.collision_type = collision_types["ball"]  # Needed to detect collisions involving the ball.

    # The shape needs to be added to make the ball appear on screen.
    # The body needs to be added to make the ball react to things like gravity and collisions.
    space.add(ball_body, ball_shape)


def setup(space):
    setup_boundaries(space)
    setup_paddles(space)


def main():
    fps = 60

    screen = pygame.display.set_mode((600, 900))

    # This makes it so that we don't have to draw things manually to Pygame's screen.
    draw_options = pymunk.pygame_util.DrawOptions(screen)

    pygame.init()

    # This is where our simulations will occur.
    space = pymunk.Space()
    space.gravity = (0, -900)

    setup(space)

    clock = pygame.time.Clock()  # Used to control frame rate of our game.

    while True:

        # This loop checks for input from the user/system.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.quit()

            if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                add_ball(space)

            # Changing the rest angle of our springs causes the corresponding paddle to move (smoothly) to that angle.
            if event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                r_spring.rest_angle = math.pi/4
            elif event.type == pygame.KEYUP and event.key == pygame.K_LEFT:
                r_spring.rest_angle = -math.pi/6

            if event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                l_spring.rest_angle = -math.pi/4
            elif event.type == pygame.KEYUP and event.key == pygame.K_RIGHT:
                l_spring.rest_angle = math.pi/6

        screen.fill((255, 255, 255))  # The screen is filled with a white background.
        space.debug_draw(draw_options)  # Pymunk tells Pygame to draw the shapes we created.
        # Neither of these two statements actually update the screen. This is done later with display.flip().

        # We take 5 small steps per frame rather than 1 big step. This makes our simulation a lot more accurate.
        step = 1/fps
        for _ in range(5):
            space.step(step / 5)

        pygame.display.flip()  # The screen is updated to display the latest fill() and debug_draw() statements.:w
        clock.tick(fps)  # Controls the frame rate of our game


if __name__ == "__main__":
    main()



