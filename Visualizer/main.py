import pandas
import pygame

from Simulation.Simulation import startSimulation, earthRadius
from Visualizer.CoordsCalculator import get_system_rect
from Visualizer.Cameras import camera_xy, camera_xz, camera_yz


if __name__ == "__main__":
    screen_width = 1280
    screen_height = 720
    real_time_per_tick = pandas.Timedelta(minutes=0, seconds=60)
    screen_ratio_step = 1.1
    frame_per_second_step = 20
    chosen_camera = camera_xy
    tracked_entity = None
    prev_frames_per_second = None

    simulation_data = startSimulation(timeUnit=real_time_per_tick)
    min_coords, max_coords = get_system_rect(simulation_data, chosen_camera)
    center = [(min_coords[i] + max_coords[i]) / 2 for i in range(len(min_coords))]
    # center_x = (max_coords[chosen_camera.first_coord()] + min_coords[chosen_camera.first_coord()]) / 2
    # center_y = (max_coords[chosen_camera.second_coord()] + min_coords[chosen_camera.second_coord()]) / 2
    max_range_x = max_coords[chosen_camera.first_coord()] - min_coords[chosen_camera.first_coord()]
    max_range_y = max_coords[chosen_camera.second_coord()] - min_coords[chosen_camera.second_coord()]
    screen_ratio_x = screen_width / max_range_x / 4
    screen_ratio_y = screen_height / max_range_y / 4
    # screen_ratio = min(screen_ratio_x, screen_ratio_y)
    screen_ratio = 0.00001
    frames_per_second = 10

    # pygame setup
    pygame.init()
    screen = pygame.display.set_mode((screen_width, screen_height))
    clock = pygame.time.Clock()
    running = True
    dt = 0
    main_font = pygame.font.SysFont(None, 48)
    secondary_font = pygame.font.SysFont(None, 36)

    frame = 0

    while running:
        # poll for events
        # pygame.QUIT event means the user clicked X to close your window
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_e:
                    tracked_entity = 'Earth'
                if event.key == pygame.K_r:
                    tracked_entity = 'Rocket'
                if event.key == pygame.K_m:
                    tracked_entity = 'MKS'
                if event.key == pygame.K_SPACE:
                    if prev_frames_per_second is None:
                        prev_frames_per_second = frames_per_second
                        frames_per_second = 0
                    else:
                        frames_per_second = prev_frames_per_second
                        prev_frames_per_second = None
                if prev_frames_per_second is None:  # means simulation paused
                    if event.key == pygame.K_RIGHT and not pygame.key.get_mods() & pygame.KMOD_LALT:
                        frames_per_second += 1
                    if event.key == pygame.K_LEFT and not pygame.key.get_mods() & pygame.KMOD_LALT:
                        frames_per_second -= 1
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_RIGHT:
                    frames_per_second = round(frames_per_second)
                if event.key == pygame.K_LEFT:
                    frames_per_second = round(frames_per_second)

        # fill the screen with a color to wipe away anything from last frame
        screen.fill("black")

        if frame >= len(simulation_data):
            frame = len(simulation_data) - 1
        elif frame < 0:
            frame = 0
        time, data = simulation_data[int(frame)]
        data.sort(key=lambda x: x[1][chosen_camera.third_coord()] * chosen_camera.orientation)
        if tracked_entity is not None:
            for entity in data:
                if entity[0] == tracked_entity:
                    center = entity[1]
                    break
        for entity in data:
            # print(entity[1][chosen_camera.first_coord()], entity[1][chosen_camera.second_coord()])
            coords_2d = (screen_width / 2 + (entity[1][chosen_camera.first_coord()] - center[chosen_camera.first_coord()]) * screen_ratio, screen_height / 2 + -(entity[1][chosen_camera.second_coord()] - center[chosen_camera.second_coord()]) * screen_ratio)
            # print(coords_2d)
            color = 'yellow'
            if entity[0] == 'Earth':
                color = 'green'
                pygame.draw.circle(screen, color, coords_2d, max(earthRadius * screen_ratio, 2))
            elif entity[0] == 'Rocket':
                color = 'blue'
                pygame.draw.circle(screen, color, coords_2d, 2)
            elif entity[0] == 'MKS':
                color = 'white'
                pygame.draw.circle(screen, color, coords_2d, 2)
        # print(screen_ratio)
        print(frames_per_second)


        width_axis = main_font.render(chosen_camera.first_axis_name(), True, 'red')
        screen.blit(width_axis, (screen_width - 40, screen_height - 40))
        height_axis = main_font.render(chosen_camera.second_axis_name(), True, 'red')
        screen.blit(height_axis, (20, 20))
        pygame.draw.line(screen, 'red', (screen_width - 20, screen_height * 8 / 13),
                         (screen_width - 20, screen_height * 11 / 13), width=2)
        ratio_text = secondary_font.render(str(round(screen_height * 3 / 13 / screen_ratio / 1000, 2)) + ' km', True,
                                           'red')
        screen.blit(ratio_text, (screen_width - 30 - ratio_text.get_width(), screen_height * 9.5 / 13))
        time_text = main_font.render(str(time), True, 'red')
        screen.blit(time_text, (20, screen_height - 60))
        time_speed_img = secondary_font.render(
            '        paused' if prev_frames_per_second else 'speed: %.2f min/sec' % (frames_per_second * real_time_per_tick.seconds / 60), True, 'red')
        screen.blit(time_speed_img, (30, screen_height - 30))


        keys = pygame.key.get_pressed()
        if keys[pygame.K_1]:
            chosen_camera = camera_xy
        if keys[pygame.K_2]:
            chosen_camera = camera_xz
        if keys[pygame.K_3]:
            chosen_camera = camera_yz
        if keys[pygame.K_UP]:
            screen_ratio *= screen_ratio_step
        if keys[pygame.K_DOWN]:
            screen_ratio /= screen_ratio_step
        if prev_frames_per_second is None:
            if keys[pygame.K_RIGHT] and pygame.key.get_mods() & pygame.KMOD_LALT:
                frames_per_second += frame_per_second_step * dt
            if keys[pygame.K_LEFT] and pygame.key.get_mods() & pygame.KMOD_LALT:
                frames_per_second -= frame_per_second_step * dt
        if keys[pygame.K_w]:
            center[chosen_camera.second_coord()] += 300 * dt / screen_ratio
            tracked_entity = None
        if keys[pygame.K_s]:
            center[chosen_camera.second_coord()] -= 300 * dt / screen_ratio
            tracked_entity = None
        if keys[pygame.K_a]:
            center[chosen_camera.first_coord()] -= 300 * dt / screen_ratio
            tracked_entity = None
        if keys[pygame.K_d]:
            center[chosen_camera.first_coord()] += 300 * dt / screen_ratio
            tracked_entity = None

        # flip() the display to put your work on screen
        pygame.display.flip()

        # limits FPS to 60
        # dt is delta time in seconds since last frame, used for framerate-
        # independent physics.
        dt = clock.tick(60) / 1000
        frame += frames_per_second * dt

    pygame.quit()