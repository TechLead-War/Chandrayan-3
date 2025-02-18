import pygame, sys, math, random
from .constants import *
from .terrain import Terrain
from .lander import Lander
from .drawables import Drawables


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH,HEIGHT))
    pygame.display.set_caption("Chandrayaan Landing")
    clock = pygame.time.Clock()
    lander = Lander()
    terrain = Terrain()
    drawable = Drawables()

    running = True
    while running:
        dt = clock.tick(20)/1000.0
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                running = False
        screen.fill(BLACK)
        lander.update(dt, terrain)
        terrain.draw(screen)
        drawable.draw_trajectory(screen, lander)
        drawable.draw_lander(screen, lander)
        drawable.draw_scale(screen, lander)
        drawable.draw_hud(screen, lander)
        pygame.display.flip()
    pygame.quit(); sys.exit()


if __name__=="__main__":
    main()
