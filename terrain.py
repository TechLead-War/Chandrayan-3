import random
import pygame
from .constants import LEFT_BOUND, RIGHT_BOUND, HEIGHT, DARK, BROWN, WIDTH


class Terrain:
    def __init__(self):
        self.craters = []
        for _ in range(6):
            cx = random.randint(LEFT_BOUND+50, RIGHT_BOUND-50)
            r = random.randint(30, 80)
            self.craters.append((cx, r))
    def draw(self, screen):
        pygame.draw.rect(screen, DARK, (0, HEIGHT-50, WIDTH, 50))
        ground_y = HEIGHT-50
        for (cx, r) in self.craters:
            pygame.draw.circle(screen, BROWN, (cx, ground_y), r)
