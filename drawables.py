from .constants import *
import pygame


class Drawables:
    def draw_scale(self, screen, lander):
        bx = WIDTH-60; by = 50; bh = HEIGHT-150
        pygame.draw.rect(screen, GRAY, (bx,by,20,bh),2)
        ratio = min(lander.altitude/100000.0,1)
        fill_h = int(bh*ratio); fy = by+(bh-fill_h)
        pygame.draw.rect(screen, GREEN, (bx+2,fy,16,fill_h))
        step = 100000//10
        font = pygame.font.Font(None,24)
        for i in range(11):
            alt_mark = i*step; y_mark = by+bh-int((alt_mark/100000.0)*bh)
            lab = f"{alt_mark//1000} km"
            screen.blit(font.render(lab, True, WHITE), (bx-50,y_mark-8))


    def draw_hud(self, screen, lander):
        font = pygame.font.Font(None,28)
        lines = [f"Phase: {lander.phase}",
                 f"Alt: {lander.altitude_km():.2f} km",
                 f"VelY: {lander.vertical_velocity:.1f} m/s",
                 f"VelX: {lander.horizontal_velocity:.1f} m/s",
                 f"Tilt: {lander.angle:.1f}°",
                 f"Fuel: {lander.fuel:.1f}%",
                 f"Temp: {lander.temperature:.1f}°C",
                 f"Time2Land: {lander.time_to_land()}"]
        y=20
        for l in lines:
            screen.blit(font.render(l, True, WHITE),(20,y)); y+=30
        thr = lander.thrusters
        s = f"A:{thr['A']:.1f} B:{thr['B']:.1f} C:{thr['C']:.1f} D:{thr['D']:.1f} Main:{thr['main']:.1f}"
        screen.blit(font.render(s, True, ORANGE),(20,y))


    def draw_trajectory(self, screen, lander):
        if len(lander.path_points)>=2:
            pts=[]
            scale = HEIGHT-200
            for (px, alt) in lander.path_points:
                r = min(alt/100000.0,1)
                py = (HEIGHT-150) - r*scale
                pts.append((int(px), int(py)))
            if len(pts)>=2:
                pygame.draw.lines(screen, GREEN, False, pts, 2)
        if len(lander.projected_path)>=2:
            pts=[]
            scale = HEIGHT-200
            for (px, alt) in lander.projected_path:
                r = min(alt/100000.0,1)
                py = (HEIGHT-150)-r*scale
                pts.append((int(px), int(py)))
            for i in range(0, len(pts)-1, 8):
                pygame.draw.line(screen, BLUE, pts[i], pts[min(i+4, len(pts)-1)], 2)


    def draw_lander(self, screen, lander):
        scale = HEIGHT-200
        r = min(lander.altitude/100000.0,1)
        py = (HEIGHT-150)-r*scale
        bw, bh = 60,60
        rect = pygame.Rect(lander.x, py, bw, bh)
        center = rect.center
        surf = pygame.Surface((bw,bh), pygame.SRCALPHA)
        pygame.draw.rect(surf, GRAY, (0,0,bw,bh))
        pygame.draw.rect(surf, RED, (bw//2-5,5,10,10))
        pygame.draw.polygon(surf, WHITE, [(0,bh), (-15,bh+30), (bw+15, bh+30), (bw,bh)])
        rsurf = pygame.transform.rotate(surf, lander.angle)
        rrect = rsurf.get_rect(center=center)
        screen.blit(rsurf, rrect)
        self.draw_thrusters(screen, lander, rrect)

        if lander.flag_raised:
            pole = pygame.Rect(rrect.centerx+20, rrect.top-40, 5, 40)
            pygame.draw.rect(screen, WHITE, pole)
            fw, fh = 30,20
            flag = pygame.Rect(pole.right, pole.top, fw, fh)
            pygame.draw.rect(screen, (255,153,51), (flag.x, flag.y, fw, fh//3))
            pygame.draw.rect(screen, (255,255,255), (flag.x, flag.y+fh//3, fw, fh//3))
            pygame.draw.rect(screen, (19,136,8), (flag.x, flag.y+2*(fh//3), fw, fh//3))


    def draw_thrusters(self, screen, lander, rect):
        cx, cy = rect.center
        m = lander.thrusters["main"]
        fl = int(m*0.6)
        if fl>0:
            pygame.draw.rect(screen, ORANGE, (cx-5, rect.bottom, 10, fl))
        for key, col, offs in [("A", YELLOW, (-1,0)), ("B", YELLOW, (1,0)),
                               ("C", YELLOW, (1, -5)), ("D", YELLOW, (-1, -5))]:
            val = int(lander.thrusters[key])
            if val>0:
                if key=="A":
                    pos = rect.topleft; pos = (pos[0]-val, pos[1])
                elif key=="B":
                    pos = rect.topright; pos = (pos[0], pos[1])
                elif key=="C":
                    pos = rect.bottomright; pos = (pos[0], pos[1]-5)
                else:
                    pos = rect.bottomleft; pos = (pos[0]-val, pos[1]-5)
                pygame.draw.rect(screen, col, (pos[0], pos[1], val, 5))
