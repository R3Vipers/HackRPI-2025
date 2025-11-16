import pygame
import sys
import random
import math
from collections import deque

pygame.init()
pygame.mixer.init()

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("COIN QUEST - AI Enhanced Edition")

# ============================================
# SOUND LOADING
# ============================================

sounds = {}
sound_files = {
    'coin': 'sounds/coin_collect.mp3',
    'dog': 'sounds/dog_bark.mp3',
    'key': 'sounds/key_collect.mp3',
    'purchase': 'sounds/purchase.mp3',
    'quest': 'sounds/quest_complete.mp3',
    'treasure': 'sounds/treasure_found.mp3'
}

# Load sound effects
for sound_name, sound_path in sound_files.items():
    try:
        sounds[sound_name] = pygame.mixer.Sound(sound_path)
        print(f"Loaded sound: {sound_name}")
    except Exception as e:
        print(f"Could not load {sound_path}: {e}")
        sounds[sound_name] = None

# Load background music
try:
    pygame.mixer.music.load('Theme/Theme.mp3')
    pygame.mixer.music.set_volume(0.3)  # Background music at 30% volume
    pygame.mixer.music.play(-1)  # Loop indefinitely
    print("Background music loaded and playing")
except Exception as e:
    print(f"Could not load background music: {e}")

# Set sound effect volumes
for sound in sounds.values():
    if sound:
        sound.set_volume(0.5)

def play_sound(sound_name):
    """Play a sound effect if it exists"""
    if sound_name in sounds and sounds[sound_name]:
        sounds[sound_name].play()

COLORS = {
    'grass_light': (124, 188, 68), 'grass_dark': (88, 160, 64),
    'dirt': (184, 136, 88), 'stone': (136, 136, 136),
    'tree_trunk': (104, 72, 48), 'tree_leaves': (48, 136, 48), 'tree_dark': (32, 96, 32),
    'sky_day': (92, 148, 252), 'sky_night': (24, 24, 72), 'sky_sunset': (200, 100, 80),
    'roof_red': (216, 40, 0), 'roof_dark': (168, 32, 0),
    'wall': (248, 184, 120), 'wall_dark': (216, 152, 88),
    'door': (88, 56, 32), 'window': (88, 168, 248),
    'player_skin': (248, 200, 168), 'player_shirt': (88, 88, 248), 'player_pants': (56, 56, 136),
    'npc_shirt': (248, 88, 88), 'coin': (248, 216, 0), 'coin_dark': (200, 168, 0),
    'ui_bg': (40, 40, 88), 'ui_border': (248, 248, 248), 'text': (248, 248, 248),
    'shadow': (0, 0, 0, 100), 'dog_brown': (139, 90, 43), 'key': (255, 215, 0),
    'chest': (139, 90, 43), 'block': (120, 120, 120), 'guard_shirt': (100, 100, 255)
}

TILE_SIZE = 32

# ============================================
# AI PATHFINDING MODULE
# ============================================

def world_to_grid(x, y):
    """Convert world coordinates to grid coordinates"""
    return int(x // TILE_SIZE), int(y // TILE_SIZE)

def grid_to_world(gx, gy):
    """Convert grid coordinates to world coordinates"""
    return gx * TILE_SIZE, gy * TILE_SIZE

def create_collision_grid(obstacles, width, height):
    """Create a grid representing walkable/unwalkable tiles"""
    grid_w = width // TILE_SIZE
    grid_h = height // TILE_SIZE
    grid = [[0 for _ in range(grid_h)] for _ in range(grid_w)]
    
    for obj in obstacles:
        if obj.solid:
            gx, gy = world_to_grid(obj.x, obj.y)
            # Mark surrounding tiles as blocked
            for dx in range(-1, 3):
                for dy in range(-1, 3):
                    ngx, ngy = gx + dx, gy + dy
                    if 0 <= ngx < grid_w and 0 <= ngy < grid_h:
                        grid[ngx][ngy] = 1
    return grid

def astar_path(grid, start, goal):
    """A* pathfinding algorithm"""
    if not grid:
        return []
    
    grid_w = len(grid)
    grid_h = len(grid[0])
    
    start_gx, start_gy = world_to_grid(start[0], start[1])
    goal_gx, goal_gy = world_to_grid(goal[0], goal[1])
    
    # Bounds checking
    if not (0 <= start_gx < grid_w and 0 <= start_gy < grid_h):
        return []
    if not (0 <= goal_gx < grid_w and 0 <= goal_gy < grid_h):
        return []
    
    # If goal is blocked, find nearest walkable tile
    if grid[goal_gx][goal_gy] == 1:
        for r in range(1, 5):
            found = False
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    ngx, ngy = goal_gx + dx, goal_gy + dy
                    if 0 <= ngx < grid_w and 0 <= ngy < grid_h and grid[ngx][ngy] == 0:
                        goal_gx, goal_gy = ngx, ngy
                        found = True
                        break
                if found:
                    break
            if found:
                break
    
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    start_node = (start_gx, start_gy)
    goal_node = (goal_gx, goal_gy)
    
    frontier = [(0, start_node)]
    came_from = {start_node: None}
    cost_so_far = {start_node: 0}
    
    while frontier:
        frontier.sort(key=lambda x: x[0])
        current_cost, current = frontier.pop(0)
        
        if current == goal_node:
            # Reconstruct path
            path = []
            while current:
                wx, wy = grid_to_world(current[0], current[1])
                path.append((wx + TILE_SIZE//2, wy + TILE_SIZE//2))
                current = came_from[current]
            path.reverse()
            return path[1:]  # Skip starting position
        
        # Check neighbors (4-directional)
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nx, ny = current[0] + dx, current[1] + dy
            
            if 0 <= nx < grid_w and 0 <= ny < grid_h and grid[nx][ny] == 0:
                neighbor = (nx, ny)
                new_cost = cost_so_far[current] + 1
                
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal_node)
                    frontier.append((priority, neighbor))
                    came_from[neighbor] = current
    
    return []  # No path found

# ============================================
# SPRITE CREATION
# ============================================

def create_pixel_grass():
    surf = pygame.Surface((TILE_SIZE, TILE_SIZE))
    surf.fill(COLORS['grass_light'])
    for y in range(TILE_SIZE):
        for x in range(TILE_SIZE):
            if (x + y) % 4 == 0:
                surf.set_at((x, y), COLORS['grass_dark'])
    for _ in range(8):
        x, y = random.randint(0, TILE_SIZE-1), random.randint(0, TILE_SIZE-1)
        surf.set_at((x, y), COLORS['grass_dark'])
    return surf

def create_pixel_tree():
    surf = pygame.Surface((TILE_SIZE, TILE_SIZE), pygame.SRCALPHA)
    shadow = pygame.Surface((20, 6), pygame.SRCALPHA)
    shadow.fill(COLORS['shadow'])
    surf.blit(shadow, (6, 26))
    pygame.draw.rect(surf, COLORS['tree_trunk'], (13, 18, 6, 10))
    pygame.draw.line(surf, (80, 56, 32), (13, 18), (13, 27), 1)
    pygame.draw.circle(surf, COLORS['tree_dark'], (16, 12), 11)
    pygame.draw.circle(surf, COLORS['tree_leaves'], (16, 10), 10)
    pygame.draw.circle(surf, (64, 168, 64), (14, 8), 5)
    for dx, dy in [(-3, 10), (4, 9), (0, 12), (-2, 7), (3, 6)]:
        surf.set_at((16 + dx, 10 + dy), COLORS['tree_dark'])
    return surf

def create_pixel_house():
    surf = pygame.Surface((TILE_SIZE*2, TILE_SIZE*2), pygame.SRCALPHA)
    shadow = pygame.Surface((60, 8), pygame.SRCALPHA)
    shadow.fill(COLORS['shadow'])
    surf.blit(shadow, (2, 56))
    pygame.draw.rect(surf, COLORS['wall'], (6, 22, 52, 38))
    pygame.draw.rect(surf, COLORS['wall_dark'], (6, 40, 52, 20))
    pygame.draw.line(surf, (255, 208, 152), (6, 22), (57, 22), 2)
    roof_points = [(32, 6), (4, 22), (60, 22)]
    pygame.draw.polygon(surf, COLORS['roof_red'], roof_points)
    roof_dark_points = [(32, 6), (4, 22), (32, 22)]
    pygame.draw.polygon(surf, COLORS['roof_dark'], roof_dark_points)
    pygame.draw.lines(surf, (152, 24, 0), False, [(4, 22), (32, 6), (60, 22)], 2)
    pygame.draw.rect(surf, COLORS['door'], (26, 42, 12, 18))
    pygame.draw.rect(surf, (64, 40, 16), (26, 42, 12, 18), 1)
    surf.set_at((35, 51), COLORS['coin'])
    for wx, wy in [(14, 32), (40, 32)]:
        pygame.draw.rect(surf, (72, 48, 24), (wx-1, wy-1, 10, 10))
        pygame.draw.rect(surf, COLORS['window'], (wx, wy, 8, 8))
        pygame.draw.line(surf, (64, 128, 200), (wx+4, wy), (wx+4, wy+8), 1)
        pygame.draw.line(surf, (64, 128, 200), (wx, wy+4), (wx+8, wy+4), 1)
        pygame.draw.line(surf, (152, 216, 255), (wx+1, wy+1), (wx+3, wy+1), 1)
    return surf

def create_pixel_player():
    surf = pygame.Surface((TILE_SIZE, TILE_SIZE), pygame.SRCALPHA)
    shadow = pygame.Surface((16, 4), pygame.SRCALPHA)
    shadow.fill(COLORS['shadow'])
    surf.blit(shadow, (8, 28))
    pygame.draw.rect(surf, COLORS['player_pants'], (10, 20, 5, 8))
    pygame.draw.rect(surf, COLORS['player_pants'], (17, 20, 5, 8))
    pygame.draw.rect(surf, COLORS['player_shirt'], (10, 14, 12, 8))
    pygame.draw.line(surf, (56, 56, 200), (10, 14), (21, 14), 1)
    pygame.draw.rect(surf, COLORS['player_skin'], (8, 16, 3, 6))
    pygame.draw.rect(surf, COLORS['player_skin'], (21, 16, 3, 6))
    pygame.draw.rect(surf, COLORS['player_skin'], (12, 8, 8, 8))
    pygame.draw.line(surf, (255, 224, 192), (12, 8), (19, 8), 1)
    pygame.draw.rect(surf, (88, 56, 24), (12, 7, 8, 3))
    surf.set_at((14, 11), (0, 0, 0))
    surf.set_at((17, 11), (0, 0, 0))
    return surf

def create_pixel_npc():
    surf = pygame.Surface((TILE_SIZE, TILE_SIZE), pygame.SRCALPHA)
    shadow = pygame.Surface((16, 4), pygame.SRCALPHA)
    shadow.fill(COLORS['shadow'])
    surf.blit(shadow, (8, 28))
    pygame.draw.rect(surf, (72, 72, 72), (10, 20, 5, 8))
    pygame.draw.rect(surf, (72, 72, 72), (17, 20, 5, 8))
    pygame.draw.rect(surf, COLORS['npc_shirt'], (10, 14, 12, 8))
    pygame.draw.line(surf, (255, 128, 128), (10, 14), (21, 14), 1)
    pygame.draw.rect(surf, COLORS['player_skin'], (8, 16, 3, 6))
    pygame.draw.rect(surf, COLORS['player_skin'], (21, 16, 3, 6))
    pygame.draw.rect(surf, COLORS['player_skin'], (12, 8, 8, 8))
    pygame.draw.line(surf, (255, 224, 192), (12, 8), (19, 8), 1)
    pygame.draw.rect(surf, (136, 88, 40), (11, 7, 10, 3))
    surf.set_at((14, 11), (0, 0, 0))
    surf.set_at((17, 11), (0, 0, 0))
    pygame.draw.line(surf, (200, 120, 120), (14, 13), (17, 13), 1)
    return surf

def create_guard_sprite():
    """Create a guard NPC sprite (different color)"""
    surf = pygame.Surface((TILE_SIZE, TILE_SIZE), pygame.SRCALPHA)
    shadow = pygame.Surface((16, 4), pygame.SRCALPHA)
    shadow.fill(COLORS['shadow'])
    surf.blit(shadow, (8, 28))
    pygame.draw.rect(surf, (40, 40, 40), (10, 20, 5, 8))
    pygame.draw.rect(surf, (40, 40, 40), (17, 20, 5, 8))
    pygame.draw.rect(surf, COLORS['guard_shirt'], (10, 14, 12, 8))
    pygame.draw.line(surf, (150, 150, 255), (10, 14), (21, 14), 1)
    pygame.draw.rect(surf, COLORS['player_skin'], (8, 16, 3, 6))
    pygame.draw.rect(surf, COLORS['player_skin'], (21, 16, 3, 6))
    pygame.draw.rect(surf, COLORS['player_skin'], (12, 8, 8, 8))
    pygame.draw.line(surf, (255, 224, 192), (12, 8), (19, 8), 1)
    pygame.draw.rect(surf, (60, 60, 60), (11, 7, 10, 3))
    surf.set_at((14, 11), (0, 0, 0))
    surf.set_at((17, 11), (0, 0, 0))
    return surf

def create_pixel_coin():
    surf = pygame.Surface((16, 16), pygame.SRCALPHA)
    shadow = pygame.Surface((12, 3), pygame.SRCALPHA)
    shadow.fill(COLORS['shadow'])
    surf.blit(shadow, (2, 13))
    pygame.draw.circle(surf, COLORS['coin_dark'], (8, 7), 6)
    pygame.draw.circle(surf, COLORS['coin'], (8, 7), 5)
    pygame.draw.arc(surf, (255, 248, 128), (4, 3, 8, 8), 0.5, 2.5, 2)
    pygame.draw.line(surf, COLORS['coin_dark'], (8, 4), (8, 10), 1)
    pygame.draw.line(surf, COLORS['coin_dark'], (6, 5), (10, 5), 1)
    pygame.draw.line(surf, COLORS['coin_dark'], (6, 9), (10, 9), 1)
    return surf

def create_dog():
    surf = pygame.Surface((TILE_SIZE, TILE_SIZE), pygame.SRCALPHA)
    pygame.draw.ellipse(surf, COLORS['dog_brown'], (8, 16, 16, 12))
    pygame.draw.circle(surf, COLORS['dog_brown'], (12, 12), 5)
    pygame.draw.rect(surf, COLORS['dog_brown'], (6, 22, 3, 6))
    pygame.draw.rect(surf, COLORS['dog_brown'], (15, 22, 3, 6))
    surf.set_at((10, 11), (0, 0, 0))
    surf.set_at((14, 11), (0, 0, 0))
    pygame.draw.circle(surf, (80, 50, 30), (12, 13), 1)
    return surf

def create_key():
    surf = pygame.Surface((16, 16), pygame.SRCALPHA)
    pygame.draw.circle(surf, COLORS['key'], (6, 6), 4)
    pygame.draw.circle(surf, (0, 0, 0), (6, 6), 2)
    pygame.draw.rect(surf, COLORS['key'], (6, 6, 6, 2))
    return surf

def create_chest():
    surf = pygame.Surface((TILE_SIZE, TILE_SIZE), pygame.SRCALPHA)
    pygame.draw.rect(surf, COLORS['chest'], (6, 16, 20, 12))
    pygame.draw.rect(surf, (100, 65, 30), (6, 10, 20, 8))
    pygame.draw.rect(surf, COLORS['key'], (14, 18, 4, 4))
    return surf

def create_pushable_block():
    surf = pygame.Surface((TILE_SIZE, TILE_SIZE), pygame.SRCALPHA)
    pygame.draw.rect(surf, COLORS['block'], (2, 2, 28, 28))
    pygame.draw.rect(surf, (90, 90, 90), (2, 2, 28, 28), 2)
    for i in range(4, 28, 6):
        pygame.draw.line(surf, (150, 150, 150), (i, 2), (i, 30), 1)
        pygame.draw.line(surf, (150, 150, 150), (2, i), (30, i), 1)
    return surf

def create_shop_sign():
    surf = pygame.Surface((TILE_SIZE, TILE_SIZE), pygame.SRCALPHA)
    pygame.draw.rect(surf, COLORS['tree_trunk'], (14, 12, 4, 20))
    pygame.draw.rect(surf, (200, 180, 140), (4, 8, 24, 12))
    pygame.draw.rect(surf, (120, 100, 60), (4, 8, 24, 12), 2)
    return surf

# Create all sprites
grass_tile = create_pixel_grass()
tree_sprite = create_pixel_tree()
house_sprite = create_pixel_house()
player_sprite = create_pixel_player()
coin_sprite = create_pixel_coin()
npc_sprite = create_pixel_npc()
guard_sprite = create_guard_sprite()
dog_sprite = create_dog()
key_sprite = create_key()
chest_sprite = create_chest()
block_sprite = create_pushable_block()
shop_sign_sprite = create_shop_sign()

# ============================================
# GAME OBJECTS
# ============================================

class GameObject:
    def __init__(self, x, y, sprite, solid=False):
        self.x, self.y, self.sprite, self.solid = x, y, sprite, solid
        self.rect = pygame.Rect(x, y, TILE_SIZE, TILE_SIZE)

class House(GameObject):
    def __init__(self, x, y):
        super().__init__(x, y, house_sprite, True)
        self.rect = pygame.Rect(x, y, TILE_SIZE*2, TILE_SIZE*2)

class PushableBlock(GameObject):
    def __init__(self, x, y):
        super().__init__(x, y, block_sprite, True)
        self.pushable = True
    
    def push(self, dx, dy, obstacles):
        new_x, new_y = self.x + dx, self.y + dy
        new_rect = pygame.Rect(new_x, new_y, TILE_SIZE, TILE_SIZE)
        for obj in obstacles:
            if obj != self and obj.solid and new_rect.colliderect(obj.rect):
                return False
        self.x, self.y = new_x, new_y
        self.rect.topleft = (new_x, new_y)
        return True

# ============================================
# AI NPC CLASS
# ============================================

class NPC:
    def __init__(self, x, y, name, sprite=npc_sprite, ai_type='static'):
        self.x, self.y, self.sprite, self.name = x, y, sprite, name
        self.rect = pygame.Rect(x, y, TILE_SIZE, TILE_SIZE)
        self.dialogue = []
        self.dialogue_index = 0
        self.visible = True
        self.quest_giver = None
        
        # AI properties
        self.ai_type = ai_type  # 'static', 'chase', 'patrol', 'wander'
        self.speed = 0.8
        self.path = []
        self.path_index = 0
        self.patrol_points = []
        self.current_patrol_target = 0
        self.wander_timer = 0
        self.wander_target = (x, y)
        self.detection_range = 150
        self.ai_update_cooldown = 0
        
    def update_ai(self, player_pos, obstacles, collision_grid):
        """Update NPC AI behavior"""
        self.ai_update_cooldown -= 1
        
        if self.ai_type == 'static':
            return
        
        elif self.ai_type == 'chase':
            # Chase player if within detection range
            dist = math.sqrt((self.x - player_pos[0])**2 + (self.y - player_pos[1])**2)
            if dist < self.detection_range:
                # Recalculate path every 30 frames
                if self.ai_update_cooldown <= 0:
                    self.path = astar_path(collision_grid, (self.x, self.y), player_pos)
                    self.path_index = 0
                    self.ai_update_cooldown = 30
                
                # Follow path
                if self.path and self.path_index < len(self.path):
                    target = self.path[self.path_index]
                    dx = target[0] - self.x
                    dy = target[1] - self.y
                    dist_to_target = math.sqrt(dx**2 + dy**2)
                    
                    if dist_to_target < 5:
                        self.path_index += 1
                    else:
                        dx, dy = dx / dist_to_target, dy / dist_to_target
                        self.x += dx * self.speed
                        self.y += dy * self.speed
        
        elif self.ai_type == 'patrol':
            # Patrol between waypoints
            if not self.patrol_points:
                return
            
            target = self.patrol_points[self.current_patrol_target]
            dx = target[0] - self.x
            dy = target[1] - self.y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < 10:
                self.current_patrol_target = (self.current_patrol_target + 1) % len(self.patrol_points)
            else:
                # Use pathfinding for patrol
                if self.ai_update_cooldown <= 0:
                    self.path = astar_path(collision_grid, (self.x, self.y), target)
                    self.path_index = 0
                    self.ai_update_cooldown = 45
                
                if self.path and self.path_index < len(self.path):
                    path_target = self.path[self.path_index]
                    dx = path_target[0] - self.x
                    dy = path_target[1] - self.y
                    dist_to_target = math.sqrt(dx**2 + dy**2)
                    
                    if dist_to_target < 5:
                        self.path_index += 1
                    else:
                        dx, dy = dx / dist_to_target, dy / dist_to_target
                        self.x += dx * self.speed
                        self.y += dy * self.speed
        
        elif self.ai_type == 'wander':
            # Random wandering
            self.wander_timer -= 1
            
            if self.wander_timer <= 0:
                # Pick new random target
                self.wander_target = (
                    random.randint(50, SCREEN_WIDTH - 50),
                    random.randint(50, SCREEN_HEIGHT - 50)
                )
                self.wander_timer = random.randint(120, 300)
            
            dx = self.wander_target[0] - self.x
            dy = self.wander_target[1] - self.y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist > 10:
                dx, dy = dx / dist, dy / dist
                self.x += dx * self.speed * 0.5
                self.y += dy * self.speed * 0.5
        
        # Update rect
        self.rect.topleft = (self.x, self.y)

# ============================================
# GAME STATE
# ============================================

player_x, player_y = 400, 300
base_speed = 2
player_speed = base_speed
coins_collected = 0
keys_collected = 0
game_time = 0
inventory = []
cheat_buffer = ""

quests = {
    'tom_coins': {'active': False, 'complete': False, 'progress': 0, 'required': 5},
    'deliver_letter': {'active': False, 'complete': False, 'has_letter': False},
    'find_dog': {'active': False, 'complete': False}
}

shop_items = {
    'speed_boots': {'name': 'Speed Boots', 'price': 10, 'owned': False, 'desc': 'Walk faster!'},
    'magnet': {'name': 'Coin Magnet', 'price': 15, 'owned': False, 'desc': 'Auto-collect coins'},
    'detector': {'name': 'Coin Detector', 'price': 12, 'owned': False, 'desc': 'Shows nearest coin'},
}

showing_dialogue = False
current_dialogue = ""
current_npc_name = ""
showing_shop = False
selected_shop_item = 0
game_won = False
game_over = False

# ============================================
# WORLD SETUP
# ============================================

trees = [
    GameObject(120, 80, tree_sprite, True), GameObject(160, 80, tree_sprite, True),
    GameObject(280, 120, tree_sprite, True), GameObject(520, 180, tree_sprite, True),
    GameObject(180, 420, tree_sprite, True), GameObject(620, 320, tree_sprite, True),
    GameObject(140, 520, tree_sprite, True), GameObject(720, 460, tree_sprite, True),
    GameObject(60, 240, tree_sprite, True), GameObject(740, 120, tree_sprite, True),
    GameObject(380, 480, tree_sprite, True), GameObject(560, 80, tree_sprite, True),
]

houses = [House(220, 160), House(520, 360)]
shop_sign = GameObject(200, 130, shop_sign_sprite, False)
pushable_blocks = [PushableBlock(400, 200), PushableBlock(432, 200)]

secret_tree = GameObject(740, 120, tree_sprite, True)
secret_tree.has_treasure = True

dog = GameObject(180, 420, dog_sprite, False)
dog.found = False

chest = GameObject(650, 450, chest_sprite, True)
chest.opened = False

key_objects = [
    GameObject(100, 500, key_sprite, False),
    GameObject(300, 100, key_sprite, False),
    GameObject(700, 200, key_sprite, False)
]

# Create NPCs with AI
npcs = [
    NPC(360, 260, "OLD MAN TOM", npc_sprite, 'static'),
    NPC(460, 420, "LITTLE SUSIE", npc_sprite, 'wander'),
    NPC(240, 140, "SHOPKEEPER JOE", npc_sprite, 'static'),
    NPC(600, 100, "GUARD PATROL", guard_sprite, 'patrol'),
    NPC(100, 300, "FRIENDLY GUARD", guard_sprite, 'chase'),
]

# Set NPC dialogues
npcs[0].dialogue = [
    "Hello traveler! I lost some coins...",
    "Can you find 5 coins for me? I'll give you a treasure map!"
]
npcs[0].quest_giver = 'tom_coins'

npcs[1].dialogue = [
    "Hi! My dog ran away! Can you find him?",
    "He likes to hide behind trees!"
]
npcs[1].quest_giver = 'find_dog'

npcs[2].dialogue = [
    "Welcome to my shop! Press S to browse!",
    "I have useful items for sale!"
]

npcs[3].dialogue = ["I'm on patrol duty!", "Stay out of trouble!"]
npcs[3].patrol_points = [(600, 100), (700, 100), (700, 250), (600, 250)]
npcs[3].speed = 1.5  # Faster patrol speed

npcs[4].dialogue = ["I'll catch you!", "You can't escape!"]
npcs[4].speed = 2.0  # Faster chase speed
npcs[4].is_enemy = True  # Mark as enemy

class Coin:
    def __init__(self, x, y):
        self.x, self.y, self.sprite = x, y, coin_sprite
        self.collected = False
        self.rect = pygame.Rect(x, y, 16, 16)
        self.bob_offset = random.uniform(0, 6.28)
        self.quest_coin = False

coins = []
for _ in range(25):
    x, y = random.randint(50, SCREEN_WIDTH - 50), random.randint(50, SCREEN_HEIGHT - 50)
    coins.append(Coin(x, y))

midnight_coins = []

font = pygame.font.Font(None, 20)
big_font = pygame.font.Font(None, 48)
title_font = pygame.font.Font(None, 32)
clock = pygame.time.Clock()
frame_count = 0

# Load sis_man image for game over screen
has_sis_man = False
sis_man_image = None

# Try multiple possible paths
possible_paths = [
    "images/sis_man.png",
    "sis_man.png",
    "images\\sis_man.png",
    "./images/sis_man.png"
]

for path in possible_paths:
    try:
        sis_man_image = pygame.image.load(path)
        # Scale it to a reasonable size
        sis_man_image = pygame.transform.scale(sis_man_image, (150, 150))
        has_sis_man = True
        print(f"Successfully loaded sis_man from: {path}")
        break
    except Exception as e:
        continue

if not has_sis_man:
    print("Warning: Could not find sis_man.png in any of these locations:")
    for path in possible_paths:
        print(f"  - {path}")
    print("Game will run without the image.")

# Create collision grid for pathfinding
all_obstacles = trees + houses + pushable_blocks
collision_grid = create_collision_grid(all_obstacles, SCREEN_WIDTH, SCREEN_HEIGHT)

# ============================================
# HELPER FUNCTIONS
# ============================================

def get_time_of_day():
    if game_time < 360: return 'night'
    elif game_time < 420: return 'sunrise'
    elif game_time < 1080: return 'day'
    elif game_time < 1140: return 'sunset'
    else: return 'night'

def get_sky_color():
    tod = get_time_of_day()
    if tod == 'day': return COLORS['sky_day']
    elif tod == 'night': return COLORS['sky_night']
    elif tod == 'sunrise' or tod == 'sunset': return COLORS['sky_sunset']

def check_collision(x, y, objects):
    player_rect = pygame.Rect(x, y, TILE_SIZE, TILE_SIZE)
    for obj in objects:
        if obj.solid and player_rect.colliderect(obj.rect):
            return True
    return False

def draw_retro_text(text, x, y, color=COLORS['text'], shadow=True):
    if shadow:
        shadow_surf = font.render(text, False, (0, 0, 0))
        screen.blit(shadow_surf, (x + 2, y + 2))
    text_surf = font.render(text, False, color)
    screen.blit(text_surf, (x, y))

def draw_dialogue_box(name, text):
    box_height = 120
    box_rect = pygame.Rect(40, SCREEN_HEIGHT - box_height - 30, SCREEN_WIDTH - 80, box_height)
    pygame.draw.rect(screen, (0, 0, 0), box_rect.inflate(4, 4))
    pygame.draw.rect(screen, COLORS['ui_bg'], box_rect)
    pygame.draw.rect(screen, COLORS['ui_border'], box_rect, 2)
    name_rect = pygame.Rect(box_rect.x + 10, box_rect.y - 15, len(name) * 10, 20)
    pygame.draw.rect(screen, COLORS['ui_bg'], name_rect)
    pygame.draw.rect(screen, COLORS['ui_border'], name_rect, 2)
    name_text = font.render(name, False, COLORS['coin'])
    screen.blit(name_text, (name_rect.x + 5, name_rect.y + 3))
    
    words = text.split(' ')
    lines, current_line = [], ""
    for word in words:
        test_line = current_line + word + " "
        if font.size(test_line)[0] < box_rect.width - 30:
            current_line = test_line
        else:
            lines.append(current_line)
            current_line = word + " "
    lines.append(current_line)
    
    for i, line in enumerate(lines[:3]):
        dialogue_text = font.render(line, False, COLORS['text'])
        screen.blit(dialogue_text, (box_rect.x + 15, box_rect.y + 25 + i * 22))
    
    if frame_count % 60 < 30:
        prompt = font.render("[SPACE]", False, COLORS['coin'])
        screen.blit(prompt, (box_rect.right - 80, box_rect.bottom - 25))

def draw_shop_menu():
    box_rect = pygame.Rect(150, 150, 500, 300)
    pygame.draw.rect(screen, (0, 0, 0), box_rect.inflate(6, 6))
    pygame.draw.rect(screen, COLORS['ui_bg'], box_rect)
    pygame.draw.rect(screen, COLORS['ui_border'], box_rect, 3)
    
    title = title_font.render("SHOP", False, COLORS['coin'])
    screen.blit(title, (box_rect.centerx - 40, box_rect.y + 10))
    
    y = box_rect.y + 60
    for i, (key, item) in enumerate(shop_items.items()):
        color = COLORS['coin'] if i == selected_shop_item else COLORS['text']
        status = "[OWNED]" if item['owned'] else f"${item['price']}"
        text = f"{item['name']} - {status}"
        item_text = font.render(text, False, color)
        screen.blit(item_text, (box_rect.x + 30, y))
        desc_text = font.render(item['desc'], False, (180, 180, 180))
        screen.blit(desc_text, (box_rect.x + 50, y + 22))
        y += 60
    
    coins_text = font.render(f"Your coins: ${coins_collected}", False, COLORS['coin'])
    screen.blit(coins_text, (box_rect.x + 30, box_rect.bottom - 40))
    
    controls = font.render("UP/DOWN: Select | ENTER: Buy | ESC: Close", False, COLORS['text'])
    screen.blit(controls, (box_rect.x + 30, box_rect.bottom - 20))

def find_nearest_coin():
    nearest, min_dist = None, float('inf')
    for coin in coins:
        if not coin.collected:
            dist = math.sqrt((coin.x - player_x)**2 + (coin.y - player_y)**2)
            if dist < min_dist:
                min_dist, nearest = dist, coin
    return nearest

# ============================================
# MAIN GAME LOOP
# ============================================

running = True
while running:
    frame_count += 1
    game_time = (game_time + 0.5) % 1440
    
    # Spawn midnight coins
    if get_time_of_day() == 'night' and game_time > 1200 and len(midnight_coins) == 0:
        for _ in range(5):
            midnight_coins.append(Coin(random.randint(50, 750), random.randint(50, 550)))
    elif get_time_of_day() != 'night':
        midnight_coins.clear()
    
    # Update speed
    player_speed = base_speed * (2 if shop_items['speed_boots']['owned'] else 1)
    
    # Update collision grid periodically
    if frame_count % 60 == 0:
        all_obstacles = trees + houses + pushable_blocks
        if not chest.opened:
            all_obstacles.append(chest)
        collision_grid = create_collision_grid(all_obstacles, SCREEN_WIDTH, SCREEN_HEIGHT)
    
    # Update NPC AI
    for npc in npcs:
        if npc.visible:
            npc.update_ai((player_x, player_y), all_obstacles, collision_grid)
            
            # Check if enemy NPC caught the player
            if hasattr(npc, 'is_enemy') and npc.is_enemy:
                dist = math.sqrt((npc.x - player_x)**2 + (npc.y - player_y)**2)
                if dist < 30:  # Caught!
                    game_over = True
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            # Cheat code handling
            if event.unicode.isalpha():
                cheat_buffer += event.unicode.upper()
                cheat_buffer = cheat_buffer[-5:]
                if cheat_buffer == "SPEED":
                    shop_items['speed_boots']['owned'] = True
                    showing_dialogue = True
                    current_dialogue = "CHEAT ACTIVATED: Speed Boots!"
                    current_npc_name = "SYSTEM"
            
            if event.key == pygame.K_ESCAPE:
                if showing_shop:
                    showing_shop = False
                else:
                    running = False
            if event.key == pygame.K_SPACE:
                if showing_dialogue:
                    showing_dialogue = False
                if game_won or game_over:
                    # Reset game
                    coins_collected, keys_collected = 0, 0
                    game_won = False
                    game_over = False
                    player_x, player_y = 400, 300  # Reset player position
                    for coin in coins:
                        coin.collected = False
                    for key in key_objects:
                        key.collected = False
                    chest.opened = False
                    dog.found = False
                    for quest in quests.values():
                        quest['active'], quest['complete'] = False, False
            
            if event.key == pygame.K_s and not showing_dialogue and not game_won:
                dist_x, dist_y = abs(player_x - npcs[2].x), abs(player_y - npcs[2].y)
                if dist_x < 60 and dist_y < 60:
                    showing_shop = not showing_shop
            
            if showing_shop:
                if event.key == pygame.K_UP:
                    selected_shop_item = (selected_shop_item - 1) % len(shop_items)
                elif event.key == pygame.K_DOWN:
                    selected_shop_item = (selected_shop_item + 1) % len(shop_items)
                elif event.key == pygame.K_RETURN:
                    item_key = list(shop_items.keys())[selected_shop_item]
                    item = shop_items[item_key]
                    if not item['owned'] and coins_collected >= item['price']:
                        coins_collected -= item['price']
                        item['owned'] = True
                        play_sound('purchase')  # Play purchase sound
                        showing_dialogue = True
                        current_dialogue = f"Purchased {item['name']}!"
                        current_npc_name = "SHOPKEEPER"
                        showing_shop = False
            
            if event.key == pygame.K_e and not showing_dialogue and not showing_shop:
                # Check NPC interaction
                for npc in npcs:
                    if not npc.visible:
                        continue
                    dist_x, dist_y = abs(player_x - npc.x), abs(player_y - npc.y)
                    if dist_x < 50 and dist_y < 50:
                        showing_dialogue = True
                        if npc.dialogue_index < len(npc.dialogue):
                            current_dialogue = npc.dialogue[npc.dialogue_index]
                        else:
                            current_dialogue = npc.dialogue[-1]
                        current_npc_name = npc.name
                        npc.dialogue_index += 1
                        
                        if npc.quest_giver and not quests[npc.quest_giver]['active']:
                            quests[npc.quest_giver]['active'] = True
                        break
                
                # Check dog interaction
                if not dog.found:
                    dist_x, dist_y = abs(player_x - dog.x), abs(player_y - dog.y)
                    if dist_x < 40 and dist_y < 40:
                        dog.found = True
                        quests['find_dog']['complete'] = True
                        play_sound('dog')  # Play dog bark sound
                        showing_dialogue = True
                        current_dialogue = "You found the dog! Susie will be happy!"
                        current_npc_name = "DOG"
                
                # Check chest
                if keys_collected >= 3 and not chest.opened:
                    dist_x, dist_y = abs(player_x - chest.x), abs(player_y - chest.y)
                    if dist_x < 40 and dist_y < 40:
                        chest.opened = True
                        coins_collected += 50
                        play_sound('treasure')  # Play treasure sound
                        showing_dialogue = True
                        current_dialogue = "You unlocked the chest! +50 coins!"
                        current_npc_name = "TREASURE"
    
    if not showing_dialogue and not game_won and not showing_shop and not game_over:
        keys = pygame.key.get_pressed()
        new_x, new_y = player_x, player_y
        moved = False
        
        if keys[pygame.K_LEFT]:
            new_x -= player_speed
            moved = True
        if keys[pygame.K_RIGHT]:
            new_x += player_speed
            moved = True
        if keys[pygame.K_UP]:
            new_y -= player_speed
            moved = True
        if keys[pygame.K_DOWN]:
            new_y += player_speed
            moved = True
        
        all_solid = trees + houses + pushable_blocks
        if chest.opened:
            all_solid = [obj for obj in all_solid if obj != chest]
        
        # Check if pushing a block
        if moved:
            player_rect = pygame.Rect(new_x, new_y, TILE_SIZE, TILE_SIZE)
            for block in pushable_blocks:
                if player_rect.colliderect(block.rect):
                    dx = player_speed if keys[pygame.K_RIGHT] else (-player_speed if keys[pygame.K_LEFT] else 0)
                    dy = player_speed if keys[pygame.K_DOWN] else (-player_speed if keys[pygame.K_UP] else 0)
                    if block.push(dx * 8, dy * 8, all_solid):
                        pass
                    else:
                        new_x, new_y = player_x, player_y
                    break
        
        if not check_collision(new_x, new_y, all_solid):
            player_x, player_y = new_x, new_y
        
        player_x = max(0, min(player_x, SCREEN_WIDTH - TILE_SIZE))
        player_y = max(0, min(player_y, SCREEN_HEIGHT - TILE_SIZE))
        
        # Collect coins
        player_rect = pygame.Rect(player_x, player_y, TILE_SIZE, TILE_SIZE)
        magnet_range = 80 if shop_items['magnet']['owned'] else 0
        
        for coin in coins + midnight_coins:
            if coin.collected:
                continue
            
            # Magnet effect
            if magnet_range > 0:
                dist = math.sqrt((coin.x - player_x)**2 + (coin.y - player_y)**2)
                if dist < magnet_range:
                    coin.collected = True
                    coins_collected += 1
                    play_sound('coin')  # Play coin sound
                    if quests['tom_coins']['active'] and not quests['tom_coins']['complete']:
                        quests['tom_coins']['progress'] += 1
                        if quests['tom_coins']['progress'] >= quests['tom_coins']['required']:
                            quests['tom_coins']['complete'] = True
                            play_sound('quest')  # Play quest complete sound
                            showing_dialogue = True
                            current_dialogue = "Quest complete! You found 5 coins for Tom! Check behind the big tree!"
                            current_npc_name = "QUEST"
                    continue
            
            # Regular collection
            if player_rect.colliderect(coin.rect):
                coin.collected = True
                coins_collected += 1
                play_sound('coin')  # Play coin sound
                if quests['tom_coins']['active'] and not quests['tom_coins']['complete']:
                    quests['tom_coins']['progress'] += 1
                    if quests['tom_coins']['progress'] >= quests['tom_coins']['required']:
                        quests['tom_coins']['complete'] = True
                        play_sound('quest')  # Play quest complete sound
                        showing_dialogue = True
                        current_dialogue = "Quest complete! You found 5 coins for Tom! Check behind the big tree!"
                        current_npc_name = "QUEST"
        
        # Collect keys
        for key in key_objects:
            if not hasattr(key, 'collected'):
                key.collected = False
            if not key.collected and player_rect.colliderect(key.rect):
                key.collected = True
                keys_collected += 1
                play_sound('key')  # Play key sound
                showing_dialogue = True
                current_dialogue = f"Found a key! ({keys_collected}/3)"
                current_npc_name = "KEY"
        
        # Secret treasure
        if quests['tom_coins']['complete'] and hasattr(secret_tree, 'has_treasure'):
            dist_x, dist_y = abs(player_x - secret_tree.x), abs(player_y - secret_tree.y)
            if dist_x < 40 and dist_y < 40:
                coins_collected += 50
                play_sound('treasure')  # Play treasure sound
                showing_dialogue = True
                current_dialogue = "SECRET FOUND! +50 coins hidden behind the tree!"
                current_npc_name = "SECRET"
                delattr(secret_tree, 'has_treasure')
        
        # Win condition
        if coins_collected >= 100:
            game_won = True
    
    # ============================================
    # DRAWING
    # ============================================
    
    sky_color = get_sky_color()
    screen.fill(sky_color)
    
    # Draw grass
    for x in range(0, SCREEN_WIDTH, TILE_SIZE):
        for y in range(0, SCREEN_HEIGHT, TILE_SIZE):
            screen.blit(grass_tile, (x, y))
    
    # Draw houses
    for house in houses:
        screen.blit(house.sprite, (house.x, house.y))
    
    # Draw shop sign
    screen.blit(shop_sign.sprite, (shop_sign.x, shop_sign.y))
    
    # Draw blocks
    for block in pushable_blocks:
        screen.blit(block.sprite, (block.x, block.y))
    
    # Draw chest
    if not chest.opened:
        screen.blit(chest.sprite, (chest.x, chest.y))
    
    # Draw keys
    for key in key_objects:
        if not key.collected:
            screen.blit(key.sprite, (key.x, key.y))
    
    # Draw coins
    for coin in coins + midnight_coins:
        if not coin.collected:
            bob = int(2 * abs(((frame_count + coin.bob_offset * 10) % 60) - 30) / 30)
            screen.blit(coin.sprite, (coin.x, coin.y - bob))
    
    # Draw trees
    for tree in trees:
        screen.blit(tree.sprite, (tree.x, tree.y))
    
    # Draw dog
    if not dog.found and quests['find_dog']['active']:
        screen.blit(dog.sprite, (dog.x, dog.y))
        if frame_count % 120 < 60:
            draw_retro_text("!", dog.x + 8, dog.y - 15, COLORS['coin'])
    
    # Draw NPCs
    for npc in npcs:
        if npc.visible:
            screen.blit(npc.sprite, (int(npc.x), int(npc.y)))
            dist_x, dist_y = abs(player_x - npc.x), abs(player_y - npc.y)
            if dist_x < 50 and dist_y < 50 and not showing_dialogue:
                bob = int(2 * abs((frame_count % 40) - 20) / 20)
                draw_retro_text("[E]", int(npc.x) + 4, int(npc.y) - 25 - bob, COLORS['coin'])
            
            # Draw AI debug info (optional - shows current behavior)
            if npc.ai_type != 'static':
                ai_label = font.render(npc.ai_type.upper(), False, (255, 255, 0))
                screen.blit(ai_label, (int(npc.x) - 10, int(npc.y) - 40))
    
    # Draw player
    screen.blit(player_sprite, (player_x, player_y))
    
    # Coin detector
    if shop_items['detector']['owned']:
        nearest = find_nearest_coin()
        if nearest:
            angle = math.atan2(nearest.y - player_y, nearest.x - player_x)
            arrow_x = player_x + 16 + math.cos(angle) * 30
            arrow_y = player_y + 16 + math.sin(angle) * 30
            pygame.draw.circle(screen, COLORS['coin'], (int(arrow_x), int(arrow_y)), 4)
    
    # UI bar
    ui_bar = pygame.Rect(0, 0, SCREEN_WIDTH, 50)
    pygame.draw.rect(screen, COLORS['ui_bg'], ui_bar)
    pygame.draw.rect(screen, COLORS['ui_border'], ui_bar, 2)
    
    title = title_font.render("COIN QUEST AI", False, COLORS['coin'])
    screen.blit(title, (10, 10))
    
    # Time display
    hours = int(game_time // 60)
    minutes = int(game_time % 60)
    time_text = font.render(f"{hours:02d}:{minutes:02d} {get_time_of_day().upper()}", False, COLORS['text'])
    screen.blit(time_text, (SCREEN_WIDTH - 150, 10))
    
    # Coins and keys
    coin_text = title_font.render(f"${coins_collected}", False, COLORS['coin'])
    screen.blit(coin_text, (SCREEN_WIDTH - 150, 28))
    
    if keys_collected > 0:
        key_text = font.render(f"Keys: {keys_collected}/3", False, COLORS['key'])
        screen.blit(key_text, (300, 18))
    
    # Quest tracker
    y_offset = 60
    for quest_name, quest in quests.items():
        if quest['active'] and not quest['complete']:
            if quest_name == 'tom_coins':
                quest_text = font.render(f"Quest: Find coins for Tom ({quest['progress']}/5)", False, COLORS['coin'])
            elif quest_name == 'find_dog':
                quest_text = font.render(f"Quest: Find Susie's dog", False, COLORS['coin'])
            screen.blit(quest_text, (10, y_offset))
            y_offset += 22
    
    # Controls
    draw_retro_text("ARROWS:Move | E:Talk | S:Shop | ESC:Quit", 10, SCREEN_HEIGHT - 25)
    
    # Show shop
    if showing_shop:
        draw_shop_menu()
    
    # Show dialogue
    if showing_dialogue:
        draw_dialogue_box(current_npc_name, current_dialogue)
    
    # Win screen
    if game_won:
        overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
        overlay.set_alpha(180)
        overlay.fill((0, 0, 32))
        screen.blit(overlay, (0, 0))
        
        win_box = pygame.Rect(150, 200, 500, 200)
        pygame.draw.rect(screen, (0, 0, 0), win_box.inflate(8, 8))
        pygame.draw.rect(screen, COLORS['ui_bg'], win_box)
        pygame.draw.rect(screen, COLORS['coin'], win_box, 4)
        
        win_text = big_font.render("VICTORY!", False, COLORS['coin'])
        win_rect = win_text.get_rect(center=(SCREEN_WIDTH//2, 260))
        shadow_text = big_font.render("VICTORY!", False, (0, 0, 0))
        screen.blit(shadow_text, (win_rect.x + 3, win_rect.y + 3))
        screen.blit(win_text, win_rect)
        
        congrats = title_font.render(f"You collected ${coins_collected}!", False, COLORS['text'])
        congrats_rect = congrats.get_rect(center=(SCREEN_WIDTH//2, 320))
        screen.blit(congrats, congrats_rect)
        
        if frame_count % 60 < 30:
            play_again = title_font.render("[SPACE] to play again", False, COLORS['coin'])
            again_rect = play_again.get_rect(center=(SCREEN_WIDTH//2, 360))
            screen.blit(play_again, again_rect)
    
    # Game Over screen
    if game_over:
        overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
        overlay.set_alpha(180)
        overlay.fill((32, 0, 0))
        screen.blit(overlay, (0, 0))
        
        game_over_box = pygame.Rect(100, 150, 600, 300)
        pygame.draw.rect(screen, (0, 0, 0), game_over_box.inflate(8, 8))
        pygame.draw.rect(screen, COLORS['ui_bg'], game_over_box)
        pygame.draw.rect(screen, (200, 0, 0), game_over_box, 4)
        
        # Display sis_man image if available
        if has_sis_man:
            img_x = game_over_box.x + 20
            img_y = game_over_box.y + 80
            screen.blit(sis_man_image, (img_x, img_y))
            
            # Text on the right side of the image
            text_x = img_x + 170
            game_over_text = big_font.render("GAME OVER!", False, (255, 50, 50))
            shadow_text = big_font.render("GAME OVER!", False, (0, 0, 0))
            screen.blit(shadow_text, (text_x + 3, game_over_box.y + 60 + 3))
            screen.blit(game_over_text, (text_x, game_over_box.y + 60))
            
            caught = title_font.render("You were caught", False, COLORS['text'])
            screen.blit(caught, (text_x, game_over_box.y + 120))
            
            caught2 = title_font.render("by the guard!", False, COLORS['text'])
            screen.blit(caught2, (text_x, game_over_box.y + 150))
        else:
            # Fallback if image not found - centered text
            game_over_text = big_font.render("GAME OVER!", False, (255, 50, 50))
            game_over_rect = game_over_text.get_rect(center=(SCREEN_WIDTH//2, 260))
            shadow_text = big_font.render("GAME OVER!", False, (0, 0, 0))
            screen.blit(shadow_text, (game_over_rect.x + 3, game_over_rect.y + 3))
            screen.blit(game_over_text, game_over_rect)
            
            caught = title_font.render("You were caught by the guard!", False, COLORS['text'])
            caught_rect = caught.get_rect(center=(SCREEN_WIDTH//2, 320))
            screen.blit(caught, caught_rect)
        
        if frame_count % 60 < 30:
            try_again = title_font.render("[SPACE] to try again", False, (255, 100, 100))
            again_rect = try_again.get_rect(center=(SCREEN_WIDTH//2, game_over_box.bottom - 40))
            screen.blit(try_again, again_rect)
    
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()