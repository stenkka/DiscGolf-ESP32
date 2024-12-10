#include <SDL.h>
#include <SDL_ttf.h>

#define FPS                 144

#define SCREEN_WIDTH        800
#define SCREEN_HEIGHT       800

#define GRAVITY             400
#define NORMAL_FORCE        400

#define PROJECTILE_VELOCITY 15
#define PROJECTILE_RADIUS   8
#define PROJECTILE_RATE     

#define PARTICLE_VELOCITY   6
#define PARTICLE_RADIUS     2

#ifndef _NEW_H_
#define _NEW_H_

struct Game {
    SDL_Window* window;
	SDL_Surface* screenSurface;
	SDL_Surface* surfaceText;
	SDL_Texture* textureText;
    SDL_Color textColor;
    char textStr[64];
	TTF_Font* myFont;
	SDL_Renderer *renderer;
	SDL_Event event;
    SDL_Rect messageRect;
    int running;

    TTF_Font* (*initFont)();
    void (*initWindow)();
    void (*render)();
} game;

struct Circle {
    int x, y, radius;
    float v, angle;
    uint8_t r, g, b;
};

struct Circle disc;

struct Ship {
    SDL_Point* trianglePoints;
    float x;
    float y;
    int hp;
    float thrust_angle;
    int r;
    float F_y_total;
    float F_x_total;
    float F_thrust;
    float a;
    float v_x;
    float v_y;
    double s_x;
    double s_y;
    double t_1, t_2;
    int on_ground;
    int numOfParticles;
    int numOfRenderParticles;
    int resetParticlesSwitch;

    struct Circle particles[10];
};

typedef struct Ship Ship;

struct Level {
    SDL_Rect floor;    
} level;

#endif
