/****************************************************************************
 *
 * Proiectare cu Microprocesoare, 2018
 * Author: Iulian Matesica <iulian.matesica@gmail.com>
 *
 ****************************************************************************/

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>

#include "game.h"
#include "vector.h"
#include "ST7735R_TFT.h"

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#define GAME_WINDOW_WIDTH  ST7735R_WIDTH
#define GAME_WINDOW_HEIGHT ST7735R_HEIGHT

#define GAME_WINDOW_BG_R 0
#define GAME_WINDOW_BG_G 0
#define GAME_WINDOW_BG_B 0

#define BRICK_WIDTH  18
#define BRICK_HEIGHT 10
#define BRICK_WEAK   1
#define BRICK_HARD   2

#define BRICKS_VSPACE 7
#define BRICKS_HSPACE 6
#define BRICKS_VMAX   5
#define BRICKS_HMAX   5

/* Ball. */

#define BALL_VELOCITYX  1
#define BALL_VELOCITYY  -1
#define BALL_RADIUS     3
#define BALL_POSX       (ST7735R_WIDTH / 2)
#define BALL_POSY       (GAME_WINDOW_HEIGHT - 10 * BALL_RADIUS)
#define BALL_COLOR_R    0xFF
#define BALL_COLOR_G    0x00
#define BALL_COLOR_B    0x00

/* Paddle. */

#define PADDLE_POSX    (GAME_WINDOW_WIDTH / 2 - PADDLE_WIDTH / 2)
#define PADDLE_POSY    (GAME_WINDOW_HEIGHT - 2 * PADDLE_HEIGHT)
#define PADDLE_COLOR_R 0xFF
#define PADDLE_COLOR_G 0xFF
#define PADDLE_COLOR_B 0x00

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct color_t
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct brick_t
{
  /* Brick's color. */

  color_t color;

  /* Brick's position. */

  vector2i_s pos;

  /* Brick's life (how many hits can it take). */

  int8_t life_left;
};

enum game_status_e
{
    GAME_RUNNING,
    GAME_LOST,
    GAME_WON,
};

struct game_t
{
    /* Q: What's red and it's not good for you teeth?
     * A: A RED BRICK!
     */

    vector2i_t  ball_pos;
    vector2i_t  ball_velocity;

    /* Our happy paddle. */

    vector2i_t  paddle_pos;
    vector2i_t  paddle_old_pos;

    /* Current game status. */

    uint8_t     status;

    /* Bricks. */

    brick_t     bricks[BRICKS_VMAX][BRICKS_HMAX];
    uint8_t     bricks_alive;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static game_t game;

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static void GAME_draw_ball(void);
static void GAME_draw_bricks(void);
static void GAME_draw_paddle(void);
static void GAME_update_bricks(void);
static void GAME_physics(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void GAME_init(void)
{
    /* Empty screen. */

    ST7735R_FillRect(0, 0, ST7735R_WIDTH - 1, ST7735R_HEIGHT - 1,
                     GAME_WINDOW_BG_R, GAME_WINDOW_BG_G,
                     GAME_WINDOW_BG_B);

    /* Initial ball position. */

    game.ball_pos.x = BALL_POSX;
    game.ball_pos.y = BALL_POSY;

    /* Initial velocity of the ball. */

    game.ball_velocity.x = BALL_VELOCITYX;
    game.ball_velocity.y = BALL_VELOCITYY;

    /* Initial paddle position. */

    game.paddle_old_pos.x = 0;
    game.paddle_pos.x = PADDLE_POSX;
    game.paddle_pos.y = PADDLE_POSY;

    /* Generate bricks. */

    game.bricks_alive = BRICKS_VMAX * BRICKS_HMAX;

    for (int i = 0; i < BRICKS_VMAX; i++)
    {
        for (int j = 0; j < BRICKS_HMAX; j++)
        {
            brick_t *brick = &game.bricks[i][j];

            /* Random (bright) color for each brick. */

            brick->color.r = (rand() % 0xff) | 0x30;
            brick->color.g = (rand() % 0xff) | 0x30;
            brick->color.b = (rand() % 0xff) | 0x30;

            /* Life of the brick. */

            brick->life_left = BRICK_WEAK;

            /* Brick's position. */

            brick->pos.x = BRICKS_HSPACE + j * (BRICKS_HSPACE + BRICK_WIDTH);
            brick->pos.y = BRICKS_VSPACE + i * (BRICKS_VSPACE + BRICK_HEIGHT);
        }
    }

    /* Draw them. */

    GAME_draw_bricks();

    /* Start the game! */

    game.status = GAME_RUNNING;
}

void GAME_move_paddle(int delta)
{
    /* Do not move paddle if delta is not valid. */

    if (game.paddle_pos.x + delta <= 0)
    {
        game.paddle_pos.x = 0;
        return;
    }
    else if (game.paddle_pos.x + delta >= GAME_WINDOW_WIDTH - PADDLE_WIDTH - 1)
    {
        game.paddle_pos.x = GAME_WINDOW_WIDTH - PADDLE_WIDTH - 1;
        return;
    }

    game.paddle_pos.x += delta;
}

void GAME_loop(void)
{
    /* Check if the player lost or won. */

    if (game.status == GAME_LOST)
    {
        /* Show message. */

        ST7735R_FillRect(0, 0, ST7735R_WIDTH - 1, ST7735R_HEIGHT - 1,
                         GAME_WINDOW_BG_R, GAME_WINDOW_BG_G,
                         GAME_WINDOW_BG_B);

        ST7735R_DrawText(ST7735R_WIDTH / 2 - 25, ST7735R_HEIGHT / 2 - 20,
                         "YOU LOST!", 255, 255, 255, 0, 0, 0);

        _delay_ms(3000);
        GAME_init();
    }
    else if (game.status == GAME_WON)
    {
        /* Show message. */

        ST7735R_FillRect(0, 0, ST7735R_WIDTH - 1, ST7735R_HEIGHT - 1,
                         GAME_WINDOW_BG_R, GAME_WINDOW_BG_G,
                         GAME_WINDOW_BG_B);

        ST7735R_DrawText(ST7735R_WIDTH / 2 - 40, ST7735R_HEIGHT / 2 - 20,
                         "NICE, YOU WON!", 255, 255, 255, 0, 0, 0);

        _delay_ms(4000);
        GAME_init();
    }

    /* Check for collisions. Update bricks accordingly and change ball's
     * direction.
     */

    GAME_physics();
    GAME_update_bricks();

    GAME_draw_paddle();
    GAME_draw_ball();
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void GAME_draw_paddle()
{
    /* Do not redraw if position is not changed. */

    if (game.paddle_pos.x == game.paddle_old_pos.x)
    {
      return;
    }

    /* Remove paddle from old position. */

    ST7735R_FillRect(game.paddle_old_pos.x, game.paddle_pos.y,
                     game.paddle_old_pos.x + PADDLE_WIDTH,
                     game.paddle_pos.y + PADDLE_HEIGHT,
                     GAME_WINDOW_BG_R, GAME_WINDOW_BG_G,
                     GAME_WINDOW_BG_B);

    game.paddle_old_pos.x = game.paddle_pos.x;

    /* Draw paddle in new position. */

    ST7735R_FillRect(game.paddle_pos.x, game.paddle_pos.y,
                     game.paddle_pos.x + PADDLE_WIDTH,
                     game.paddle_pos.y + PADDLE_HEIGHT, PADDLE_COLOR_R,
                     PADDLE_COLOR_G, PADDLE_COLOR_B);
}

static void GAME_draw_ball(void)
{
    /* Remove ball from old position. */

    ST7735R_FilledCircle(game.ball_pos.x, game.ball_pos.y, BALL_RADIUS,
                         GAME_WINDOW_BG_R, GAME_WINDOW_BG_G,
                         GAME_WINDOW_BG_B);

    /* Update ball's position based on its speed. */

    game.ball_pos.x += game.ball_velocity.x;
    game.ball_pos.y += game.ball_velocity.y;

    /* Do not draw if ball went outside. */

    if (game.ball_pos.x < 0 || game.ball_pos.x > GAME_WINDOW_WIDTH)
    {
        return;
    }

    if (game.ball_pos.y < 0 || game.ball_pos.y > GAME_WINDOW_HEIGHT)
    {
        return;
    }

    /* Draw ball in new position. */

    ST7735R_FilledCircle(game.ball_pos.x, game.ball_pos.y, BALL_RADIUS,
                         BALL_COLOR_R, BALL_COLOR_G, BALL_COLOR_B);
}

static void GAME_update_bricks(void)
{
    for (int i = 0; i < BRICKS_VMAX; i++)
    {
        for (int j = 0; j < BRICKS_HMAX; j++)
        {
            brick_t *brick = &game.bricks[i][j];

            /* Brick already dead. */

            if (brick->life_left == -1)
            {
                continue;
            }

            /* We have to remove the brick. */

            if (brick->life_left == 0)
            {
                /* Brick won't get removed again. */

                brick->life_left--;

                ST7735R_FillRect(brick->pos.x, brick->pos.y,
                                 brick->pos.x + BRICK_WIDTH,
                                 brick->pos.y + BRICK_HEIGHT,
                                 GAME_WINDOW_BG_R,
                                 GAME_WINDOW_BG_G,
                                 GAME_WINDOW_BG_B);

                game.bricks_alive--;
                if (game.bricks_alive == 0)
                {
                  game.status = GAME_WON;
                  return;
                }

                continue;
            }
        }
    }
}

static void GAME_draw_bricks(void)
{
    for (int i = 0; i < BRICKS_VMAX; i++)
    {
        for (int j = 0; j < BRICKS_HMAX; j++)
        {
            brick_t *brick = &game.bricks[i][j];

            /* Brick already dead. */

            if (brick->life_left <= 0)
            {
                continue;
            }

            /* Brick still alive, draw it. */

            ST7735R_FillRect(brick->pos.x, brick->pos.y,
                             brick->pos.x + BRICK_WIDTH,
                             brick->pos.y + BRICK_HEIGHT,
                             brick->color.r, brick->color.g, brick->color.b);
        }
    }
}

static void GAME_physics(void)
{
    vector2i_t ball_left_top;
    vector2i_t ball_right_bottom;

    vector2i_t paddle_left_top;
    vector2i_t paddle_right_bottom;

    vector2i_t brick_left_top;
    vector2i_t brick_right_bottom;

    /* Wrap the ball in a rectangle. It's easier (and faster) this way.
     * +-----+
     * |/---\|
     * |\---/|
     * +-----+
     *
     * Pre-calculate ball's top-left corner and bottom-right corner
     * coordinates.
     *
     * We're using (BALL_RADIUS + 1) instead of BALL_RADIUS because we do not
     * want to overlap ball with bricks/paddle. Overlapping would "eat" pixels
     * out of the paddle/brick.
     */

    ball_left_top.x = game.ball_pos.x - (BALL_RADIUS + 1);
    ball_left_top.y = game.ball_pos.y - (BALL_RADIUS + 1);

    ball_right_bottom.x = game.ball_pos.x + (BALL_RADIUS + 1);
    ball_right_bottom.y = game.ball_pos.y + (BALL_RADIUS + 1);

    /* Pre-calculate paddle's top-left corner and bottom-right corner
     * coordinates.
     */

    paddle_left_top.x = game.paddle_pos.x;
    paddle_left_top.y = game.paddle_pos.y;

    paddle_right_bottom.x = game.paddle_pos.x + PADDLE_WIDTH;
    paddle_right_bottom.y = game.paddle_pos.y + PADDLE_HEIGHT;

    /* Check for left wall collision. */

    if (ball_left_top.x <= 0)
    {
        game.ball_velocity.x =- game.ball_velocity.x;
    }

    /* Check for right wall collision. */

    if (ball_right_bottom.x >= GAME_WINDOW_WIDTH)
    {
        game.ball_velocity.x =- game.ball_velocity.x;
    }

    /* Check for ceil collision. */

    if (ball_left_top.y <= 0)
    {
        game.ball_velocity.y =- game.ball_velocity.y;
    }

    /* Check for floor collision. */

    if (ball_left_top.y >= GAME_WINDOW_HEIGHT)
    {
        game.status = GAME_LOST;
    }

    /* Check for Y axis paddle collision. */

    if (ball_right_bottom.y >= paddle_left_top.y)
    {
        /* Check for X axis paddle collision. */

        if (ball_right_bottom.x >= paddle_left_top.x &&
            ball_left_top.x <= paddle_right_bottom.x)
        {
            /* Yup, ball collision with paddle. */

            game.ball_velocity.y = -game.ball_velocity.y;

            if (ball_right_bottom.x < paddle_right_bottom.x - PADDLE_WIDTH/2)
            {
                game.ball_velocity.x = -abs(game.ball_velocity.x);
            }
            else
            {
                game.ball_velocity.x = abs(game.ball_velocity.x);
            }

            /* No other collision is possible. */

            return;
        }
    }

    /* Check ball collision with bricks. */

    for (int i = 0; i < BRICKS_VMAX; i++)
    {
        for (int j = 0; j < BRICKS_HMAX; j++)
        {
            brick_t *brick = &game.bricks[i][j];

            /* Brick already dead. It shouldn't even be rendered. */

            if (brick->life_left <= 0)
            {
                continue;
            }

            /* Check for collision with this brick. We also have to adjust
             * ball's velocity vector accordingly, based on the brick's face
             * that was hit.
             */

            brick_left_top       = brick->pos;
            brick_right_bottom.x = brick_left_top.x + BRICK_WIDTH;
            brick_right_bottom.y = brick_left_top.y + BRICK_HEIGHT;

            if (ball_right_bottom.x < brick_left_top.x ||
                ball_left_top.x > brick_right_bottom.x)
            {
                /* No chance for collision. */

                continue;
            }

            if (ball_left_top.y > brick_right_bottom.y ||
                ball_right_bottom.y < brick_left_top.y)
            {
                /* No chance for collision. */

                continue;
            }

            /* We have a collision with this brick. Update brick's life.
             * Need to find out on which side.
             */

            brick->life_left--;

            /* Test for brick's bottom face. */

            if (ball_left_top.y <= brick_right_bottom.y)
            {
              game.ball_velocity.y = -game.ball_velocity.y;
              continue;
            }

            /* Test for brick's top face. */

            if (ball_right_bottom.y >= brick_left_top.y)
            {
              game.ball_velocity.y = -game.ball_velocity.y;
              continue;
            }

            /* Test for brick's left face. */

            if (ball_right_bottom.x >= brick_left_top.x)
            {
              game.ball_velocity.x = -game.ball_velocity.x;
              continue;
            }

            /* Test for brick's right face. */

            if (ball_left_top.x <= brick_right_bottom.x)
            {
              game.ball_velocity.x = -game.ball_velocity.x;
              continue;
            }
        }
    }
}
