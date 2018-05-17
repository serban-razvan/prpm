#ifndef _GAME_H_
#define _GAME_H_

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* Paddle. */

#define PADDLE_WIDTH   28
#define PADDLE_HEIGHT  3

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void GAME_init(void);
void GAME_move_paddle(int delta);
void GAME_loop(void);

#endif
