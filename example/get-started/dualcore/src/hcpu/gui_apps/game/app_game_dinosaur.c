/**
 ******************************************************************************
 * @file   app_game_dinosaur.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#ifdef BSP_USING_BLOC
#include "bloc_control.h"
#include "bloc_setting.h"
#include "bloc_peripheral.h"
#include "bloc_v2t.h"
#endif
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#include "ui_img_helper.h"
#endif
#ifdef BSP_USING_GESTURE_HANDLER
#include "gesture_handler.h"
#endif
#include "gesture_recognition_task.h"

#define DBG_TAG "app.game"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_GAME_DINOSAUR
LV_IMG_DECLARE(img_dinosaur);
LV_IMG_DECLARE(img_enemy);

// Game constants
#define SCREEN_WIDTH 466
#define SCREEN_HEIGHT 466
#define GROUND_Y 350
#define DINO_WIDTH 64
#define DINO_HEIGHT 64
#define DINO_X 80
#define DINO_GROUND_Y (GROUND_Y - DINO_HEIGHT)
#define OBSTACLE_WIDTH 25
#define OBSTACLE_MIN_HEIGHT 40
#define OBSTACLE_MAX_HEIGHT 60
#define OBSTACLE_SPEED 6
#define ENEMY_SIZE 40
#define ENEMY_MIN_Y 150
#define ENEMY_MAX_Y 280
#define JUMP_VELOCITY -18
#define VIRTUAL_GRAVITY 1
#define GAME_TICK_MS 30
#define OBSTACLE_SPAWN_DISTANCE 400
#define MAX_OBSTACLES 3
#define MAX_PROJECTILES 5
#define PROJECTILE_SPEED 12
#define PROJECTILE_SIZE 24

// Game state
typedef enum
{
    GAME_STATE_READY,
    GAME_STATE_RUNNING,
    GAME_STATE_OVER
} game_state_t;

typedef struct
{
    lv_obj_t *obj;
    int y;
    int velocity;
    bool is_jumping;
    int jump_count; // Track number of jumps (0, 1, or 2)
} dino_t;

typedef enum
{
    OBSTACLE_TYPE_GROUND, // Ground obstacle (cannot be destroyed)
    OBSTACLE_TYPE_ENEMY   // Flying/ground enemy (can be destroyed by projectiles)
} obstacle_type_t;

typedef struct
{
    lv_obj_t *obj;
    int x;
    int y;      // For flying enemies
    int height; // For ground obstacles
    bool active;
    obstacle_type_t type;
} obstacle_t;

typedef struct
{
    lv_obj_t *obj;
    int x;
    int y;
    bool active;
} projectile_t;

static game_state_t game_state = GAME_STATE_READY;
static dino_t dino;
static obstacle_t obstacles[MAX_OBSTACLES];
static projectile_t projectiles[MAX_PROJECTILES];
static int score = 0;
static int kills = 0; // Track destroyed obstacles
static lv_obj_t *score_label = NULL;
static lv_obj_t *kills_label = NULL;
static lv_obj_t *game_over_label = NULL;
static lv_obj_t *restart_label = NULL;
static lv_obj_t *ground_line = NULL;
static lv_obj_t *game_container = NULL;
static lv_timer_t *game_timer = NULL;
static int obstacle_spawn_timer = 0;

// Forward declarations
static void game_tick(lv_timer_t *timer);
static void reset_game(void);
static void start_game(void);
static void game_over(void);

// Check collision between dino and obstacle
static bool check_collision(void)
{
    int dino_left = DINO_X;
    int dino_right = DINO_X + DINO_WIDTH;
    int dino_top = dino.y;
    int dino_bottom = dino.y + DINO_HEIGHT;

    for (int i = 0; i < MAX_OBSTACLES; i++)
    {
        if (obstacles[i].active)
        {
            int obs_left, obs_right, obs_top, obs_bottom;

            if (obstacles[i].type == OBSTACLE_TYPE_GROUND)
            {
                // Ground obstacle collision
                obs_left = obstacles[i].x;
                obs_right = obstacles[i].x + OBSTACLE_WIDTH;
                obs_top = GROUND_Y - obstacles[i].height;
                obs_bottom = GROUND_Y;
            }
            else if (obstacles[i].type == OBSTACLE_TYPE_ENEMY)
            {
                // Enemy collision
                obs_left = obstacles[i].x;
                obs_right = obstacles[i].x + ENEMY_SIZE;
                obs_top = obstacles[i].y;
                obs_bottom = obstacles[i].y + ENEMY_SIZE;
            }

            // AABB collision detection
            if (dino_right > obs_left && dino_left < obs_right &&
                dino_bottom > obs_top && dino_top < obs_bottom)
            {
                return true;
            }
        }
    }
    return false;
}

// Create dinosaur
static void create_dino(lv_obj_t *parent)
{
    dino.obj = lv_img_create(parent);
    lv_img_set_src(dino.obj, LV_EXT_IMG_GET(img_dinosaur));
    lv_obj_set_size(dino.obj, DINO_WIDTH, DINO_HEIGHT);
    lv_obj_clear_flag(dino.obj, LV_OBJ_FLAG_SCROLLABLE);

    dino.y = DINO_GROUND_Y;
    dino.velocity = 0;
    dino.is_jumping = false;
    dino.jump_count = 0;

    lv_obj_set_pos(dino.obj, DINO_X, dino.y);
}

// Create projectile
static void create_projectile(void)
{
    for (int i = 0; i < MAX_PROJECTILES; i++)
    {
        if (!projectiles[i].active)
        {
            if (projectiles[i].obj == NULL)
            {
                projectiles[i].obj = lv_obj_create(game_container);
                lv_obj_set_size(projectiles[i].obj, PROJECTILE_SIZE, PROJECTILE_SIZE);
                lv_obj_set_style_bg_color(projectiles[i].obj, lv_color_hex(0xFF5555), LV_PART_MAIN);
                lv_obj_set_style_border_width(projectiles[i].obj, 0, LV_PART_MAIN);
                lv_obj_set_style_radius(projectiles[i].obj, PROJECTILE_SIZE / 2, LV_PART_MAIN);
                lv_obj_clear_flag(projectiles[i].obj, LV_OBJ_FLAG_SCROLLABLE);
            }

            projectiles[i].x = DINO_X + DINO_WIDTH;
            projectiles[i].y = dino.y + DINO_HEIGHT / 6 - PROJECTILE_SIZE / 2;
            projectiles[i].active = true;

            lv_obj_set_pos(projectiles[i].obj, projectiles[i].x, projectiles[i].y);
            lv_obj_clear_flag(projectiles[i].obj, LV_OBJ_FLAG_HIDDEN);
            break;
        }
    }
}

// Create obstacle (ground type or enemy type)
static void create_obstacle(int index, obstacle_type_t type)
{
    // If switching types, delete old object
    if (obstacles[index].obj != NULL && obstacles[index].type != type)
    {
        lv_obj_del(obstacles[index].obj);
        obstacles[index].obj = NULL;
    }

    obstacles[index].type = type;
    obstacles[index].x = SCREEN_WIDTH;
    obstacles[index].active = true;

    if (type == OBSTACLE_TYPE_GROUND)
    {
        // Ground obstacle - gray rectangular blocks
        if (obstacles[index].obj == NULL)
        {
            obstacles[index].obj = lv_obj_create(game_container);
            lv_obj_set_style_border_width(obstacles[index].obj, 0, LV_PART_MAIN);
            lv_obj_set_style_bg_color(obstacles[index].obj, lv_color_hex(0xCCCCCC), LV_PART_MAIN);
            lv_obj_set_style_radius(obstacles[index].obj, 3, LV_PART_MAIN);
            lv_obj_clear_flag(obstacles[index].obj, LV_OBJ_FLAG_SCROLLABLE);
        }

        obstacles[index].height = OBSTACLE_MIN_HEIGHT + (rand() % (OBSTACLE_MAX_HEIGHT - OBSTACLE_MIN_HEIGHT));
        obstacles[index].y = GROUND_Y - obstacles[index].height;

        lv_obj_set_size(obstacles[index].obj, OBSTACLE_WIDTH, obstacles[index].height);
    }
    else if (type == OBSTACLE_TYPE_ENEMY)
    {
        // Flying/ground enemy - use img_enemy image
        if (obstacles[index].obj == NULL)
        {
            obstacles[index].obj = lv_img_create(game_container);
            lv_img_set_src(obstacles[index].obj, LV_EXT_IMG_GET(img_enemy));
            lv_obj_set_size(obstacles[index].obj, ENEMY_SIZE, ENEMY_SIZE);
            lv_obj_clear_flag(obstacles[index].obj, LV_OBJ_FLAG_SCROLLABLE);
        }

        obstacles[index].y = ENEMY_MIN_Y + (rand() % (ENEMY_MAX_Y - ENEMY_MIN_Y));
        obstacles[index].height = ENEMY_SIZE;
    }

    lv_obj_set_pos(obstacles[index].obj, obstacles[index].x, obstacles[index].y);
    lv_obj_clear_flag(obstacles[index].obj, LV_OBJ_FLAG_HIDDEN);
}

// Spawn obstacles (randomly choose type)
static void spawn_obstacle(void)
{
    for (int i = 0; i < MAX_OBSTACLES; i++)
    {
        if (!obstacles[i].active)
        {
            // 60% chance ground obstacle, 40% chance enemy
            obstacle_type_t type = (rand() % 10 < 6) ? OBSTACLE_TYPE_GROUND : OBSTACLE_TYPE_ENEMY;
            create_obstacle(i, type);
            obstacle_spawn_timer = 0;
            break;
        }
    }
}

// Update projectiles
static void update_projectiles(void)
{
    for (int i = 0; i < MAX_PROJECTILES; i++)
    {
        if (projectiles[i].active)
        {
            projectiles[i].x += PROJECTILE_SPEED;

            // Check collision with obstacles (only enemies can be destroyed)
            bool hit = false;
            for (int j = 0; j < MAX_OBSTACLES; j++)
            {
                if (obstacles[j].active && obstacles[j].type == OBSTACLE_TYPE_ENEMY)
                {
                    int obs_left = obstacles[j].x;
                    int obs_right = obstacles[j].x + ENEMY_SIZE;
                    int obs_top = obstacles[j].y;
                    int obs_bottom = obstacles[j].y + ENEMY_SIZE;

                    int proj_left = projectiles[i].x;
                    int proj_right = projectiles[i].x + PROJECTILE_SIZE;
                    int proj_top = projectiles[i].y;
                    int proj_bottom = projectiles[i].y + PROJECTILE_SIZE;

                    if (proj_right > obs_left && proj_left < obs_right &&
                        proj_bottom > obs_top && proj_top < obs_bottom)
                    {
                        // Hit! Destroy both projectile and enemy
                        obstacles[j].active = false;
                        lv_obj_add_flag(obstacles[j].obj, LV_OBJ_FLAG_HIDDEN);
                        hit = true;
                        kills++;
                        score += 5; // Bonus points for destroying enemy
                        lv_label_set_text_fmt(kills_label, "Kills: %d", kills);
                        lv_label_set_text_fmt(score_label, "%d", score);
                        break;
                    }
                }
            }

            if (hit || projectiles[i].x > SCREEN_WIDTH)
            {
                projectiles[i].active = false;
                lv_obj_add_flag(projectiles[i].obj, LV_OBJ_FLAG_HIDDEN);
            }
            else
            {
                lv_obj_set_x(projectiles[i].obj, projectiles[i].x);
            }
        }
    }
}

// Update obstacles
static void update_obstacles(void)
{
    for (int i = 0; i < MAX_OBSTACLES; i++)
    {
        if (obstacles[i].active)
        {
            obstacles[i].x -= OBSTACLE_SPEED;

            // Check if obstacle went off screen
            int max_width = (obstacles[i].type == OBSTACLE_TYPE_GROUND) ? OBSTACLE_WIDTH : ENEMY_SIZE;
            if (obstacles[i].x < -max_width)
            {
                obstacles[i].active = false;
                lv_obj_add_flag(obstacles[i].obj, LV_OBJ_FLAG_HIDDEN);
                score++;
                lv_label_set_text_fmt(score_label, "%d", score);
            }
            else
            {
                lv_obj_set_pos(obstacles[i].obj, obstacles[i].x, obstacles[i].y);
            }
        }
    }

    // Spawn new obstacles
    obstacle_spawn_timer += OBSTACLE_SPEED;
    if (obstacle_spawn_timer >= OBSTACLE_SPAWN_DISTANCE)
    {
        spawn_obstacle();
    }
}

// Update dinosaur
static void update_dino(void)
{
    if (dino.is_jumping || dino.y < DINO_GROUND_Y)
    {
        dino.velocity += VIRTUAL_GRAVITY;
        dino.y += dino.velocity;

        if (dino.y >= DINO_GROUND_Y)
        {
            dino.y = DINO_GROUND_Y;
            dino.velocity = 0;
            dino.is_jumping = false;
            dino.jump_count = 0; // Reset jump count when landing
        }

        lv_obj_set_y(dino.obj, dino.y);
    }
}

// Jump action (supports double jump)
static void dino_jump(void)
{
    // Allow jump if on ground (first jump) or in air but haven't double jumped yet
    if (dino.jump_count < 2)
    {
        dino.is_jumping = true;
        dino.velocity = JUMP_VELOCITY;
        dino.jump_count++;

        // Visual feedback for double jump
        if (dino.jump_count == 2)
        {
            lv_obj_set_style_transform_zoom(dino.obj, 280, 0); // Scale up briefly
            lv_obj_set_style_transform_zoom(dino.obj, 256, LV_STATE_DEFAULT);
        }
    }
}

// Fire projectile
static void dino_fire(void)
{
    create_projectile();
}

// Game tick callback
static void game_tick(lv_timer_t *timer)
{
    if (game_state == GAME_STATE_RUNNING)
    {
        update_dino();
        update_obstacles();
        update_projectiles();

        if (check_collision())
        {
            extern void motor_pattern_scrolling_app(void);
            motor_pattern_scrolling_app();
            game_over();
        }
    }
}

// Reset game
static void reset_game(void)
{
    score = 0;
    kills = 0;
    obstacle_spawn_timer = 0;

    // Reset dino
    dino.y = DINO_GROUND_Y;
    dino.velocity = 0;
    dino.is_jumping = false;
    dino.jump_count = 0;
    lv_obj_set_pos(dino.obj, DINO_X, dino.y);

    // Reset obstacles
    for (int i = 0; i < MAX_OBSTACLES; i++)
    {
        obstacles[i].active = false;
        if (obstacles[i].obj != NULL)
        {
            lv_obj_add_flag(obstacles[i].obj, LV_OBJ_FLAG_HIDDEN);
        }
    }

    // Reset projectiles
    for (int i = 0; i < MAX_PROJECTILES; i++)
    {
        projectiles[i].active = false;
        if (projectiles[i].obj != NULL)
        {
            lv_obj_add_flag(projectiles[i].obj, LV_OBJ_FLAG_HIDDEN);
        }
    }

    // Update score display
    lv_label_set_text_fmt(score_label, "%d", score);
    lv_label_set_text_fmt(kills_label, "Kills: %d", kills);
}

// Start game
static void start_game(void)
{
    reset_game();
    game_state = GAME_STATE_RUNNING;

    if (game_over_label)
    {
        lv_obj_add_flag(game_over_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (restart_label)
    {
        lv_obj_add_flag(restart_label, LV_OBJ_FLAG_HIDDEN);
    }
}

// Game over
static void game_over(void)
{
    game_state = GAME_STATE_OVER;

    if (game_over_label)
    {
        lv_obj_clear_flag(game_over_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (restart_label)
    {
        lv_obj_clear_flag(restart_label, LV_OBJ_FLAG_HIDDEN);
    }
}

// Screen tap event
static void screen_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (event == LV_EVENT_CLICKED)
    {
        if (game_state == GAME_STATE_READY || game_state == GAME_STATE_OVER)
        {
            start_game();
        }
        else if (game_state == GAME_STATE_RUNNING)
        {
            dino_jump();
        }
    }
}

// Exit button event
static void exit_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        gui_app_self_exit();
    }
}

// Create game UI
static void create_game_ui(lv_obj_t *parent)
{
    // Create game container (dark theme)
    game_container = lv_obj_create(parent);
    lv_obj_set_size(game_container, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_align(game_container, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(game_container, lv_color_hex(0x1A1A1A), LV_PART_MAIN);
    lv_obj_set_style_border_width(game_container, 0, LV_PART_MAIN);
    lv_obj_clear_flag(game_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(game_container, screen_event_cb, LV_EVENT_CLICKED, NULL);

    // Create ground line
    ground_line = lv_obj_create(game_container);
    lv_obj_set_size(ground_line, SCREEN_WIDTH, 3);
    lv_obj_set_pos(ground_line, 0, GROUND_Y);
    lv_obj_set_style_bg_color(ground_line, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
    lv_obj_set_style_border_width(ground_line, 0, LV_PART_MAIN);
    lv_obj_clear_flag(ground_line, LV_OBJ_FLAG_SCROLLABLE);

    // Create score label
    score_label = lv_label_create(game_container);
    lv_label_set_text(score_label, "0");
    lv_obj_align(score_label, LV_ALIGN_TOP_RIGHT, -70, 45);
    lv_obj_set_style_text_font(score_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(score_label, lv_color_hex(0xFFFFFF), 0);

    // Create kills label
    kills_label = lv_label_create(game_container);
    lv_label_set_text(kills_label, "Kills: 0");
    lv_obj_align(kills_label, LV_ALIGN_TOP_RIGHT, -70, 95);
    lv_obj_set_style_text_font(kills_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(kills_label, lv_color_hex(0xFF5555), 0);

    // Create game over label
    game_over_label = lv_label_create(game_container);
    lv_label_set_text(game_over_label, "GAME OVER");
    lv_obj_align(game_over_label, LV_ALIGN_CENTER, 0, -40);
    lv_obj_set_style_text_font(game_over_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(game_over_label, lv_color_hex(0xFF5555), 0);
    lv_obj_add_flag(game_over_label, LV_OBJ_FLAG_HIDDEN);

    // Create restart label
    restart_label = lv_label_create(game_container);
    lv_label_set_text(restart_label, "Tap to restart");
    lv_obj_align(restart_label, LV_ALIGN_CENTER, 0, 10);
    lv_obj_set_style_text_font(restart_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(restart_label, lv_color_hex(0xAAAAAA), 0);
    lv_obj_add_flag(restart_label, LV_OBJ_FLAG_HIDDEN);

    // Create dinosaur
    create_dino(game_container);

    // Initialize obstacles
    for (int i = 0; i < MAX_OBSTACLES; i++)
    {
        obstacles[i].obj = NULL;
        obstacles[i].active = false;
    }

    // Initialize projectiles
    for (int i = 0; i < MAX_PROJECTILES; i++)
    {
        projectiles[i].obj = NULL;
        projectiles[i].active = false;
    }

    // Create exit button
    lv_obj_t *exit_btn = common_text_button(parent, "Exit", NULL, 100, 50, exit_btn_event_cb);
    lv_obj_t *exit_btn_label = lv_obj_get_child(exit_btn, 0);
    lv_obj_set_style_text_color(exit_btn_label, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(exit_btn, LV_ALIGN_BOTTOM_MID, 0, -20);

    // Create game timer
    game_timer = lv_timer_create(game_tick, GAME_TICK_MS, NULL);
}

static void handle_tap_indicator(uint8_t gesture)
{
    if (gesture == 1) // tap
    {
        if (game_state == GAME_STATE_READY || game_state == GAME_STATE_OVER)
        {
            start_game();
        }
        else if (game_state == GAME_STATE_RUNNING)
        {
            dino_jump();
        }
        LOG_D("Dino jump triggered by tap gesture");
    }
}

static void handle_release_indicator(void)
{
    if (game_state == GAME_STATE_RUNNING)
    {
        dino_fire();
    }
}

static lv_obj_t *on_start(lv_obj_t *parent)
{
    create_game_ui(parent);
    game_state = GAME_STATE_READY;
    return parent;
}

static void on_resume(void)
{
    setting_provider.set_power_save_mode(0);
    app_control_set_game_mode(true);
    lvgl_msg_handler.handle_ungrab_event = handle_release_indicator;
    lvgl_msg_handler.handle_tap_indicator = handle_tap_indicator;
}

static void on_pause(void)
{
    if (game_state == GAME_STATE_RUNNING)
    {
        game_state = GAME_STATE_READY;
    }
    app_control_set_game_mode(false);
    setting_provider.set_power_save_mode(1);
    if (lvgl_msg_handler.handle_ungrab_event == handle_release_indicator)
        lvgl_msg_handler.handle_ungrab_event = NULL;
    lvgl_msg_handler.handle_tap_indicator = NULL;
}

static void on_stop(void)
{
    if (game_timer)
    {
        lv_timer_del(game_timer);
        game_timer = NULL;
    }

    // Clean up obstacles
    for (int i = 0; i < MAX_OBSTACLES; i++)
    {
        if (obstacles[i].obj != NULL)
        {
            lv_obj_del(obstacles[i].obj);
            obstacles[i].obj = NULL;
        }
    }

    // Clean up projectiles
    for (int i = 0; i < MAX_PROJECTILES; i++)
    {
        if (projectiles[i].obj != NULL)
        {
            lv_obj_del(projectiles[i].obj);
            projectiles[i].obj = NULL;
        }
    }
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start(lv_scr_act());
        break;

    case GUI_APP_MSG_ONRESUME:
        on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_GAME_DINOSAUR, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(game), IMG_GAME, APP_ID_GAME_DINOSAUR, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
