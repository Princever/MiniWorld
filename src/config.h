#ifndef _config_h_
#define _config_h_

// app parameters
#define DEBUG 0
#define FULLSCREEN 0
#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768
#define VSYNC 1
#define SCROLL_THRESHOLD 0.1
#define MAX_MESSAGES 4
#define DB_PATH "world.db"
#define USE_CACHE 1
#define DAY_LENGTH 600
#define INVERT_MOUSE 0

// rendering options
#define SHOW_LIGHTS 1
#define SHOW_PLANTS 1
#define SHOW_CLOUDS 1
#define SHOW_TREES 1
#define SHOW_ITEM 1
#define SHOW_CROSSHAIRS 1
#define SHOW_WIREFRAME 1
#define SHOW_INFO_TEXT 1
#define SHOW_CHAT_TEXT 1
#define SHOW_PLAYER_NAMES 1

// key bindings
#define WORLD_KEY_FORWARD 'W'
#define WORLD_KEY_BACKWARD 'S'
#define WORLD_KEY_LEFT 'A'
#define WORLD_KEY_RIGHT 'D'
#define WORLD_KEY_JUMP GLFW_KEY_SPACE
#define WORLD_KEY_FLY GLFW_KEY_TAB
// #define WORLD_KEY_OBSERVE 'O'
// #define WORLD_KEY_OBSERVE_INSET 'P'
#define WORLD_KEY_ITEM_NEXT 'E'
#define WORLD_KEY_ITEM_PREV 'R'
#define WORLD_KEY_ZOOM GLFW_KEY_LEFT_SHIFT
#define WORLD_KEY_ORTHO 'F'
// #define WORLD_KEY_CHAT 't'
// #define WORLD_KEY_COMMAND '/'
// #define WORLD_KEY_SIGN '`'

// advanced parameters
#define CREATE_CHUNK_RADIUS 10
#define RENDER_CHUNK_RADIUS 10
#define RENDER_SIGN_RADIUS 4
#define DELETE_CHUNK_RADIUS 14
#define CHUNK_SIZE 32
#define COMMIT_INTERVAL 5

#endif
