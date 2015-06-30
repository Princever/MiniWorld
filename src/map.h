#ifndef _map_h_
#define _map_h_

#define EMPTY_ENTRY(entry) ((entry)->value == 0)

#define MAP_FOR_EACH(map, ex, ey, ez, ew) \
    for (unsigned int i = 0; i <= map->mask; i++) { \
        MapEntry *entry = map->data + i; \
        if (EMPTY_ENTRY(entry)) { \
            continue; \
        } \
        int ex = entry->e.x + map->dx; \
        int ey = entry->e.y + map->dy; \
        int ez = entry->e.z + map->dz; \
        int ew = entry->e.w;

#define END_MAP_FOR_EACH }

typedef union {                 //地图项
    unsigned int value;         //值
    struct {
        unsigned char x;        //x坐标
        unsigned char y;        //y坐标
        unsigned char z;        //z坐标
        char w;                 //方块类型
    } e;
} MapEntry;

typedef struct {                //区块的详细地图
    int dx;                     //x增量
    int dy;                     //y增量
    int dz;                     //z增量
    unsigned int mask;          //数量
    unsigned int size;          //大小
    MapEntry *data;             //数据
} Map;

void map_alloc(Map *map, int dx, int dy, int dz, int mask);
void map_free(Map *map);
void map_copy(Map *dst, Map *src);
void map_grow(Map *map);
int map_set(Map *map, int x, int y, int z, int w);
int map_get(Map *map, int x, int y, int z);

#endif
