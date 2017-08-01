#ifndef RESOURCES_H
#define RESOURCES_H

typedef struct
{
    const unsigned char *data;
    uint16_t width;
    uint16_t height;
    uint8_t  dataSize;
} tImage;

typedef struct
{
    char code;
    const tImage *image;
} tChar;

typedef struct
{
    int length;
    const tChar *chars;
} tFont;

extern const tFont font_small;
extern const tFont font_medium;
extern const tFont font_large;


#endif
