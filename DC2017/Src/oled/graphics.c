#include "datatypes.h"
#include "oled.h"
#include "resources.h"
#include <string.h>

void graphics_draw_bitmap_mono(int const x, int const y, const tImage *image)
{
    int x0, y0, i, bit_count, byte_count;
    char cur_value;

    x0 = 0;
    y0 = 0;
    bit_count = 8;
    byte_count = 0;

    //assumes image is stored as a stream of bits
    for (i = 0; i < (image->height * image->width); i++)
    {
        if (bit_count == 8)
        {
            cur_value = image->data[byte_count++];
            bit_count = 0;
        }

        if (cur_value & 0x80)
        {
            oled_put_pixel(x0 + x, y0 + y);
        }

        cur_value <<= 1;

        x0++;
        bit_count++;

        if (x0 == image->width)
        {
            y0++;
            x0 = 0;
            bit_count = 8;
        }

    }
}

static tChar *find_char_by_code(int const code, const tFont *font)
{
    /* all fonts start with 0x20, which is space */
    if ((code - 0x20) >= font->length)
    {
        return &(font->chars[0x7E - 0x20]);
    }

    return &(font->chars[code - 0x20]);
}

void graphics_put_text(const char *str, int const x, int const y, const tFont *font)
{
    int len = strlen(str);
    int index = 0;
    int code = 0;
    int x1 = x;
    int y1 = y;

    code = str[index];
    tChar *ch = find_char_by_code(code, font);

    while (index < len)
    {

        graphics_draw_bitmap_mono(x1, y1, ch->image);

        x1 += ch->image->width;

        index++;

        code = str[index];
        ch = find_char_by_code(code, font);

        if ((x1 + ch->image->width) >= 128)
        {
            /* wrap */
            x1 = x;
            y1 += ch->image->height;

            if (y1 >= 128)
            {
                /* not enough room */
                return;
            }
        }

    }
}
