
/*******************************************************************************
* image
* filename: C:/Repos/SpecializedSolutions/QCM/Software/DC2017/Src/demos/dc2017/resources/bat_full.xml
* name: bat_full
*
* preset name: Monochrome
* data block size: 8 bit(s), uint8_t
* RLE compression enabled: no
* conversion type: Monochrome, Edge 128
* bits per pixel: 1
*
* preprocess:
*  main scan direction: top_to_bottom
*  line scan direction: forward
*  inverse: no
*******************************************************************************/

/*
 typedef struct {
     const uint8_t *data;
     uint16_t width;
     uint16_t height;
     } tImage;
*/
#include <stdint.h>
#include "resources.h"


static const uint8_t image_data_bat_full[16] = {
    0xff, 0xfc,
    0x80, 0x04,
    0xbf, 0xf4,
    0xbf, 0xf6,
    0xbf, 0xf6,
    0xbf, 0xf4,
    0x80, 0x04,
    0xff, 0xfc
};
const tImage bat_full = { image_data_bat_full, 15, 8};

