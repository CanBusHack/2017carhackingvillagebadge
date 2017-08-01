
/*******************************************************************************
* image
* filename: C:/Repos/SpecializedSolutions/QCM/Software/DC2017/Src/demos/dc2017/resources/bat_75.xml
* name: bat_75
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


static const uint8_t image_data_bat_75[16] = {
    0xff, 0xfc,
    0x80, 0x04,
    0xbf, 0xc4,
    0xbf, 0xc6,
    0xbf, 0xc6,
    0xbf, 0xc4,
    0x80, 0x04,
    0xff, 0xfc
};
const tImage bat_75 = { image_data_bat_75, 15, 8};

