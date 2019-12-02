#ifndef GETIMAGES_H
#define GETIMAGES_H

#include "img_struct.h"
#include <iostream>
#include "pipeline.h"

using namespace std;

class GetImages
{
    private:
        img_struct images_struct;
        Pipeline pipe;
    public:
        GetImages();
        img_struct GetPic(void);
};

#endif