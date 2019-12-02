#include "getImages.h"

using namespace std;

GetImages::GetImages()
{
    pipe.pipeStart();
    cout << "GETIMAGES CREATED" << endl;
}

img_struct GetImages::GetPic(void)
{
    images_struct = pipe.getFrames();
    return images_struct;
}