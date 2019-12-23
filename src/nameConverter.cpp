#include "nameConverter.h"

using namespace std;

NameConverter::NameConverter()
{
    cout << "Created nameConverter" << endl;
}

string NameConverter::convertName(int x)
{
    //Names depended on file v1.names in yolo_config_v2 folder
    if(x == 0)
        return("Bolt");
    else if (x == 1)
    {
        return("Small nut");
    }
    else if (x == 2)
    {
        return("Big nut");
    }
    else if (x == 3)
    {
        return("Small metal profile");
    }
    else if (x == 4)
    {
        return("Small black profile");
    }
    else if (x == 5)
    {
        return("Big metal profile");
    }
    else if (x == 6)
    {
        return("Big black profile");
    }    
}