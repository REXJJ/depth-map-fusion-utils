#pragma once
#include <iostream>

namespace DebuggingUtilities
{
    void print(){std::cout<<std::endl;}
    template<typename T,typename... Args>
        void print(T Contents, Args... args) 
        {
            std::cout<< (Contents) <<" ";print(args...);
        }
}
