/**
 * @file test.cpp
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "nav_base.h"
#include "nav_sl.h"

int main(){
   std::shared_ptr<base::NavigationBase> nav = std::make_shared<sl::NavigationStraightLine>("Nav");
   nav->Plan();
}