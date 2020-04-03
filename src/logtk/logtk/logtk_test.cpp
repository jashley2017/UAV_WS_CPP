/*
 *****************************************************************
 *                   C++ Logger Toolkit Library                  *
 *                                                               *
 * Performance Benchmark                                         *
 * Author: Arash Partow (2005)                                   *
 * URL: http://www.partow.net/programming/logtk/index.html       *
 *                                                               *
 * Copyright notice:                                             *
 * Free use of the Logger Toolkit Library is permitted under the *
 * guidelines and in accordance with the MIT License.            *
 * http://www.opensource.org/licenses/MIT                        *
 *                                                               *
 *****************************************************************
*/


#include <cstdlib>
#include <iostream>
#include <string>

#include "logtk.hpp"


int main(int argc, char* argv[])
{
   if ((3 != argc) && (4 != argc))
   {
      std::cout << "usage: logtk_test <thread count> <no. logs> <log path (optional)> \n";
      return 1;
   }

   logtk::test::run(atoi(argv[1]), atoi(argv[2]), ((argc == 3) ? "." : argv[3]));

   return 0;
}
