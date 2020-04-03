/*
 *****************************************************************
 *                   C++ Logger Toolkit Library                  *
 *                                                               *
 * C++ Logger Toolkit Library Example                            *
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


#include <iostream>
#include <string>
#include <cmath>

#include "logtk.hpp"

int main()
{
   const std::string log_name = "logtk_example";
   const std::string path     = ".";

   if (!logtk::start_logger(log_name,path))
   {
      std::cerr << "ERROR - Failed to start logger!\n";
      return 1;
   }

   const double pi2 = (3.14159265358979323846 * 3.14159265358979323846);

   logtk::log("pi^2 = %20.18f",pi2);

   logtk::log("Starting pi^2 loop");

   const std::size_t max_iterations = 10000;

   for (std::size_t i = 0; i < max_iterations; ++i)
   {
      double pi_4 = 0.0;

      for (std::size_t j = 0; j < i; ++j)
      {
         pi_4 += 1.0 / ((j + 1.0) * (j + 1.0));
      }

      double pi2_approx = 6.0 * pi_4;

      logtk::log("log[%04d] pi^2 = %20.18f\terror = %20.18f",
                 i,
                 pi2_approx,
                 std::abs(pi2_approx - pi2));
   }

   logtk::log("Completed %d iterations of pi^2 loop",static_cast<unsigned int>(max_iterations));

   if (!logtk::stop_logger())
   {
      std::cerr << "ERROR - Failed to stop logger!\n";
      return 1;
   }

   return 0;
}
