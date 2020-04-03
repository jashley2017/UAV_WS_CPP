/*
 *****************************************************************
 *                     Logger Toolkit Library                    *
 *                                                               *
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


#ifndef INCLUDE_LOGTK_HPP
#define INCLUDE_LOGTK_HPP


#include <string>
#include <map>


namespace logtk
{
   namespace sink_type
   {
      enum mode
      {
         e_unknown,
         e_disk
      };
   }

   bool start_logger(const sink_type::mode mode,
                     const std::map<std::string,std::string>& parameters,
                     const std::size_t initial_log_pool_size = 102400);

   // Instantiate disk sink based logger
   bool start_logger(const std::string& name,
                     const std::string& path,
                     const std::size_t initial_log_pool_size = 102400);

   bool stop_logger();

   bool logger_running();

   void log(const std::string& message);

   void rte_and_log(const std::string& message);

   void flush_logs();

   namespace details
   {
      struct time_point;

      time_point get_time_point();

      void log(const time_point& tp, const std::string& message);

      void rte_and_log(const time_point& tp, const std::string& message);

      void format_params(std::string& result, const char* format, ...);

      #define register_return_type(Type) inline Type return_type(const Type& t) { return t; }

      register_return_type(char)
      register_return_type(unsigned char)
      register_return_type(short)
      register_return_type(unsigned short)
      register_return_type(int)
      register_return_type(unsigned int)
      register_return_type(unsigned long)
      register_return_type(long long)
      register_return_type(unsigned long long)
      register_return_type(float)
      register_return_type(double)
      register_return_type(long double)

      #undef register_return_type

      const char* return_type(const char* cptr);
      const unsigned char* return_type(const unsigned char* cptr);
      const char* return_type(const std::string& s);

      template <typename T, std::size_t N>
      inline const T* return_type(const T (&a)[N])
      {
         return reinterpret_cast<const T*>(&a[0]);
      }

      template <typename T>
      inline const T* return_type(const T* t)
      {
         return t;
      }
   }

   template <typename T1>
   inline void log(const char* format, const T1& t1)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1));
      logtk::log(result);
   }

   template <typename T1, typename T2>
   inline void log(const char* format,
                   const T1& t1, const T2& t2)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2));
      logtk::log(result);
   }

   template <typename T1, typename T2, typename T3>
   inline void log(const char* format,
                   const T1& t1, const T2& t2, const T3& t3)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3));
      logtk::log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4>
   inline void log(const char* format,
                   const T1& t1, const T2& t2, const T3& t3, const T4& t4)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3),
                                    details::return_type(t4));
      logtk::log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4, typename T5>
   inline void log(const char* format,
                   const T1& t1, const T2& t2, const T3& t3, const T4& t4, const T5& t5)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3),
                                    details::return_type(t4),
                                    details::return_type(t5));
      logtk::log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4,
             typename T5, typename T6>
   inline void log(const char* format,
                   const T1& t1, const T2& t2, const T3& t3, const T4& t4,
                   const T5& t5, const T6& t6)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3),
                                    details::return_type(t4),
                                    details::return_type(t5),
                                    details::return_type(t6));
      logtk::log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4,
             typename T5, typename T6, typename T7>
   inline void log(const char* format,
                   const T1& t1, const T2& t2, const T3& t3, const T4& t4,
                   const T5& t5, const T6& t6, const T7& t7)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3),
                                    details::return_type(t4),
                                    details::return_type(t5),
                                    details::return_type(t6),
                                    details::return_type(t7));
      logtk::log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4,
             typename T5, typename T6, typename T7, typename T8>
   inline void log(const char* format,
                   const T1& t1, const T2& t2, const T3& t3, const T4& t4,
                   const T5& t5, const T6& t6, const T7& t7, const T8& t8)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3),
                                    details::return_type(t4),
                                    details::return_type(t5),
                                    details::return_type(t6),
                                    details::return_type(t7),
                                    details::return_type(t8));
      logtk::log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4,
             typename T5, typename T6, typename T7, typename T8,
             typename T9>
   inline void log(const char* format,
                   const T1& t1, const T2& t2, const T3& t3, const T4& t4,
                   const T5& t5, const T6& t6, const T7& t7, const T8& t8,
                   const T9& t9)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3),
                                    details::return_type(t4),
                                    details::return_type(t5),
                                    details::return_type(t6),
                                    details::return_type(t7),
                                    details::return_type(t8),
                                    details::return_type(t9));
      logtk::log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4,
             typename T5, typename T6, typename T7, typename T8,
             typename T9, typename T10>
   inline void log(const char* format,
                   const T1& t1, const T2& t2, const T3& t3, const T4& t4,
                   const T5& t5, const T6& t6, const T7& t7, const T8& t8,
                   const T9& t9, const T10& t10)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type( t1),
                                    details::return_type( t2),
                                    details::return_type( t3),
                                    details::return_type( t4),
                                    details::return_type( t5),
                                    details::return_type( t6),
                                    details::return_type( t7),
                                    details::return_type( t8),
                                    details::return_type( t9),
                                    details::return_type(t10));
      logtk::log(result);
   }

   template <typename T1>
   inline void rte_and_log(const char* format, const T1& t1)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1));
      logtk::rte_and_log(result);
   }

   template <typename T1, typename T2>
   inline void rte_and_log(const char* format,
                           const T1& t1, const T2& t2)
   {
      std::string result;
      details::format_params(result,
                             format,
                             details::return_type(t1),
                             details::return_type(t2));
      logtk::rte_and_log(result);
   }

   template <typename T1, typename T2, typename T3>
   inline void rte_and_log(const char* format,
                           const T1& t1, const T2& t2, const T3& t3)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3));
      logtk::rte_and_log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4>
   inline void rte_and_log(const char* format,
                           const T1& t1, const T2& t2, const T3& t3, const T4& t4)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3),
                                    details::return_type(t4));
      logtk::rte_and_log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4, typename T5>
   inline void rte_and_log(const char* format,
                           const T1& t1, const T2& t2, const T3& t3, const T4& t4, const T5& t5)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3),
                                    details::return_type(t4),
                                    details::return_type(t5));
      logtk::rte_and_log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4,
             typename T5, typename T6>
   inline void rte_and_log(const char* format,
                           const T1& t1, const T2& t2, const T3& t3, const T4& t4,
                           const T5& t5, const T6& t6)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3),
                                    details::return_type(t4),
                                    details::return_type(t5),
                                    details::return_type(t6));
      logtk::rte_and_log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4,
             typename T5, typename T6, typename T7>
   inline void rte_and_log(const char* format,
                           const T1& t1, const T2& t2, const T3& t3, const T4& t4,
                           const T5& t5, const T6& t6, const T7& t7)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3),
                                    details::return_type(t4),
                                    details::return_type(t5),
                                    details::return_type(t6),
                                    details::return_type(t7));
      logtk::rte_and_log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4,
             typename T5, typename T6, typename T7, typename T8>
   inline void rte_and_log(const char* format,
                           const T1& t1, const T2& t2, const T3& t3, const T4& t4,
                           const T5& t5, const T6& t6, const T7& t7, const T8& t8)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3),
                                    details::return_type(t4),
                                    details::return_type(t5),
                                    details::return_type(t6),
                                    details::return_type(t7),
                                    details::return_type(t8));
      logtk::rte_and_log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4,
             typename T5, typename T6, typename T7, typename T8,
             typename T9>
   inline void rte_and_log(const char* format,
                           const T1& t1, const T2& t2, const T3& t3, const T4& t4,
                           const T5& t5, const T6& t6, const T7& t7, const T8& t8,
                           const T9& t9)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type(t1),
                                    details::return_type(t2),
                                    details::return_type(t3),
                                    details::return_type(t4),
                                    details::return_type(t5),
                                    details::return_type(t6),
                                    details::return_type(t7),
                                    details::return_type(t8),
                                    details::return_type(t9));
      logtk::rte_and_log(result);
   }

   template <typename T1, typename T2, typename T3, typename T4,
             typename T5, typename T6, typename T7, typename T8,
             typename T9, typename T10>
   inline void rte_and_log(const char* format,
                           const T1& t1, const T2& t2, const T3& t3, const T4& t4,
                           const T5& t5, const T6& t6, const T7& t7, const T8& t8,
                           const T9& t9, const T10& t10)
   {
      std::string result;
      logtk::details::format_params(result,
                                    format,
                                    details::return_type( t1),
                                    details::return_type( t2),
                                    details::return_type( t3),
                                    details::return_type( t4),
                                    details::return_type( t5),
                                    details::return_type( t6),
                                    details::return_type( t7),
                                    details::return_type( t8),
                                    details::return_type( t9),
                                    details::return_type(t10));
      logtk::rte_and_log(result);
   }

   namespace test
   {
      void run(const std::size_t& thread_count, const std::size_t& max_long_count, const std::string& path = "");
   }

}

#endif
