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


#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <utility>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/date_time/local_time_adjustor.hpp>
#include <boost/date_time/c_local_time_adjustor.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include "logtk.hpp"
#define strtk_no_tr1_or_boost
#include "strtk.hpp"


namespace logtk
{
   namespace details
   {
      typedef unsigned long long uint64;
      typedef unsigned short     uint16;
      typedef boost::posix_time::ptime ptime;
      typedef std::pair<boost::posix_time::ptime,std::string> log_msg_t;
      typedef std::vector<log_msg_t> log_msg_list_t;
      typedef std::map<std::string,std::string> params_t;

      bool lml_compare(const log_msg_t& m0, const log_msg_t& m1)
      {
         return (m0.first < m1.first);
      }

      const char* return_type(const char* cptr)
      {
         return cptr;
      }

      const unsigned char* return_type(const unsigned char* cptr)
      {
         return cptr;
      }

      const char* return_type(const std::string& s)
      {
         return s.data();
      }

      struct datetime
      {
         unsigned short year;
         unsigned short month;
         unsigned short day;
         unsigned short hour;
         unsigned short minute;
         unsigned short second;
         unsigned int   microsecond;
      };

      std::string time_to_str(const boost::posix_time::ptime pt)
      {
         std::ostringstream datetime_ss;
         boost::posix_time::time_facet * p_time_output = new boost::posix_time::time_facet;
         std::locale special_locale (std::locale(""), p_time_output);
         datetime_ss.imbue (special_locale);
         (*p_time_output).format("%Y%m%d%H%M%S");
         datetime_ss << pt;
         return datetime_ss.str();
      }

      int get_local_tz_offset()
      {
         typedef boost::date_time::c_local_adjustor<boost::posix_time::ptime> local_adjustor;
         ptime   utc_now = boost::posix_time::microsec_clock::universal_time();
         ptime local_now = local_adjustor::utc_to_local(utc_now);
         return ((local_now - utc_now).total_seconds() / 3600);
      }

      std::string get_utc_offset()
      {
         std::string result = "_UTC";
         int utc_offset = get_local_tz_offset();
         result += ((utc_offset >= 0) ? "+" : "-");
         result += strtk::text::right_align(2,'0',std::abs(utc_offset));
         return result;
      }

      datetime get_current_adjusted_datetime()
      {
         typedef boost::date_time::c_local_adjustor<boost::posix_time::ptime> local_adjustor;
         typedef boost::posix_time::ptime ptime;
         ptime now = boost::posix_time::microsec_clock::universal_time();
         now = local_adjustor::utc_to_local(now);
         datetime result;
         result.year        = now.date().year();
         result.month       = now.date().month();
         result.day         = now.date().day();
         result.hour        = now.time_of_day().hours();
         result.minute      = now.time_of_day().minutes();
         result.second      = now.time_of_day().seconds();
         result.microsecond = static_cast<unsigned int>(now.time_of_day().fractional_seconds() / 1000000.0);
         return result;
      }

      boost::posix_time::ptime datetime_to_ptime(const datetime& dt)
      {
         return
            boost::posix_time::ptime(
                                       boost::gregorian::date
                                          ( dt.year, dt.month, dt.day),
                                       boost::posix_time::time_duration
                                          ( dt.hour, dt.minute, dt.second,
                                          static_cast<std::size_t>(dt.microsecond / 1000000.0))
                                    );
      }

      static const std::size_t max_log_message_size = 64 * 1024; //64KB

      static const char dtgen_lut[] =
                           {
                              "00010203040506070809101112131415161718192021222324"
                              "25262728293031323334353637383940414243444546474849"
                              "50515253545556575859606162636465666768697071727374"
                              "75767778798081828384858687888990919293949596979899"
                           };

      class datetime_generator
      {
      public:

         //Format: YYYY-MM-DD HH:MM:SS.MMMMMM
         //Note: Format is 26 bytes long.

         enum { format_length = 26 };

         datetime_generator()
         {
            update();
         }

         void update()
         {
            const boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
            year_   = time.date().year();
            month_  = time.date().month();
            day_    = time.date().day();
            hour_   = time.time_of_day().hours();
            minute_ = time.time_of_day().minutes();

            std::memcpy(&static_buff_[ 0], &dtgen_lut[(year_ / 100) * 2], 2);
            std::memcpy(&static_buff_[ 2], &dtgen_lut[(year_ % 100) * 2], 2);
            std::memcpy(&static_buff_[ 5], &dtgen_lut[month_        * 2], 2);
            std::memcpy(&static_buff_[ 8], &dtgen_lut[day_          * 2], 2);
            std::memcpy(&static_buff_[11], &dtgen_lut[hour_         * 2], 2);
            std::memcpy(&static_buff_[14], &dtgen_lut[minute_       * 2], 2);

            static_buff_[ 4] = '-';
            static_buff_[ 7] = '-';
            static_buff_[10] = ' ';
            static_buff_[13] = ':';
            static_buff_[16] = ':';
         }

         void operator()(std::string& buffer)
         {
            if (buffer.size() < format_length)
               return;
            else
               operator()(const_cast<char*>(buffer.data()));
         }

         void operator()(char* buffer)
         {
            operator()(boost::posix_time::microsec_clock::local_time(),buffer);
         }

         void operator()(const boost::posix_time::ptime& time, char* output_buffer)
         {
            if (time.time_of_day().minutes() != minute_)
            {
               update();
            }

            const unsigned int sec_index   = time.time_of_day().seconds() * 2;
                  unsigned int microsecond = static_cast<unsigned int>(
                                                time.time_of_day().fractional_seconds() * (1000000.0 / time.time_of_day().ticks_per_second()));

            std::memcpy(&output_buffer[ 0], &static_buff_                       , 17);
            std::memcpy(&output_buffer[17], &dtgen_lut[sec_index]               ,  2);
            std::memcpy(&output_buffer[20], &dtgen_lut[(microsecond / 10000) * 2], 2); microsecond %= 10000;
            std::memcpy(&output_buffer[22], &dtgen_lut[(microsecond /   100) * 2], 2);
            std::memcpy(&output_buffer[24], &dtgen_lut[(microsecond %   100) * 2], 2);
            output_buffer[19] = '.';
         }

         void operator()(const boost::posix_time::ptime& time, unsigned char* output_buffer)
         {
            operator()(time,reinterpret_cast<char*>(output_buffer));
         }

         std::string as_string()
         {
            return as_string(boost::posix_time::microsec_clock::local_time());
         }

         std::string as_string(const boost::posix_time::ptime& time)
         {
            std::string result(format_length,'\0');
            operator()(time,const_cast<char*>(result.data()));
            return result;
         }

      private:

         unsigned int     year_;
         unsigned short  month_;
         unsigned short    day_;
         int              hour_;
         int            minute_;
         char      static_buff_[19];
      };

      class sink_interface
      {
      public:

         virtual ~sink_interface() {}

         virtual bool start() = 0;

         virtual bool stop() = 0;

         virtual void write_logs(log_msg_list_t& log_msg_list) = 0;

         virtual std::string details() = 0;

         virtual void set_parameters(const params_t&) {}

         virtual void update() {}
      };

      class disk_sink : public sink_interface
      {
      public:

         disk_sink()
         : perform_time_sort_(false)
         {}

         bool start()
         {
            if (!path_.empty())
            {
               if (!boost::filesystem::exists(path_) || !boost::filesystem::is_directory(path_))
               {
                  std::cerr << "LOGGER ERROR - Invalid path/directory: " << path_ << std::endl;
                  return false;
               }
            }

            datetime_generator_.update();
            std::string datetime_str = datetime_generator_.as_string();
            strtk::replace(' ','_',datetime_str);
            strtk::remove_inplace(':',datetime_str);
            file_name_ = path_ + name_ + datetime_str + get_utc_offset() + ".log";

            if (!create_log_file())
            {
               std::cerr << "LOGGER ERROR - Failed to create log file: " << file_name_ << std::endl;
               return false;
            }

            return true;
         }

         bool stop()
         {
            return true;
         }

         void write_logs(log_msg_list_t& log_msg_list)
         {
            if (log_msg_list.empty())
            {
               return;
            }

            std::ofstream stream(file_name_.c_str(), std::ios::out | std::ios::app | std::ios::binary);

            if (!stream)
            {
               static std::size_t error_count = 0;
               std::cerr << "LOGGER ERROR [" << strtk::text::right_align(3,'0',error_count++) <<"] - Failed to open log: " << file_name_ << std::endl;
               log_to_stderr(log_msg_list);
               return;
            }

            if (perform_time_sort_)
            {
               std::stable_sort(log_msg_list.begin(),log_msg_list.end(),lml_compare);
            }

            write_logs_to_disk(stream,log_msg_list);
         }

         std::string details()
         {
            return std::string("File: ") + file_name_;
         }

         void set_parameters(const params_t& params)
         {
            params_t::const_iterator itr;

            if (params.end() != (itr = params.find("name")))
            {
               name_ = itr->second;
            }
            else if (params.end() != (itr = params.find("path")))
            {
               path_ = itr->second;

               if (
                    (!path_.empty()           ) &&
                    ('/'  != *(path_.rbegin())) &&
                    ('\\' != *(path_.rbegin()))
                  )
               {
                  path_ += "/";
               }
            }
            else if (params.end() != (itr = params.find("time_sort")))
            {
               perform_time_sort_ = true;
            }
         }

         void update()
         {
            datetime_generator_.update();
         }

      private:

         void write_logs_to_disk(std::ostream& stream, const log_msg_list_t& log_msg_list)
         {
            const std::size_t dt_padding_length = datetime_generator::format_length + 2; // 2bytes = ' ' + '\n'
            const std::size_t buffer_size = 416 * 1024; //416KB

            char buffer[buffer_size];
            std::size_t bytes_written_to_buffer = 0;
            char* itr = buffer;

            for (std::size_t i = 0; i < log_msg_list.size(); ++i)
            {
               if ((bytes_written_to_buffer + log_msg_list[i].second.size() + dt_padding_length) >= buffer_size)
               {
                  stream.write(buffer,bytes_written_to_buffer);

                  bytes_written_to_buffer = 0;
                  itr = buffer;
               }

               datetime_generator_(log_msg_list[i].first,itr);
               itr += datetime_generator::format_length;
               (*itr) = ' ';
               itr += 1;

               std::memcpy(itr,log_msg_list[i].second.data(),log_msg_list[i].second.size());
               itr += log_msg_list[i].second.size();
               (*itr) = '\n';
               itr += 1;

               bytes_written_to_buffer += log_msg_list[i].second.size() + dt_padding_length;
            }

            if (bytes_written_to_buffer)
            {
               stream.write(buffer,bytes_written_to_buffer);
            }
         }

         bool create_log_file()
         {
            try
            {
               if (boost::filesystem::exists(file_name_))
               {
                  std::cerr << "LOGGER - INFO - create_log_file()  Log file: " << file_name_ << " already exists!" << std::endl;

                  std::string datetime_str = time_to_str(datetime_to_ptime(get_current_adjusted_datetime()));

                  strtk::replace(' ','_',datetime_str);
                  strtk::remove_inplace(':',datetime_str);

                  std::string rename_file_name = file_name_ + datetime_str + get_utc_offset() + ".log";

                  try
                  {
                     boost::filesystem::rename(file_name_,rename_file_name);
                  }
                  catch(std::exception& e)
                  {
                     std::cerr << "LOGGER ERROR - create_log_file()  Failed to rename log file: " << file_name_ << " to "
                               << rename_file_name << std::endl;
                     std::cerr << "LOGGER ERROR - Exception message: " << e.what() << std::endl;
                     return false;
                  }

                  std::cerr << "LOGGER - INFO - create_log_file()  Successfully renamed old log file: " << file_name_ << " to " << rename_file_name << std::endl;
               }
            }
            catch(std::exception& e)
            {
               std::cerr << "LOGGER ERROR - create_log_file()  Failed to access log file: " << file_name_ << std::endl;
               std::cerr << "LOGGER ERROR - Exception message: " << e.what() << std::endl;
               return false;
            }

            std::ofstream stream(file_name_.c_str(), std::ios::out | std::ios::ate | std::ios::binary);

            if (stream)
               return true;
            else
            {
               std::cerr << "LOGGER ERROR - create_log_file()  Failed to create log file: " << file_name_ << std::endl;
               return false;
            }
         }

         void log_to_stderr(const log_msg_list_t& log_msg_list)
         {
            if (log_msg_list.empty())
            {
               return;
            }

            write_logs_to_disk(std::cerr,log_msg_list);
            std::cerr.flush();
         }

         bool perform_time_sort_;
         std::string name_;
         std::string path_;
         std::string file_name_;
         datetime_generator datetime_generator_;
      };

      class logger
      {
      public:

         logger(const sink_type::mode mode, const params_t& params, const std::size_t initial_log_pool_size = 102400)
         : logger_running_(false),
           logger_stopped_(true),
           logger_thread_signaled_(false),
           sink_(0),
           mode_(mode),
           params_(params)
         {
            log_messages_.reserve(initial_log_pool_size);
         }

         bool start()
         {
            switch (mode_)
            {
               case sink_type::e_disk : sink_ = new disk_sink();
                                        break;

               default                : std::cerr << "LOGGER ERROR - Invalid sink!" << std::endl;
                                        return false;
            }

            sink_->set_parameters(params_);

            if (!sink_->start())
            {
               return false;
            }

            logger_running_         = true;
            logger_stopped_         = false;
            logger_thread_signaled_ = false;

            logger_thread_ = boost::make_shared<boost::thread>(&logger::run,this);
            logger_thread_->detach();

            log_message("logger::start() - INFO - Logger has started.  " + sink_->details());

            return true;
         }

         bool stop()
         {
            if (!logger_running_ && logger_stopped_)
               return true;

            log_message("logger::stop() - INFO - Logger has been stopped.");
            const unsigned int decrement_ms = 100;
            unsigned int retry              = 10000; // 10sec worth of retries.
            logger_running_                 = false;

            condition_variable_.notify_one();

            while (!logger_stopped_ && retry)
            {
               boost::this_thread::sleep(boost::posix_time::milliseconds(decrement_ms));
               retry -= decrement_ms;
            }

            delete sink_;
            sink_ = 0;

            return logger_stopped_;
         }

         void log_message(const ptime& pt, const std::string& message)
         {
            log_msg_t log = std::make_pair(pt,message);

            {
               boost::mutex::scoped_lock lock(log_messages_mutex_);
               log_messages_.push_back(log);
            }

            if (logger_thread_signaled_)
               return;
            else
            {
               logger_thread_signaled_ = true;
               condition_variable_.notify_one();
            }
         }

         void log_message(const std::string& message)
         {
            log_message(boost::posix_time::microsec_clock::local_time(),message);
         }

         void flush()
         {
            boost::mutex::scoped_lock lock(log_messages_mutex_);
            {
               if (!log_messages_.empty())
               {
                  sink_->write_logs(log_messages_);
                  log_messages_.clear();
               }

               condition_variable_.notify_one();
            }
         }

      private:

         void run()
         {
            log_msg_list_t lm_list;

            while (logger_running_)
            {
               if (logger_thread_signaled_)
               {
                  {
                     boost::mutex::scoped_lock lm_lock(log_messages_mutex_);
                     logger_thread_signaled_ = false;

                     if (!log_messages_.empty())
                     {
                        lm_list.swap(log_messages_);
                     }
                     else
                        continue;
                  }

                  sink_->write_logs(lm_list);
                  lm_list.clear();
               }
               else
               {
                  while (logger_running_ && !logger_thread_signaled_)
                  {
                     static const boost::posix_time::seconds timeout_sec(5);
                     boost::mutex::scoped_lock lock(signal_mutex_);

                     if (condition_variable_.timed_wait(lock,timeout_sec))
                     {
                        break;
                     }
                     else
                        sink_->update();
                  }
               }
            }

            for ( ; ; )
            {
               {
                  boost::mutex::scoped_lock lm_lock(log_messages_mutex_);

                  if (log_messages_.empty())
                     break;
                  else
                  {
                     sink_->write_logs(log_messages_);
                     log_messages_.clear();
                  }
               }

               boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }

            logger_stopped_ = true;
         }

         logger(const logger&);
         logger& operator=(const logger&);

         volatile bool logger_running_;
         volatile bool logger_stopped_;
         volatile bool logger_thread_signaled_;

         sink_interface* sink_;
         sink_type::mode mode_;
         params_t params_;
         boost::shared_ptr<boost::thread> logger_thread_;

         mutable boost::mutex log_messages_mutex_;
         log_msg_list_t log_messages_;

         mutable boost::mutex signal_mutex_;
         boost::condition_variable condition_variable_;

      protected:

         logger() {}
      };

      static logger* logger_instance = 0;
      static boost::mutex global_logger_mutex;
   }

   bool start_logger(const sink_type::mode mode,
                     const std::map<std::string,std::string>& parameters,
                     const std::size_t initial_log_pool_size)
   {
      boost::mutex::scoped_lock lock(logtk::details::global_logger_mutex);
      {
         if (logtk::details::logger_instance)
            return false;

         logtk::details::logger_instance = new logtk::details::logger(mode,parameters,initial_log_pool_size);

         if (!logtk::details::logger_instance->start())
         {
            delete logtk::details::logger_instance;
            logtk::details::logger_instance = 0;
            return false;
         }
         else
            return true;
      }
   }

   bool start_logger(const std::string& name, const std::string& path, const std::size_t initial_log_pool_size)
   {
      std::map<std::string,std::string> parameters;

      parameters["name"] = name;
      parameters["path"] = path;

      return start_logger(sink_type::e_disk,parameters,initial_log_pool_size);
   }

   bool stop_logger()
   {
      boost::mutex::scoped_lock lock(logtk::details::global_logger_mutex);
      {
         if (0 == logtk::details::logger_instance)
            return false;
         else if (!logtk::details::logger_instance->stop())
            return false;

         delete logtk::details::logger_instance;
         logtk::details::logger_instance = 0;

         return true;
      }
   }

   bool logger_running()
   {
      boost::mutex::scoped_lock lock(logtk::details::global_logger_mutex);
      {
         return (0 != logtk::details::logger_instance);
      }
   }

   void log(const std::string& message)
   {
      if (!message.empty() && (0 != logtk::details::logger_instance))
      {
         if (message.size() <= logtk::details::max_log_message_size)
            logtk::details::logger_instance->log_message(message);
         else
            logtk::details::logger_instance->log_message(message.substr(0,logtk::details::max_log_message_size));
      }
      else
         std::cerr << message << std::endl;
   }

   void rte_and_log(const std::string& message)
   {
      if (!message.empty() && ('\n' == message[message.size() - 1]))
      {
         log("%s",message.data());
         flush_logs();
         throw std::runtime_error(message.substr(0,message.size() - 1).data());
      }
      else
      {
         log("%s\n",message.data());
         flush_logs();
         throw std::runtime_error(message.data());
      }
   }

   void flush_logs()
   {
      boost::mutex::scoped_lock lock(logtk::details::global_logger_mutex);
      if (logtk::details::logger_instance)
      {
         logtk::details::logger_instance->flush();
      }
   }

   namespace details
   {
      struct time_point
      {
         ptime pt;
      };

      time_point get_time_point()
      {
         time_point tp;
         tp.pt = boost::posix_time::microsec_clock::local_time();
         return tp;
      }

      void log(const time_point& tp, const std::string& message)
      {
         if (!message.empty() && (0 != logtk::details::logger_instance))
         {
            if (message.size() <= logtk::details::max_log_message_size)
               logtk::details::logger_instance->log_message(tp.pt,message);
            else
               logtk::details::logger_instance->log_message(tp.pt,message.substr(0,logtk::details::max_log_message_size));
         }
         else
            std::cerr << message << std::endl;
      }

      void rte_and_log(const time_point& tp, const std::string& message)
      {
         if (!message.empty() && ('\n' == message[message.size() - 1]))
         {
            log(tp,message);
            flush_logs();
            throw std::runtime_error(message.substr(0,message.size() - 1).data());
         }
         else
         {
            logtk::log("%s\n",message.data());
            flush_logs();
            throw std::runtime_error(message.data());
         }
      }

      void format_params(std::string& result, const char* format, ...)
      {
         static const std::size_t max_size = 96 * 1024; //96KB
         char buffer[max_size];
         va_list args;
         va_start(args, format);

         int length =
         #if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
         _vsnprintf
         #else
         vsnprintf
         #endif
         (buffer, max_size, format, args);

         if ((length < 0) || (length >= static_cast<int>(max_size)))
         {
            result.clear();
            return;
         }

         va_end(args);

         result.resize(length);
         std::memcpy(const_cast<char*>(result.data()),buffer,length);
      }
   }

   namespace test
   {
      namespace details
      {
         void test_thread(const std::string& name, const std::size_t& max_log_count)
         {
            static const std::size_t array_size = 11;
            typedef unsigned char array_t[array_size];

            const std::size_t logid_padd = static_cast<std::size_t>(std::log10(static_cast<double>(max_log_count)) + 1.0);
            const std::string s = "abcdefghijklmnopqrstuvwxyz0123456789!@#$%^&*() ";

            double d = 123.456;
            array_t a;
            strtk::set_array(a,"0V1X2Y3Z4W5");

            strtk::util::timer timer;
            timer.start();

            for (std::size_t i = 0; i < max_log_count; ++i)
            {
               logtk::log("%s str: %s [%.11s] double: %12.5f  log: %s",
                          name,
                          s,
                          a,
                          d,
                          strtk::text::right_align(logid_padd,'0',i));
               d += 0.01234;
            }

            timer.stop();

            printf("logtk::test::run() - %s Total time: %8.4fsec\tRate:%10.3flogs/sec\n",
                   name.c_str(),
                   timer.time(),
                   max_log_count / timer.time());
         }
      }

      void run(const std::size_t& thread_count, const std::size_t& log_count, const std::string& path)
      {
         strtk::util::timer timer;
         timer.start();

         logtk::details::params_t logger_params;

         logger_params["name"] = "logtk_test";
         logger_params["path"] = path;

         logtk::start_logger(logtk::sink_type::e_disk,logger_params);

         logtk::log("No. threads: %02d",thread_count);
         logtk::log("Logs per thread: %d",log_count);

         std::deque<boost::shared_ptr<boost::thread> > thread_list;

         for (std::size_t i = 0; i < thread_count; ++i)
         {
            thread_list.push_back(
               boost::make_shared<boost::thread>(
                  boost::bind(
                     test::details::test_thread,
                     std::string("logger_thread") + strtk::text::right_align(2,'0',i),
                     log_count)));
         }

         for (std::size_t i = 0; i < thread_count; ++i)
         {
            thread_list[i]->join();
         }

         timer.stop();

         logtk::log("logtk::test::run() - Total time: %7.3fsec  Total logs: %d  Rate: %10.3flogs/sec\n",
                    timer.time(),
                    thread_count * log_count,
                    (thread_count * log_count) / timer.time());

         printf("logtk::test::run() - Total time: %7.3fsec  Total logs: %d  Rate: %10.3flogs/sec\n",
                timer.time(),
                static_cast<unsigned int>(thread_count * log_count),
                (thread_count * log_count) / timer.time());

         if (!logtk::stop_logger())
         {
            std::cerr << "logtk::test::run() - Failed to stop logger thread!" << std::endl;
         }
      }

   } // namespace test

} // namespace logtk
