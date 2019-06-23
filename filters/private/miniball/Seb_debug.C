// Synopsis: A simple class to save debugging comments to files
//
// Authors: Martin Kutz <kutz@math.fu-berlin.de>,
//          Kaspar Fischer <kf@iaeth.ch>

#include <cmath>
#include <sys/resource.h>
#include <sys/time.h>

#include "Seb_configure.h"

namespace SEB_NAMESPACE {
  
  // Implementation of class Logger:
  
  Logger::Logger()
  {
  }
  
  Logger::~Logger()
  {
    // we walk through the list of all opened files and close them:
    for (Streams::iterator it = channels.begin();
         it != channels.end(); ++it) {
      (*it).second->close();
      delete (*it).second;
    }
  }
  
  Logger& Logger::instance()
  {
    // Here's where we maintain the only instance: (Notice that it
    // gets constructed automatically the first time instance() is
    // called, and that it gets disposed of (if ever contructed) at
    // program termination.)
    static Logger instance;
    return instance;
  }
  
  void Logger::log(const char* ch,const std::string& msg)
  {
    const std::string name(ch);
    Streams::iterator it = channels.find(name);
    
    // have we already opened this file?
    if (it != channels.end()) {
      // If so, then just append the message:
      *(*it).second << msg;
      (*it).second->flush();
      
    } else {
      // If we haven't seen 'name' before, we create a new file:
      using std::ofstream;
      ofstream *o = new ofstream((name+".log").c_str(),
                                 ofstream::out|ofstream::trunc);
      channels[name] = o;
      *o << msg;
    }
  }
  
  // Implementation of class Timer:
  
  // The following routine is taken from file mptimeval.h from
  // "Matpack Library Release 1.7.1" which is copyright (C) 1991-2002
  // by Berndt M. Gammel.  It works on the timeval struct defined in
  // sys/time.h:
  //
  //       struct timeval {
  //         long tv_sec;        /* seconds */
  //         long tv_usec;       /* microseconds */
  //       };
  //
  inline timeval& operator-=(timeval &t1,const timeval &t2)
  {
    t1.tv_sec -= t2.tv_sec;
    if ( (t1.tv_usec -= t2.tv_usec) < 0 ) {
      --t1.tv_sec;
      t1.tv_usec += 1000000;
    }
    return t1;
  }
  
  Timer::Timer()
  {
  }
  
  Timer& Timer::instance()
  {
    // Here's where we maintain the only instance: (Notice that it
    // gets constructed automatically the first time instance() is
    // called, and that it gets disposed of (if ever contructed) at
    // program termination.)
    static Timer instance;
    return instance;
  }
  
  void Timer::start(const char *timer_name)
  {
    // fetch current usage:
    rusage now;
    int status = getrusage(RUSAGE_SELF,&now);
    SEB_ASSERT(status == 0);
    
    // save it:
    timers[std::string(timer_name)] = now.ru_utime;
  }
  
  float Timer::lapse(const char *name)
  {
    // assert that start(name) has been called before:
    SEB_ASSERT(timers.find(std::string(name)) != timers.end());
    
    // get current usage:
    rusage now;
    int status = getrusage(RUSAGE_SELF,&now);
    SEB_ASSERT(status == 0);
    
    // compute elapsed usage:
    now.ru_utime -= (*timers.find(std::string(name))).second;
    return now.ru_utime.tv_sec + now.ru_utime.tv_usec * 1e-6;
  }
  
} // namespace SEB_NAMESPACE
