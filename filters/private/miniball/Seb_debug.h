// Synopsis: A simple class to save debugging comments to files
//
// Authors: Martin Kutz <kutz@math.fu-berlin.de>,
//          Kaspar Fischer <kf@iaeth.ch>

#ifndef SEB_DEBUG_H
#define SEB_DEBUG_H

#include <fstream>
#include <map>
#include <string>
#include <sys/resource.h>
#include <sys/time.h>

namespace SEB_NAMESPACE {
  
  class Logger
  // A singleton class which sends debugging comments to log files.
  // (A class is called a "singleton" if at most one instance of it
  // may ever exist.)  By calling void log(channel,msg) you can send
  // the string msg to the file with name channel.
  {
  private: // (Construction and destruction are private to prevent
    // more than one instantiation.)
    
    Logger();
    Logger(const Logger&);
    Logger& operator=(const Logger&);
    ~Logger();
    
  public: // access and routines:
    
    static Logger& instance();
    // Returns a reference to the only existing instance of this class:
    
    void log(const char *channel,const std::string& msg);
    // If this is the first call to log with string channel as the
    // first parameter, then the file with name channel.log is
    // opened (at the beginning) for writing and msg is written to
    // it.  Otherwise, the string msg is opened to the already open
    // file channel.log.
    
  private: // private members:
    typedef std::map<std::string,std::ofstream*> Streams;
    Streams channels;           // a collection of pairs (k,v) where
    // k is the file-name and v is the
    // (open) stream associated with k
  };
  
  class Timer
  // A singleton class which maintains a collection of named timers.
  // (A class is called a "singleton" if at most one instance of it
  // may ever exist.)  The following routines are provided:
  //
  // - start(name): If this is the first time start() has been
  //   called with name as the first parameter, then a new timer is
  //   created and started.  Otherwise, the timer with name name is
  //   restarted.
  //
  // - lapse(name): Retuns the number of seconds which have elapsed
  //   since start(name) was called last.
  //   Precondition: start(name) has been called once.
  {
  private: // (Construction and destruction are private to prevent
    // more than one instantiation.)
    
    Timer();
    Timer(const Timer&);
    Timer& operator=(const Timer&);
    
  public: // access and routines:
    
    static Timer& instance();
    // Returns a reference to the only existing instance of this class:
    
    void start(const char *name);
    float lapse(const char *name);
    
  private: // private members:
    typedef std::map<std::string,timeval> Timers;
    Timers timers;              // a collection of pairs (k,v) where
    // k is the timer name and v is the
    // (started) timer associated with k
  };
  
} // namespace SEB_NAMESPACE

#endif // SEB_DEBUG_H
