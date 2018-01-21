#ifndef LOGGING_H_
#define LOGGING_H_

#ifdef FENGHUI_DEBUG
  #include <iostream>
  #define LOG(a) std::cout << a
  #define LOGLINE(a) std::cout << a << std::endl
#else
  #define LOG(a)
  #define LOGLINE(a)
#endif

#endif
