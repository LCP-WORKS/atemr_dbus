#ifndef ATEMR_EXCEPTION_H
#define ATEMR_EXCEPTION_H

#include <sstream>
#include <string>
#include <exception>

#define DBUS_ERROR 0000
#define FL_ENGINE_ERROR 0010

#define ATEMR_EXCP(code, msg) throw ATEMRException(code, msg, __FILE__, __LINE__)

class ATEMRException : public std::exception
{
public:
  ATEMRException(const int code, const std::string msg,
                             const char* fileName,
                             const std::size_t line)
  {
    stream_.str("");
    code_ = code;
    stream_ << std::to_string(code) << " | " << msg << ", file " << fileName << " line " << line;
  }

  const int code() const noexcept{
    return code_;
  }

  virtual const char* what() const noexcept {
         return stream_.str().c_str();
      }

private:
  std::ostringstream stream_;
  int code_;
};

#endif // ATEMR_EXCEPTION_H
