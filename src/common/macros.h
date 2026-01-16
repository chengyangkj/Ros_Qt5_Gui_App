#pragma once

#include <mutex>
#include <memory>

#define CLASS_TYPE_TRAITS(name,func)                              \
 template<typename T>                                             \
  struct name{                                                    \
       template<typename Class>                                   \
       static constexpr bool Test(decltype(&Class::func)* type){  \
        return true;                                              \
       }                                                          \
                                                                  \
       template<typename>                                         \
       static constexpr bool Test(...){                           \
        return false;                                             \
       }                                                          \
      static inline constexpr bool value = Test<T>(nullptr);     \
  };


CLASS_TYPE_TRAITS(HasShutdown,Shutdown)    

template<typename T>
typename std::enable_if<HasShutdown<T>::value,void>::type CallShutDown(T* instance){
  if (instance!=nullptr) 
  {
    instance->Shutdown();
  }
}

template<typename T>
typename std::enable_if<!HasShutdown<T>::value,void>::type CallShutDown(T* instance){
  (void)instance;
}


#define DISABLE_COPY_AND_ASSIGN(class_name) \
  class_name(const class_name&) = delete; \
  class_name& operator=(const class_name&) = delete; 

#define DEFINE_SINGLETON(class_name)                                        \
  public:                                                                   \
    static class_name* Instance(bool create_if_not_exist = true) {          \
        static class_name* instance=nullptr;                                \
        if (create_if_not_exist && instance == nullptr) {                   \
        static std::once_flag once;                                         \
        std::call_once(once,[&](){instance = new class_name();});           \
        }                                                                   \
        return instance;                                                    \
    }                                                                       \
    static void Clear(){                                                    \
      auto instance = Instance(false);                                      \
      if (instance!=nullptr) {                                              \
        CallShutDown(instance);                                             \
        delete instance;                                                    \
        instance = nullptr;                                                 \
      }                                                                     \
    }                                                                       \
  private:                                                                  \
    class_name();                                                           \
    DISABLE_COPY_AND_ASSIGN(class_name);
