#ifndef __TRAITS_H__
#define __TRAITS_H__

#include <string>
#include <memory>

template <typename T>
struct HasSharedPtr
{
    typedef std::shared_ptr<T> ptr_type;
};

#endif // __TRAITS_H__
