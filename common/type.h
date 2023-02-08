#ifndef TYPE_H
#define TYPE_H
#ifdef __cplusplus
#include <string>
#include <typeinfo>

std::string demangle(const char* name);

/*!
 * \brief type returns a string based on the typename of T
 * \return A string representation of the typename
 */
template <class T>
std::string type(const T& t) {

    return demangle(typeid(t).name());
}

#endif
#endif