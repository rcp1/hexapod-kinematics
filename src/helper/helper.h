#ifndef HELPER_H
#define HELPER_H

#include <WString.h>
#include <stdint.h>

template<typename t_type>
inline
void swap(t_type& first, t_type& second)
{
    t_type temp(first);
    first = second;
    second = temp;
}

template<typename t_type>
inline
void swap(t_type* first, t_type* second, uint16_t size)
{
    for (uint16_t i = 0; i < size; ++i)
    {
        swap<t_type>(first[i], second[i]);
    }
}

template<typename t_type>
inline
t_type minimum(t_type val1, t_type val2)
{
    return ((val1 <= val2) ? val1 : val2);
}

template<typename t_type>
inline
t_type maximum(t_type val1, t_type val2)
{
    return ((val1 >= val2) ? val1 : val2);
}

template<typename t_type>
inline
float limit(t_type lower, t_type value, t_type upper)
{
    return maximum(minimum(value, upper), lower);
}

template<typename TYPE>
void shiftArrayLeft(TYPE* array_t, uint8_t size)
{
    TYPE lastElement = array_t[0];
    memmove(&array_t[0], &array_t[1], size - 1);
    array_t[size - 1] = lastElement;
}

template<typename t_type>
void shiftArrayRight(t_type* array_t, uint8_t size)
{
    t_type lastElement = array_t[size - 1];
    memmove(&array_t[1], &array_t[0], size - 1);
    array_t[0] = lastElement;
}

#endif // HELPER_H
