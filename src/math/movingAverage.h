#ifndef MOVINGAVERAGE_H
#define MOVINGAVERAGE_H

#include <stdint.h>

template<uint8_t t_size>
class MovingAverage
{
public:
    MovingAverage();

    ~MovingAverage();

    void clear();

    void addValue(const float& value);

    void fillValue(const float& value, const uint8_t& countOfAdds);

    float getAverage();

    float getElement(const uint32_t& elementNr);

    uint8_t getSize()
    {
        return t_size;
    }
    uint8_t getCount()
    {
        return m_count;
    }

private:
    uint8_t m_count;
    uint8_t m_actIndex;
    float m_sum;
    float m_data_af[t_size];
};

template<uint8_t t_size>
MovingAverage<t_size>::MovingAverage()
{
    clear();
}

template<uint8_t t_size>
MovingAverage<t_size>::~MovingAverage()
{
}

template<uint8_t t_size>
void MovingAverage<t_size>::clear()
{
    m_count = 0;
    m_actIndex = 0;
    m_sum = 0.0;
    for (int i = 0; i < t_size; ++i)
    {
        m_data_af[i] = 0.0;
    }
}

template<uint8_t t_size>
void MovingAverage<t_size>::addValue(const float& value)
{
    m_sum -= m_data_af[m_actIndex];
    m_data_af[m_actIndex] = value;
    m_sum += m_data_af[m_actIndex];
    m_actIndex++;

    if (m_actIndex == t_size)
    {
        m_actIndex = 0;  // faster than %
    }

    if (m_count < t_size)
    {
        m_count++;
    }
}

template<uint8_t t_size>
float MovingAverage<t_size>::getAverage()
{
    if (m_count == 0)
    {
        return 0.0f;
    }

    return m_sum / (float)m_count;
}

template<uint8_t t_size>
float MovingAverage<t_size>::getElement(const uint32_t& elementNr)
{
    if (elementNr >= m_count)
    {
        return 0.0f;
    }

    return m_data_af[elementNr];
}

template<uint8_t t_size>
void MovingAverage<t_size>::fillValue(const float& value, const uint8_t& countOfAdds)
{
    clear();
    for (int i = 0; i < countOfAdds; i++)
    {
        addValue(value);
    }
}

#endif
