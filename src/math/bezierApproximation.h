#ifndef BEZIERAPPROXIMATION_H
#define BEZIERAPPROXIMATION_H

#include <stdint.h>

template<typename t_type, uint8_t t_countAnchors>
class BezierApproximation
{
public:
	BezierApproximation();
	virtual ~BezierApproximation();
    BezierApproximation<t_type, t_countAnchors>& operator=(const BezierApproximation<t_type, t_countAnchors>& source);
    t_type getPos(float actualTime) const;
	void addAnchor(const Vector3d& newAnchor);
	void addAnchor(const Orientation3d& newAnchor);
    void reset();
private:
    t_type m_anchorPoints_ast[t_countAnchors];
	uint8_t m_countUsedAnchors;
};

template<typename t_type, uint8_t t_countAnchors>
BezierApproximation<t_type, t_countAnchors>::BezierApproximation() : m_countUsedAnchors(0)
{
}

template<typename t_type, uint8_t t_countAnchors>
BezierApproximation<t_type, t_countAnchors>::~BezierApproximation()
{
}

template<typename t_type, uint8_t t_countAnchors>
BezierApproximation<t_type, t_countAnchors>&
BezierApproximation<t_type, t_countAnchors>::operator=(const BezierApproximation<t_type, t_countAnchors>& source)
{
    this->m_countUsedAnchors = source.m_countUsedAnchors;
    for (uint8_t i = 0; i < m_countUsedAnchors; ++i)
    {
        this->m_anchorPoints_ast[i] = source.m_anchorPoints_ast[i];
    }

    return *this;
}

template<typename t_type, uint8_t t_countAnchors>
void BezierApproximation<t_type, t_countAnchors>::addAnchor(const Vector3d& newAnchor)
{
    m_anchorPoints_ast[m_countUsedAnchors] = newAnchor;
    m_countUsedAnchors++;
}

template<typename t_type, uint8_t t_countAnchors>
void BezierApproximation<t_type, t_countAnchors>::addAnchor(const Orientation3d& newAnchor)
{
    m_anchorPoints_ast[m_countUsedAnchors] = newAnchor;
    m_countUsedAnchors++;
}

template<typename t_type, uint8_t t_countAnchors>
void BezierApproximation<t_type, t_countAnchors>::reset()
{
    m_countUsedAnchors = 0;
}

template<typename t_type, uint8_t t_countAnchors>
t_type BezierApproximation<t_type, t_countAnchors>::getPos(float actualTime) const
{
    t_type approxPos_ast[t_countAnchors];

    for (uint8_t i = 0; i < m_countUsedAnchors; ++i)
    {
        approxPos_ast[i] = m_anchorPoints_ast[i];
    }

    // double iteration for the mid point between all anchor points at t
    for (uint8_t i = 0; i < m_countUsedAnchors - 1; ++i)
    {
        for (uint8_t j = 0; j < m_countUsedAnchors - i - 1; ++j)
        {
            float diffTime = (1.0f - actualTime);
            approxPos_ast[j] = t_type(diffTime * approxPos_ast[j][0] + actualTime * approxPos_ast[j + 1][0],
                                      diffTime * approxPos_ast[j][1] + actualTime * approxPos_ast[j + 1][1],
                                      diffTime * approxPos_ast[j][2] + actualTime * approxPos_ast[j + 1][2]);
        }
    }

    return approxPos_ast[0];
}

#endif // BEZIERAPPROXIMATION_H
