#pragma once

#include <deque>

using namespace std;

template<typename T = float>
class CircularBuffer
{
public:
    CircularBuffer(const size_t &expected_size) :
        _expected_size(expected_size)
    {};
    
    virtual ~CircularBuffer() {}

    void add(const T x)
    {
        _q.push_back(x);
        while (_q.size() > _expected_size) {
            _q.pop_front();
        }
    }

    void replace_last(const T x)
    {
        if (_q.size() > 0) {
            _q.pop_back();
        }

        add(x);
    }

    void write_to(T buff[]) const
    {
        typename deque<T>::const_iterator it = _q.begin();
        for (size_t i = 0; it != _q.end(); it++, i++) {
            buff[i] = *it;
        }
    }

    void clear()
    {
        _q.clear();
    }

    bool ready() const
    {
        return _q.size() >= _expected_size;
    }

private:
    size_t _expected_size;
    deque<T> _q;
};
