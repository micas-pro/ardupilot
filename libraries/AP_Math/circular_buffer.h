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

    size_t size() const
    {
        return _q.size();
    }

    void add(const T x)
    {
        _q.push_back(x);
        while (_q.size() > _expected_size) {
            _q.pop_front();
        }
    }

    void remove_last() 
    {
        if (_q.size() > 0) {
            _q.pop_back();
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

    // returns [x(k-n+1), x(k-n+2), ..., x(k-1), x(k)]
    //         [the oldest       -->      most recent]
    void get_last_n_items(T buff[], const size_t n) const
    {
        size_t nn = n;
        typename deque<T>::const_reverse_iterator it = _q.rbegin();
        for (size_t i = n-1; it != _q.rend() && nn-- > 0; it++, i--) {
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

    T& get_last_item()
    {
        return _q.back();
    }

private:
    size_t _expected_size;
    deque<T> _q;
};
