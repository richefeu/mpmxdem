#ifndef ANY_TYPE_HPP_ADBB2D75
#define ANY_TYPE_HPP_ADBB2D75

// This class any mimics boost::any
// For details, see: http://alpmestan.wordpress.com/2009/10/18/can-i-haz-teh-boostany-c-class-plz/

#include <algorithm>

class any;
template <class T>
T any_cast(any& a); // declaration for being able to make classes friend of it

class value_base
{
public:
	virtual ~value_base() { }
	virtual value_base* clone() const = 0;
};

template <class T>
class value_type : public value_base
{
	T t;
public:
	value_type(const T& t_) : t(t_) { }
	value_base* clone() const
	{
		return new value_type(t);
	}
	friend T any_cast<>(any& a);
};

/// A clone of boost::any
class any
{
	value_base* v;
public:
	any() : v(0) { }
	template <class value_t>
	any(const value_t& v_) : v(new value_type<value_t>(v_)) { }
	any(any const & other) : v(other.v ? other.v->clone() : 0) { }
	any& operator=(const any& other)
	{
		if (&other != this) {
			any copy(other);
			swap(copy);
		}
		return *this;
	}
	void swap(any& other)
	{
		std::swap(v, other.v);
	}
	~any()
	{
		delete v;
	}

	template <class T> friend T any_cast(any& a);
};


/// bad_any_cast exception class
class bad_any_cast : public std::exception
{
public:
	const char* what() const throw()
	{
		return "Bad any_cast exception";
	}
};

template <class T>
T any_cast(any& a)
{
	value_type<T>* v = dynamic_cast<value_type<T>*>(a.v);
	if (v == 0) {
		throw bad_any_cast();
	}
	else {
		return v->t;
	}
}


#endif /* end of include guard: ANY_TYPE_HPP_ADBB2D75 */

