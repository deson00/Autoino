#ifndef BIG_INTEGER_H
#define BIG_INTEGER_H

#include <stdint.h>

class BigInteger {
public:
  BigInteger();
  BigInteger(uint32_t num);
  BigInteger(uint64_t num);
  BigInteger(const char* str);
  BigInteger(const BigInteger& other);

  BigInteger& operator=(const BigInteger& other);

  BigInteger operator+(const BigInteger& other) const;
  BigInteger operator-(const BigInteger& other) const;
  BigInteger operator*(const BigInteger& other) const;
  BigInteger operator/(const BigInteger& other) const;
  BigInteger operator%(const BigInteger& other) const;

  bool operator==(const BigInteger& other) const;
  bool operator!=(const BigInteger& other) const;
  bool operator<(const BigInteger& other) const;
  bool operator<=(const BigInteger& other) const;
  bool operator>(const BigInteger& other) const;
  bool operator>=(const BigInteger& other) const;

  BigInteger operator-() const;
  BigInteger operator++();
  BigInteger operator++(int);
  BigInteger operator--();
  BigInteger operator--(int);

  BigInteger& operator+=(const BigInteger& other);
  BigInteger& operator-=(const BigInteger& other);
  BigInteger& operator*=(const BigInteger& other);
  BigInteger& operator/=(const BigInteger& other);
  BigInteger& operator%=(const BigInteger& other);

  char* toString() const;
};

#endif // BIG_INTEGER_H
