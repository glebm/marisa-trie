#ifndef MARISA_GRIMOIRE_VECTOR_VECTOR_H_
#define MARISA_GRIMOIRE_VECTOR_VECTOR_H_

#include <type_traits>
#include <utility>
#include <vector>

#include "marisa/grimoire/io.h"

namespace marisa::grimoire::vector {

template <typename T>
class Vector {
 public:
  // These assertions are repeated for clarity/robustness where the property
  // is used.
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(std::is_trivially_destructible_v<T>);

  Vector() = default;
  // `T` is trivially destructible, so default destructor is ok.
  ~Vector() = default;

  Vector(const Vector<T> &other) {
    size_ = other.size_;
    if (other.fixed()) {
      const_objs_ = other.const_objs_;
    } else {
      owned_objs_ = other.owned_objs_;
      const_objs_ = owned_objs_.data();
    }
  }

  Vector &operator=(const Vector<T> &other) {
    size_ = other.size_;
    if (other.fixed()) {
      const_objs_ = other.const_objs_;
    } else {
      owned_objs_ = other.owned_objs_;
      const_objs_ = owned_objs_.data();
    }
    return *this;
  }

  Vector(Vector &&) noexcept = default;
  Vector &operator=(Vector<T> &&) noexcept = default;

  void map(Mapper &mapper) {
    Vector temp;
    temp.map_(mapper);
    swap(temp);
  }

  void read(Reader &reader) {
    Vector temp;
    temp.read_(reader);
    swap(temp);
  }

  void write(Writer &writer) const {
    write_(writer);
  }

  void push_back(const T &x) {
    MARISA_DEBUG_IF(fixed(), MARISA_STATE_ERROR);
    MARISA_DEBUG_IF(size_ == max_size(), MARISA_SIZE_ERROR);
    owned_objs_.push_back(x);
    const_objs_ = owned_objs_.data();
    ++size_;
  }

  void pop_back() {
    MARISA_DEBUG_IF(fixed(), MARISA_STATE_ERROR);
    MARISA_DEBUG_IF(size_ == 0, MARISA_STATE_ERROR);
    owned_objs_.pop_back();
    const_objs_ = owned_objs_.data();
    --size_;
  }

  // resize() assumes that T's placement new does not throw an exception.
  void resize(std::size_t size) {
    MARISA_DEBUG_IF(fixed(), MARISA_STATE_ERROR);
    owned_objs_.resize(size);
    const_objs_ = owned_objs_.data();
    size_ = size;
  }

  // resize() assumes that T's placement new does not throw an exception.
  void resize(std::size_t size, const T &x) {
    MARISA_DEBUG_IF(fixed(), MARISA_STATE_ERROR);
    owned_objs_.resize(size, x);
    const_objs_ = owned_objs_.data();
    size_ = size;
  }

  void reserve(std::size_t capacity) {
    MARISA_DEBUG_IF(fixed(), MARISA_STATE_ERROR);
    owned_objs_.reserve(capacity);
    const_objs_ = owned_objs_.data();
  }

  void shrink() {
    MARISA_THROW_IF(fixed(), MARISA_STATE_ERROR);
    owned_objs_.shrink_to_fit();
    const_objs_ = owned_objs_.data();
  }

  const T *begin() const {
    return const_objs_;
  }
  const T *end() const {
    return const_objs_ + size_;
  }
  const T &operator[](std::size_t i) const {
    MARISA_DEBUG_IF(i >= size_, MARISA_BOUND_ERROR);
    return const_objs_[i];
  }
  const T &front() const {
    MARISA_DEBUG_IF(size_ == 0, MARISA_STATE_ERROR);
    return const_objs_[0];
  }
  const T &back() const {
    MARISA_DEBUG_IF(size_ == 0, MARISA_STATE_ERROR);
    return const_objs_[size_ - 1];
  }

  T *begin() {
    MARISA_DEBUG_IF(fixed(), MARISA_STATE_ERROR);
    return owned_objs_.data();
  }
  T *end() {
    MARISA_DEBUG_IF(fixed(), MARISA_STATE_ERROR);
    return owned_objs_.data() + size_;
  }
  T &operator[](std::size_t i) {
    MARISA_DEBUG_IF(fixed(), MARISA_STATE_ERROR);
    MARISA_DEBUG_IF(i >= size_, MARISA_BOUND_ERROR);
    return owned_objs_[i];
  }
  T &front() {
    MARISA_DEBUG_IF(fixed(), MARISA_STATE_ERROR);
    MARISA_DEBUG_IF(size_ == 0, MARISA_STATE_ERROR);
    return owned_objs_.front();
  }
  T &back() {
    MARISA_DEBUG_IF(fixed(), MARISA_STATE_ERROR);
    MARISA_DEBUG_IF(size_ == 0, MARISA_STATE_ERROR);
    return owned_objs_.back();
  }

  std::size_t size() const {
    return size_;
  }
  std::size_t capacity() const {
    return owned_objs_.capacity();
  }
  bool fixed() const {
    return owned_objs_.empty() && size_ != 0;
  }

  bool empty() const {
    return size_ == 0;
  }
  std::size_t total_size() const {
    return sizeof(T) * size_;
  }
  std::size_t io_size() const {
    return sizeof(UInt64) + ((total_size() + 7) & ~(std::size_t)0x07);
  }

  void clear() {
    Vector().swap(*this);
  }
  void swap(Vector &rhs) {
    owned_objs_.swap(rhs.owned_objs_);
    marisa::swap(const_objs_, rhs.const_objs_);
    marisa::swap(size_, rhs.size_);
  }

  static std::size_t max_size() {
    return MARISA_SIZE_MAX / sizeof(T);
  }

 private:
  std::vector<T> owned_objs_;
  const T *const_objs_ = nullptr;
  std::size_t size_ = 0;

  void map_(Mapper &mapper) {
    UInt64 total_size;
    mapper.map(&total_size);
    MARISA_THROW_IF(total_size > MARISA_SIZE_MAX, MARISA_SIZE_ERROR);
    MARISA_THROW_IF((total_size % sizeof(T)) != 0, MARISA_FORMAT_ERROR);
    const std::size_t size = (std::size_t)(total_size / sizeof(T));
    mapper.map(&const_objs_, size);
    mapper.seek((std::size_t)((8 - (total_size % 8)) % 8));
    size_ = size;
  }
  void read_(Reader &reader) {
    UInt64 total_size;
    reader.read(&total_size);
    MARISA_THROW_IF(total_size > MARISA_SIZE_MAX, MARISA_SIZE_ERROR);
    MARISA_THROW_IF((total_size % sizeof(T)) != 0, MARISA_FORMAT_ERROR);
    const std::size_t size = (std::size_t)(total_size / sizeof(T));
    resize(size);
    reader.read(owned_objs_.data(), size);
    const_objs_ = owned_objs_.data();
    reader.seek((std::size_t)((8 - (total_size % 8)) % 8));
  }
  void write_(Writer &writer) const {
    writer.write((UInt64)total_size());
    writer.write(const_objs_, size_);
    writer.seek((8 - (total_size() % 8)) % 8);
  }
};

}  // namespace marisa::grimoire::vector

#endif  // MARISA_GRIMOIRE_VECTOR_VECTOR_H_
